#include "drive.h"
#include "drive_control.h"
#include "drive_math.h"
#include "motor.h"
#include "debug.h"

#include <math.h>

volatile DebugMonitor_t dbg;

typedef struct
{
    float integral;
    float previous_progress;
    uint32_t previous_ms;
    uint8_t previous_valid;
} TurnPidContext;

typedef struct
{
    DriveState state;
    uint32_t state_since_ms;
    uint32_t launch_since_ms;
    uint32_t spin_since_ms;
    uint32_t stuck_since_ms;

    uint8_t turn_right;
    uint8_t swap_count;
    uint8_t reverse_count;
    uint8_t clear_count;
    uint8_t front_near_count;
    uint8_t corner_count;
    int8_t corner_candidate;
    uint8_t grid_clear_count;
    uint8_t side_clear_count;
    uint8_t corner_tight;
    uint8_t launching;
    uint8_t last_full_imu_live;

    float turn_last_heading;
    float turn_accum_deg;
    float heading_ref;
    float stuck_heading;
    float course_heading;
    float course_zero;
    uint8_t entry_heading_valid;
    uint8_t course_latched;
    uint8_t course_reacquire_pending;

    int16_t motion_left;
    int16_t motion_right;
    uint32_t motion_ms;
    uint8_t motion_valid;

    TurnPidContext turn_pid;
} DriveContext;

static DriveContext drive = {
    .state = DS_CRUISE,
    .launching = 1U,
    .corner_candidate = -1,
};

static uint8_t increment_u8(uint8_t value)
{
    return (value < 255U) ? (uint8_t)(value + 1U) : value;
}

/* A short echo dropout must not erase the last near-wall observation. */
static uint8_t front_recent_below(const DriveInputs *in, uint16_t cm)
{
    if (in->f_valid) return (uint8_t)(in->f < cm);
    return (uint8_t)(in->front_miss > 0U
        && in->front_miss < FRONT_FAIL_LIMIT
        && in->f < cm);
}

static uint8_t wide_corner_context(const DriveInputs *in)
{
    return (uint8_t)(drive_corridor_class((float)in->l, (float)in->r)
            == DRIVE_CORRIDOR_WIDE
        && in->l >= CORNER_WIDE_NEAR_CM
        && in->r >= CORNER_WIDE_NEAR_CM);
}

static uint8_t strong_side_corner(const DriveInputs *in, int8_t direction)
{
    if (direction < 0 || !in->left_valid || !in->right_valid) return 0U;

    uint16_t open = (direction == 0) ? in->l : in->r;
    uint16_t near = (direction == 0) ? in->r : in->l;
    uint8_t wide = wide_corner_context(in);
    uint16_t open_min = wide
        ? CORNER_SIDE_ONLY_WIDE_OPEN_CM
        : CORNER_SIDE_ONLY_OPEN_CM;
    uint16_t asym_min = wide
        ? CORNER_SIDE_ONLY_WIDE_ASYM_CM
        : CORNER_SIDE_ONLY_ASYM_CM;

    return (uint8_t)(open >= open_min
        && near <= CORNER_SIDE_ONLY_NEAR_CM
        && open >= (uint16_t)(near + asym_min));
}

/* testtrack.drawio ends with an eastbound-to-southbound right turn before the
 * speed bump. Its side ranges can be symmetric, so geometry alone has no turn
 * direction. Restrict this fallback to that course axis and a confirmed wall. */
static int8_t symmetric_course_direction(const DriveInputs *in)
{
    if (!drive.course_latched || !in->imu_live
        || !in->left_valid || !in->right_valid
        || !front_recent_below(in, FRONT_STOP_CM))
        return -1;

    uint16_t side_diff = (in->l >= in->r) ? (in->l - in->r) : (in->r - in->l);
    if (side_diff > CORNER_SYMMETRIC_MAX_DIFF_CM) return -1;

    float relative_heading = drive_wrap180(in->heading - drive.course_zero);
    float final_axis_error = drive_wrap180(relative_heading - COURSE_FINAL_ENTRY_DEG);
    return (fabsf(final_axis_error) <= COURSE_FINAL_ENTRY_TOL_DEG) ? 1 : -1;
}

static int16_t clamp_pct(int16_t value)
{
    if (value > 100) return 100;
    if (value < -100) return -100;
    return value;
}

static int16_t approach_i16(int16_t current, int16_t target, int16_t step)
{
    if (current < target)
    {
        int32_t next = (int32_t)current + step;
        return (next > target) ? target : (int16_t)next;
    }
    if (current > target)
    {
        int32_t next = (int32_t)current - step;
        return (next < target) ? target : (int16_t)next;
    }
    return current;
}

/* Motion states share one time-based wheel ramp. Brake/stop bypass it. */
static void motion_command(int16_t left_pct, int16_t right_pct, uint32_t now)
{
    left_pct = clamp_pct(left_pct);
    right_pct = clamp_pct(right_pct);

    if (!drive.motion_valid)
    {
        drive.motion_left = 0;
        drive.motion_right = 0;
        drive.motion_ms = now - DRIVE_NOMINAL_UPDATE_MS;
        drive.motion_valid = 1U;
    }

    uint32_t elapsed_ms = now - drive.motion_ms;
    if (elapsed_ms > 100U) elapsed_ms = 100U;
    int16_t step = (int16_t)((DRIVE_WHEEL_SLEW_PCT_PER_S * (float)elapsed_ms / 1000.0f) + 0.5f);
    if (elapsed_ms > 0U && step < 1) step = 1;
    drive.motion_left = approach_i16(drive.motion_left, left_pct, step);
    drive.motion_right = approach_i16(drive.motion_right, right_pct, step);

    drive.motion_ms = now;
    Motor_SetWheels(drive.motion_left, drive.motion_right);
    dbg.duty_l = drive.motion_left;
    dbg.duty_r = drive.motion_right;
}

static void motion_command_immediate(int16_t left_pct, int16_t right_pct, uint32_t now)
{
    drive.motion_left = clamp_pct(left_pct);
    drive.motion_right = clamp_pct(right_pct);
    drive.motion_ms = now;
    drive.motion_valid = 1U;
    Motor_SetWheels(drive.motion_left, drive.motion_right);
    dbg.duty_l = drive.motion_left;
    dbg.duty_r = drive.motion_right;
}

static void motion_stop(uint32_t now)
{
    Car_Stop();
    drive.motion_left = 0;
    drive.motion_right = 0;
    drive.motion_ms = now;
    drive.motion_valid = 1U;
    dbg.duty_l = 0;
    dbg.duty_r = 0;
    dbg.steer = 0.0f;
}

static void motion_brake(uint32_t now)
{
    Car_Brake();
    drive.motion_left = 0;
    drive.motion_right = 0;
    drive.motion_ms = now;
    drive.motion_valid = 1U;
    dbg.duty_l = 0;
    dbg.duty_r = 0;
    dbg.steer = 0.0f;
}

static void command_arc(uint8_t outer, uint8_t inner, uint32_t now)
{
    int16_t delta = (int16_t)((outer - inner) / 2U);
    if (drive.turn_right)
    {
        motion_command((int16_t)outer, (int16_t)inner, now);
        dbg.steer = -(float)delta;
    }
    else
    {
        motion_command((int16_t)inner, (int16_t)outer, now);
        dbg.steer = (float)delta;
    }
}

static void command_pivot(uint8_t outer, uint8_t inner, uint32_t now, uint8_t immediate)
{
    int16_t left = drive.turn_right ? (int16_t)outer : -(int16_t)inner;
    int16_t right = drive.turn_right ? -(int16_t)inner : (int16_t)outer;
    if (immediate) motion_command_immediate(left, right, now);
    else motion_command(left, right, now);

    if (drive.turn_right)
        dbg.steer = (float)(-((int16_t)outer + (int16_t)inner) / 2);
    else
        dbg.steer = (float)(((int16_t)outer + (int16_t)inner) / 2);
}

static void state_enter(DriveState state, uint32_t now)
{
    drive.state = state;
    drive.state_since_ms = now;
}

static float course_grid_snap(float heading)
{
    if (!drive.course_latched) return heading;
    float index = roundf(drive_wrap180(heading - drive.course_zero) / 45.0f);
    return drive_wrap360(drive.course_zero + (index * 45.0f));
}

/* Signed turn progress accumulated frame by frame. A single
 * wrap180(heading - entry) cannot represent rotations past 180 deg — it wraps
 * to -180 and reads as "turning the wrong way", flipping the spin direction
 * mid-search. Accumulation keeps the angle unbounded and monotonic. */
static void turn_progress_reset(float heading)
{
    drive.turn_accum_deg = 0.0f;
    drive.turn_last_heading = heading;
}

static float turn_progress_update(float heading)
{
    float delta = drive_wrap180(heading - drive.turn_last_heading);
    drive.turn_last_heading = heading;
    drive.turn_accum_deg += drive.turn_right ? delta : -delta;
    return drive.turn_accum_deg;
}

static void turn_pid_reset(void)
{
    drive.turn_pid = (TurnPidContext){0};
}

static void turn_pid_run(const DriveInputs *in, float progress)
{
    float dt = 0.02f;
    if (drive.turn_pid.previous_ms != 0U && in->now > drive.turn_pid.previous_ms)
        dt = drive_clampf((float)(in->now - drive.turn_pid.previous_ms) / 1000.0f, 0.005f, 0.100f);

    float error = TURN_TARGET_DEG - progress;
    if (error < 0.0f) error = 0.0f;
    if (error < 45.0f)
    {
        drive.turn_pid.integral += error * dt;
        drive.turn_pid.integral = drive_clampf(drive.turn_pid.integral,
            -TURN_PID_I_MAX, TURN_PID_I_MAX);
    }
    else
    {
        drive.turn_pid.integral = 0.0f;
    }

    float rate = drive.turn_pid.previous_valid
        ? (progress - drive.turn_pid.previous_progress) / dt
        : 0.0f;
    float command = (TURN_PID_KP * error)
                  + (TURN_PID_KI * drive.turn_pid.integral)
                  - (TURN_PID_KD * rate);
    float minimum = (error <= TURN_PID_FINE_DEG)
        ? (float)TURN_PID_FINE_MIN_PCT
        : (float)TURN_PID_MIN_PCT;
    command = drive_clampf(command, minimum, (float)TURN_SPEED);

    uint8_t outer = (uint8_t)(command + 0.5f);
    /* The inner pair reverses against the skid; below the stall floor it
     * freezes and the rotation centre drifts onto the stopped side. */
    uint8_t inner = (uint8_t)((command * TURN_PID_INNER_RATIO) + 0.5f);
    if (inner < MOTOR_MIN_PCT) inner = MOTOR_MIN_PCT;
    if (inner > TURN_INNER) inner = TURN_INNER;
    command_pivot(outer, inner, in->now, 0U);

    drive.turn_pid.previous_progress = progress;
    drive.turn_pid.previous_valid = 1U;
    drive.turn_pid.previous_ms = in->now;
}

static void apply_cruise_control(const DriveInputs *in)
{
    DriveControlOutput output = {0};
    float grid_heading = in->imu_live ? course_grid_snap(in->heading) : drive.heading_ref;
    DriveControl_Run(in, drive.heading_ref, grid_heading,
        in->now - drive.state_since_ms, &output);

    drive.heading_ref = output.heading_ref_deg;
    if (output.axis_resnapped) drive.course_heading = output.heading_ref_deg;
    motion_command(output.left_pct, output.right_pct, in->now);

    dbg.steer = output.steer_pct;
    dbg.steer_mode = output.mode;
    dbg.yaw_rate = output.yaw_rate_dps;
    dbg.hdg_err = output.heading_error_deg;
}

static void reset_detection_counts(void)
{
    drive.front_near_count = 0U;
    drive.corner_count = 0U;
    drive.corner_candidate = -1;
    drive.side_clear_count = 0U;
}

static void cruise_enter(const DriveInputs *in)
{
    DriveState previous = drive.state;
    uint8_t previous_was_turn = (uint8_t)(previous == DS_SPIN || previous == DS_CORNER);
    uint8_t from_turn = (uint8_t)(previous_was_turn
        && drive.entry_heading_valid && in->imu_live);
    uint8_t from_avoid = (uint8_t)(previous == DS_SIDE_AVOID
        && in->imu_live && drive.course_latched);
    uint8_t reacquire_course = (uint8_t)(in->imu_live
        && drive.course_reacquire_pending);

    DriveControl_Reset();
    if (from_turn || reacquire_course)
        drive.heading_ref = course_grid_snap(in->heading);
    else if (from_avoid)
        drive.heading_ref = drive.course_heading;
    else if (in->imu_live && drive.course_latched)
        drive.heading_ref = drive.course_heading;
    else if (in->imu_live)
        drive.heading_ref = in->heading;

    if (in->imu_live)
    {
        if (from_turn || reacquire_course)
            drive.course_heading = drive.heading_ref;
        drive.course_reacquire_pending = 0U;
        if (from_turn || from_avoid || reacquire_course)
            DriveControl_PrimeHeading(in->heading);
    }
    else if (previous_was_turn)
    {
        drive.course_reacquire_pending = 1U;
    }

    drive.reverse_count = 0U;
    reset_detection_counts();
    state_enter(DS_CRUISE, in->now);

    if (in->side_valid) apply_cruise_control(in);
    else motion_command((int16_t)CENTER_SETTLE_SPEED_PCT,
                        (int16_t)CENTER_SETTLE_SPEED_PCT, in->now);
}

static void brake_enter(const DriveInputs *in)
{
    reset_detection_counts();
    motion_brake(in->now);
    state_enter(DS_BRAKE, in->now);
}

static void select_spin_direction(const DriveInputs *in, uint8_t tie_flip)
{
    if (in->left_valid && in->right_valid)
    {
        if (in->l > (uint16_t)(in->r + SIDE_HYST)) drive.turn_right = 0U;
        else if (in->r > (uint16_t)(in->l + SIDE_HYST)) drive.turn_right = 1U;
        else
        {
            int8_t course_direction = symmetric_course_direction(in);
            if (course_direction >= 0) drive.turn_right = (uint8_t)course_direction;
            else if (tie_flip) drive.turn_right ^= 1U;
        }
    }
    else if (in->left_valid)
    {
        drive.turn_right = 1U;
    }
    else if (in->right_valid)
    {
        drive.turn_right = 0U;
    }
    else if (tie_flip)
    {
        drive.turn_right ^= 1U;
    }
}

static void spin_enter(const DriveInputs *in, uint8_t tie_flip)
{
    reset_detection_counts();
    select_spin_direction(in, tie_flip);
    drive.swap_count = 0U;
    drive.clear_count = 0U;
    turn_progress_reset(in->heading);
    drive.stuck_heading = in->heading;
    drive.entry_heading_valid = in->imu_live;
    drive.stuck_since_ms = in->now;
    drive.spin_since_ms = in->now;
    turn_pid_reset();
    state_enter(DS_SPIN, in->now);
    command_pivot(TURN_SPEED, TURN_INNER, in->now, 1U);
}

static void reverse_enter(const DriveInputs *in)
{
    reset_detection_counts();
    state_enter(DS_REVERSE, in->now);
    if (drive.reverse_count < 255U) drive.reverse_count++;
    motion_command_immediate(-DRIVE_SPEED, -DRIVE_SPEED, in->now);
}

static void hold_enter(const DriveInputs *in)
{
    reset_detection_counts();
    motion_stop(in->now);
    state_enter(DS_HOLD, in->now);
}

static void side_avoid_enter(const DriveInputs *in)
{
    reset_detection_counts();
    if (in->left_valid && (!in->right_valid || in->l < in->r))
        drive.turn_right = 1U;
    else if (in->right_valid && (!in->left_valid || in->r < in->l))
        drive.turn_right = 0U;
    state_enter(DS_SIDE_AVOID, in->now);
    command_arc(SIDE_ESCAPE_OUTER, SIDE_ESCAPE_INNER, in->now);
}

static void corner_enter(const DriveInputs *in, uint8_t turn_right)
{
    reset_detection_counts();
    drive.grid_clear_count = 0U;
    drive.clear_count = 0U;
    drive.reverse_count = 0U;   /* a confirmed corner is forward progress */
    drive.turn_right = turn_right;
    drive.corner_tight = front_recent_below(in, CORNER_TIGHTEN_CM);
    turn_progress_reset(in->heading);
    drive.entry_heading_valid = in->imu_live;
    turn_pid_reset();
    state_enter(DS_CORNER, in->now);
    if (drive.corner_tight)
        command_pivot(CORNER_RESCUE_OUTER, CORNER_RESCUE_INNER, in->now, 1U);
    else
        command_arc(ARC_OUTER, ARC_INNER, in->now);
}

static uint8_t front_graze_suspected(const DriveInputs *in)
{
    if (!in->left_valid && !in->right_valid) return 0U;

    if (in->left_valid && in->right_valid)
    {
        uint16_t near = (in->l < in->r) ? in->l : in->r;
        uint16_t far = (in->l < in->r) ? in->r : in->l;
        uint8_t wide = wide_corner_context(in);
        uint16_t asymmetry = wide ? CORNER_WIDE_ASYM_CM : CORNER_ASYM_CM;
        uint16_t open = wide ? CORNER_WIDE_ASYM_OPEN_CM : CORNER_ASYM_OPEN_CM;

        if (far >= (uint16_t)(near + asymmetry) && far >= open && near >= CORNER_NEAR_SAFE_CM)
        {
            if (in->imu_live
                && fabsf(drive_wrap180(in->heading - drive.heading_ref)) > CORNER_ENTRY_HDG_MAX_DEG)
                return 1U;
            return 0U;
        }
    }

    uint16_t near = in->left_valid
        ? (in->right_valid && in->r < in->l ? in->r : in->l)
        : in->r;
    if ((float)near <= (float)in->f * FRONT_BEAM_HALF_SIN) return 1U;

    if (in->imu_live)
    {
        float signed_yaw = drive_wrap180(in->heading - drive.heading_ref);
        float yaw = drive_clampf(fabsf(signed_yaw), 0.0f, 45.0f);
        float radians = yaw * 0.0174532925f;
        float beam_cos = sqrtf(1.0f - (FRONT_BEAM_HALF_SIN * FRONT_BEAM_HALF_SIN));
        float effective_sin = (FRONT_BEAM_HALF_SIN * cosf(radians))
                            + (beam_cos * sinf(radians));
        if (signed_yaw > 0.0f && in->right_valid
            && (float)in->r <= (float)in->f * effective_sin)
            return 1U;
        if (signed_yaw <= 0.0f && in->left_valid
            && (float)in->l <= (float)in->f * effective_sin)
            return 1U;
        if (fabsf(signed_yaw) > FRONT_OFFAXIS_DEG) return 1U;
    }
    return 0U;
}

/* Returns -1 for no corner, 0 for left, 1 for right. */
static int8_t corner_direction(const DriveInputs *in)
{
    if (!in->left_valid || !in->right_valid) return -1;

    uint8_t wide = wide_corner_context(in);
    uint16_t side_open = wide ? CORNER_WIDE_OPEN_CM : SIDE_OPEN_CM;
    uint16_t asym_open = wide ? CORNER_WIDE_ASYM_OPEN_CM : CORNER_ASYM_OPEN_CM;
    uint16_t asymmetry = wide ? CORNER_WIDE_ASYM_CM : CORNER_ASYM_CM;

    uint8_t left_open = (uint8_t)(in->l >= side_open && in->r >= CORNER_NEAR_SAFE_CM);
    uint8_t right_open = (uint8_t)(in->r >= side_open && in->l >= CORNER_NEAR_SAFE_CM);
    uint8_t left_asym = (uint8_t)(in->l >= asym_open
        && in->l >= (uint16_t)(in->r + asymmetry) && in->r >= CORNER_NEAR_SAFE_CM);
    uint8_t right_asym = (uint8_t)(in->r >= asym_open
        && in->r >= (uint16_t)(in->l + asymmetry) && in->l >= CORNER_NEAR_SAFE_CM);

    if (left_asym || (left_open && !right_open)) return 0;
    if (right_asym || (right_open && !left_open)) return 1;
    return -1;
}

static uint8_t side_emergency(const DriveInputs *in)
{
    return (uint8_t)((in->left_valid && in->l < SIDE_AVOID_CM)
        || (in->right_valid && in->r < SIDE_AVOID_CM));
}

static uint8_t dead_end(const DriveInputs *in)
{
    return (uint8_t)(in->f_valid && in->f < FRONT_DANGER_CM
        && in->left_valid && in->right_valid
        && in->l < SIDE_BLOCK_CM && in->r < SIDE_BLOCK_CM);
}

/* No echo within the 6 ms budget means nothing within ~100 cm. A persistent
 * timeout therefore counts as open space; otherwise turn exits stay gated
 * shut whenever the car ends up facing an open corridor. */
static uint8_t front_open_at(const DriveInputs *in, uint16_t cm)
{
    if (in->f_valid) return (uint8_t)(in->f >= cm);
    return (uint8_t)(in->front_miss >= FRONT_FAIL_LIMIT);
}

static void cruise_run(const DriveInputs *in)
{
    if (in->front_miss >= FRONT_FAIL_LIMIT && !in->left_valid && !in->right_valid)
    {
        hold_enter(in);
        return;
    }

    if (drive.launching)
    {
        if (drive.launch_since_ms == 0U) drive.launch_since_ms = in->now;
        uint8_t in_window = (uint8_t)((in->now - drive.launch_since_ms) < LAUNCH_MS);
        uint8_t front_warning = front_recent_below(in, FRONT_TURN_CM);
        if (in_window && !front_warning && !side_emergency(in))
        {
            if (in->imu_live)
            {
                drive.heading_ref = course_grid_snap(in->heading);
                DriveControl_PrimeHeading(in->heading);
            }
            motion_command((int16_t)CENTER_SETTLE_SPEED_PCT,
                           (int16_t)CENTER_SETTLE_SPEED_PCT, in->now);
            dbg.launch = 1U;
            dbg.steer = 0.0f;
            return;
        }
        drive.launching = 0U;
        dbg.launch = 0U;
    }

    if (front_recent_below(in, FRONT_DANGER_CM))
    {
        brake_enter(in);
        return;
    }

    uint8_t front_near = front_recent_below(in, FRONT_ARC_CM);
    uint8_t graze = (uint8_t)(front_near ? front_graze_suspected(in) : 0U);
    dbg.graze = graze;

    int8_t direction = corner_direction(in);
    uint8_t symmetric_course_corner = 0U;
    if (direction < 0)
    {
        direction = symmetric_course_direction(in);
        symmetric_course_corner = (uint8_t)(direction >= 0);
    }
    uint8_t side_only_corner = (uint8_t)(!front_near
        && in->front_miss >= FRONT_FAIL_LIMIT
        && strong_side_corner(in, direction));
    if (direction >= 0 && (front_near || side_only_corner))
    {
        if (drive.corner_candidate != direction)
        {
            drive.corner_candidate = direction;
            drive.corner_count = 1U;
        }
        else
        {
            drive.corner_count = increment_u8(drive.corner_count);
        }
        drive.front_near_count = 0U;
        uint8_t confirm_needed = symmetric_course_corner
            ? CORNER_SYMMETRIC_CONFIRM_N
            : (side_only_corner ? CORNER_SIDE_ONLY_CONFIRM_N : CORNER_CONFIRM_N);
        if (drive.corner_count >= confirm_needed)
        {
            corner_enter(in, (uint8_t)direction);
            return;
        }

        /* Do not spend the confirmation frame moving into a nearby or unseen
         * front wall. The next full frame can enter the turn from rest. */
        if (side_only_corner || front_recent_below(in, FRONT_STOP_CM))
            motion_brake(in->now);
        else
            apply_cruise_control(in);
        return;
    }
    drive.corner_count = 0U;
    drive.corner_candidate = -1;

    if (front_recent_below(in, FRONT_STOP_CM))
    {
        brake_enter(in);
        return;
    }

    if (front_recent_below(in, FRONT_TURN_CM))
    {
        drive.front_near_count = increment_u8(drive.front_near_count);
        if (drive.front_near_count >= FRONT_CONFIRM_N)
        {
            dbg.front_near = drive.front_near_count;
            brake_enter(in);
            return;
        }
    }
    else
    {
        drive.front_near_count = 0U;
    }
    dbg.front_near = drive.front_near_count;

    if (side_emergency(in))
    {
        side_avoid_enter(in);
        return;
    }
    apply_cruise_control(in);
}

static void brake_run(const DriveInputs *in)
{
    if ((in->now - drive.state_since_ms) < BRAKE_MS) return;
    motion_stop(in->now);

    /* A turn direction must come from at least one current side sample. */
    if (!in->left_valid && !in->right_valid) return;

    if (in->f_valid && in->f < FRONT_DANGER_CM)
    {
        if (drive.reverse_count < REV_MAX_CHUNKS)
            reverse_enter(in);
        else
            spin_enter(in, 0U);
        return;
    }
    spin_enter(in, (uint8_t)(drive.reverse_count > 0U));
}

static void spin_restart(const DriveInputs *in, uint8_t flip)
{
    if (flip)
    {
        drive.turn_right ^= 1U;
        drive.swap_count = increment_u8(drive.swap_count);
    }
    drive.clear_count = 0U;
    turn_progress_reset(in->heading);
    drive.entry_heading_valid = in->imu_live;
    drive.stuck_heading = in->heading;
    drive.stuck_since_ms = in->now;
    drive.spin_since_ms = in->now;
    turn_pid_reset();
    state_enter(DS_SPIN, in->now);
    command_pivot(TURN_SPEED, TURN_INNER, in->now, 1U);
}

static void spin_run(const DriveInputs *in)
{
    if (in->f_valid && in->f < FRONT_DANGER_CM
        && (in->now - drive.state_since_ms) >= SPIN_COMMIT_MS)
    {
        brake_enter(in);
        return;
    }

    if (dead_end(in) && drive.reverse_count < REV_MAX_CHUNKS)
    {
        reverse_enter(in);
        return;
    }

    float progress = 0.0f;
    uint8_t progress_valid = 0U;
    if (in->imu_live)
    {
        if (!drive.entry_heading_valid)
        {
            turn_progress_reset(in->heading);
            drive.stuck_heading = in->heading;
            drive.stuck_since_ms = in->now;
            drive.entry_heading_valid = 1U;
            turn_pid_reset();
        }
        progress = turn_progress_update(in->heading);
        progress_valid = 1U;
        dbg.spin_deg = progress;

        if ((in->now - drive.state_since_ms) >= SPIN_COMMIT_MS && progress < -TURN_WRONG_DEG)
        {
            spin_restart(in, 1U);
            return;
        }

        if ((in->now - drive.stuck_since_ms) >= ROT_STUCK_MS)
        {
            if (fabsf(drive_wrap180(in->heading - drive.stuck_heading)) < ROT_STUCK_DEG
                && drive.reverse_count < REV_MAX_CHUNKS)
            {
                reverse_enter(in);
                return;
            }
            drive.stuck_heading = in->heading;
            drive.stuck_since_ms = in->now;
        }
    }

    if ((in->now - drive.state_since_ms) > SPIN_MAX_MS)
    {
        float course_deviation_now = in->imu_live
            ? fabsf(drive_wrap180(in->heading - drive.course_heading))
            : 0.0f;
        if (progress_valid && progress >= TURN_MIN_DEG
            && (course_deviation_now < COURSE_REV_DEG || progress >= 180.0f)
            && front_open_at(in, FRONT_TURN_CM))
        {
            /* Turned far enough and the nose is clear — roll out instead of
             * flipping direction and unwinding the turn we just made. */
            cruise_enter(in);
        }
        else if (drive.reverse_count < REV_MAX_CHUNKS)
        {
            reverse_enter(in);
        }
        else
        {
            /* Keep the same direction once the swap budget is spent so the
             * car cannot ping-pong between mirrored half-turns forever. */
            spin_restart(in, (uint8_t)(drive.swap_count < SWAP_LIMIT));
        }
        return;
    }

    if (in->left_valid && in->right_valid
        && (in->now - drive.state_since_ms) >= SPIN_COMMIT_MS
        && drive.swap_count < SWAP_LIMIT)
    {
        uint16_t selected = drive.turn_right ? in->r : in->l;
        uint16_t other = drive.turn_right ? in->l : in->r;
        if (selected < SIDE_BLOCK_CM && other > (uint16_t)(selected + SIDE_HYST))
        {
            spin_restart(in, 1U);
            return;
        }
    }

    /* course_heading is only trustworthy within a half revolution: needing to
     * rotate past 180 deg proves the pocket demanded a course reversal, so the
     * stale reference must not keep every exit gated shut. */
    float course_deviation = in->imu_live
        ? fabsf(drive_wrap180(in->heading - drive.course_heading))
        : 0.0f;
    uint8_t forward_ok = (uint8_t)(!in->imu_live
        || course_deviation < COURSE_REV_DEG
        || (progress_valid && progress >= 180.0f));
    dbg.course_dev = course_deviation;
    dbg.reverse = (uint8_t)(in->imu_live && !forward_ok);

    if (front_open_at(in, FRONT_CLEAR_CM))
    {
        drive.clear_count = increment_u8(drive.clear_count);
        if (drive.clear_count >= CLEAR_CONFIRM && forward_ok
            && (!drive.entry_heading_valid
                || (progress_valid && progress >= TURN_MIN_DEG)))
        {
            cruise_enter(in);
            return;
        }
    }
    else if (in->f_valid)
    {
        drive.clear_count = 0U;
    }

    if (progress_valid && forward_ok
        && progress >= TURN_TARGET_DEG && front_open_at(in, FRONT_TURN_CM))
    {
        cruise_enter(in);
        return;
    }
    /* Past the cutoff angle the pivot becomes a search: keep rotating at the
     * controller floor until the nose actually faces open space — exiting
     * blind here just re-enters BRAKE on the next frame. */
    if (progress_valid && forward_ok && progress >= TURN_CUTOFF_DEG
        && front_open_at(in, FRONT_TURN_CM))
    {
        cruise_enter(in);
        return;
    }
    if (!drive.entry_heading_valid
        && (in->now - drive.spin_since_ms) >= SPIN_BLIND_MS
        && front_open_at(in, FRONT_TURN_CM)
        && (in->left_valid || in->right_valid))
    {
        cruise_enter(in);
        return;
    }

    if (progress_valid) turn_pid_run(in, progress);
    else command_pivot(TURN_SPEED, TURN_INNER, in->now, 0U);
}

static void corner_run(const DriveInputs *in)
{
    if (in->imu_live && !drive.entry_heading_valid)
    {
        turn_progress_reset(in->heading);
        drive.entry_heading_valid = 1U;
    }
    if (front_recent_below(in, CORNER_ABORT_CM))
    {
        brake_enter(in);
        return;
    }
    if (side_emergency(in))
    {
        side_avoid_enter(in);
        return;
    }
    if ((in->now - drive.state_since_ms) > ARC_MAX_MS)
    {
        brake_enter(in);
        return;
    }

    float progress = (in->imu_live && drive.entry_heading_valid)
        ? turn_progress_update(in->heading)
        : 0.0f;
    uint32_t corner_elapsed_ms = in->now - drive.state_since_ms;
    uint8_t front_tight = front_recent_below(in, CORNER_TIGHTEN_CM);
    uint8_t rescue_immediate = 0U;
    if (front_tight)
    {
        rescue_immediate = (uint8_t)(!drive.corner_tight);
        drive.corner_tight = 1U;
    }
    if (in->imu_live && drive.entry_heading_valid
        && corner_elapsed_ms > CORNER_PROGRESS_GRACE_MS)
    {
        float minimum_progress = ((float)(corner_elapsed_ms - CORNER_PROGRESS_GRACE_MS)
            * CORNER_MIN_PROGRESS_DPS) / 1000.0f;
        if ((progress + CORNER_PROGRESS_SLACK_DEG) < minimum_progress)
            drive.corner_tight = 1U;
        if (progress >= CORNER_TIGHT_RELEASE_DEG && !front_tight)
            drive.corner_tight = 0U;
    }
    else if (!drive.entry_heading_valid && front_open_at(in, FRONT_CLEAR_CM))
    {
        drive.corner_tight = 0U;
    }
    if (in->imu_live && drive.entry_heading_valid
        && (in->now - drive.state_since_ms) >= SPIN_COMMIT_MS
        && progress < -TURN_WRONG_DEG)
    {
        brake_enter(in);
        return;
    }

    if (in->imu_live && drive.entry_heading_valid
        && progress >= TURN_TARGET_DEG && front_open_at(in, FRONT_TURN_CM))
    {
        cruise_enter(in);
        return;
    }

    if (front_open_at(in, CORNER_GRID_EXIT_CM))
        drive.grid_clear_count = increment_u8(drive.grid_clear_count);
    else if (in->f_valid)
        drive.grid_clear_count = 0U;
    if (in->imu_live && drive.entry_heading_valid
        && drive.grid_clear_count >= CLEAR_CONFIRM
        && progress >= CORNER_GRID_EXIT_MIN_DEG
        && progress <= CORNER_GRID_EXIT_MAX_DEG
        && fabsf(drive_wrap180(in->heading - course_grid_snap(in->heading))) <= CORNER_GRID_ALIGN_DEG)
    {
        cruise_enter(in);
        return;
    }

    if (front_open_at(in, FRONT_CLEAR_CM))
    {
        drive.clear_count = increment_u8(drive.clear_count);
        if (drive.clear_count >= CLEAR_CONFIRM
            && (!drive.entry_heading_valid
                || (in->imu_live && progress >= CORNER_EXIT_MIN_DEG)))
        {
            cruise_enter(in);
            return;
        }
    }
    else if (in->f_valid)
    {
        drive.clear_count = 0U;
    }

    float remaining = TURN_TARGET_DEG - progress;
    if (remaining < 0.0f) remaining = 0.0f;
    if (drive.corner_tight)
    {
        command_pivot(CORNER_RESCUE_OUTER, CORNER_RESCUE_INNER,
            in->now, rescue_immediate);
        return;
    }

    float outer = (float)ARC_OUTER;
    float inner = (float)ARC_INNER;
    if (in->imu_live && drive.entry_heading_valid && remaining <= ARC_APPROACH_DEG)
    {
        float ratio = remaining / ARC_APPROACH_DEG;
        outer = (float)ARC_APPROACH_OUTER
              + (((float)ARC_OUTER - (float)ARC_APPROACH_OUTER) * ratio);
        inner = (float)ARC_APPROACH_INNER
              + (((float)ARC_INNER - (float)ARC_APPROACH_INNER) * ratio);
    }
    command_arc((uint8_t)(outer + 0.5f), (uint8_t)(inner + 0.5f), in->now);
}

static void reverse_run(const DriveInputs *in)
{
    if ((in->now - drive.state_since_ms) >= BACK_CHUNK_MS)
    {
        brake_enter(in);
        return;
    }
    motion_command(-DRIVE_SPEED, -DRIVE_SPEED, in->now);
}

static void hold_run(const DriveInputs *in)
{
    if (in->f_valid)
    {
        if (in->f < FRONT_STOP_CM) brake_enter(in);
        else cruise_enter(in);
        return;
    }
    if (in->left_valid || in->right_valid)
    {
        /* Side sensing is back; a silent front just means open corridor. */
        cruise_enter(in);
        return;
    }
    motion_stop(in->now);
}

static void side_avoid_run(const DriveInputs *in)
{
    if (in->f_valid && in->f < FRONT_STOP_CM)
    {
        brake_enter(in);
        return;
    }

    uint8_t wall_valid = drive.turn_right ? in->left_valid : in->right_valid;
    uint16_t wall_distance = drive.turn_right ? in->l : in->r;
    if (wall_valid && wall_distance >= SIDE_AVOID_CLEAR_CM)
    {
        drive.side_clear_count = increment_u8(drive.side_clear_count);
        if (drive.side_clear_count >= SIDE_AVOID_CLEAR_CONFIRM)
        {
            cruise_enter(in);
            return;
        }
    }
    else
    {
        drive.side_clear_count = 0U;
    }

    if ((in->now - drive.state_since_ms) > SIDE_AVOID_MAX_MS)
    {
        brake_enter(in);
        return;
    }
    command_arc(SIDE_ESCAPE_OUTER, SIDE_ESCAPE_INNER, in->now);
}

void Drive_Init(void)
{
    drive = (DriveContext){
        .state = DS_CRUISE,
        .launching = 1U,
        .corner_candidate = -1,
    };
    DriveControl_Reset();
    dbg.state = DS_CRUISE;
    dbg.turn_dir = 0U;
    dbg.rev_cnt = 0U;
    dbg.launch = 0U;
}

void Drive_Update(const DriveInputs *in)
{
    if (in == NULL)
    {
        motion_stop(0U);
        return;
    }

    if (!drive.course_latched && in->imu_live)
    {
        drive.course_zero = in->heading;
        drive.course_heading = in->heading;
        drive.heading_ref = in->heading;
        drive.course_latched = 1U;
    }

    /* Front-only messages are safety events, never normal control frames. */
    if (!in->side_valid)
    {
        uint8_t spin_committed = (uint8_t)(drive.state == DS_SPIN
            && (in->now - drive.state_since_ms) >= SPIN_COMMIT_MS);
        if (in->f_valid && in->f < FRONT_DANGER_CM
            && (drive.state == DS_CRUISE
                || drive.state == DS_CORNER
                || drive.state == DS_SIDE_AVOID
                || spin_committed))
        {
            brake_enter(in);
        }
        dbg.state = (uint8_t)drive.state;
        return;
    }

    if (drive.state == DS_CRUISE && in->imu_live && !drive.last_full_imu_live)
    {
        if (drive.course_reacquire_pending)
        {
            drive.heading_ref = course_grid_snap(in->heading);
            drive.course_heading = drive.heading_ref;
        }
        else
        {
            drive.heading_ref = drive.course_heading;
        }
        drive.course_reacquire_pending = 0U;
        DriveControl_Reset();
        DriveControl_PrimeHeading(in->heading);
    }

    switch (drive.state)
    {
    case DS_CRUISE:     cruise_run(in); break;
    case DS_BRAKE:      brake_run(in); break;
    case DS_SPIN:       spin_run(in); break;
    case DS_REVERSE:    reverse_run(in); break;
    case DS_HOLD:       hold_run(in); break;
    case DS_SIDE_AVOID: side_avoid_run(in); break;
    case DS_CORNER:     corner_run(in); break;
    default:
        motion_stop(in->now);
        drive.state = DS_CRUISE;
        break;
    }

    dbg.state = (uint8_t)drive.state;
    dbg.turn_dir = drive.turn_right;
    dbg.rev_cnt = drive.reverse_count;
    drive.last_full_imu_live = in->imu_live;
}
