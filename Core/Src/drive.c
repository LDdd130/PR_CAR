/* ============================================================================
 * PR_CAR drive core v2 (§5.17 rewrite).
 *
 * Three logical layers behind the unchanged seven-state telemetry enum:
 *
 *   CRUISE  — corridor centering (perpendicular ToF pair) + heading hold on
 *             the 90 deg course axis + a speed governor that keeps the car
 *             inside its sensing/braking budget at all times.
 *   TURN    — DS_CORNER rolling arc as the primary corner, DS_SPIN pivot as
 *             the fallback and the decide-at-20cm path (user spec §5.9).
 *   RECOVER — DS_BRAKE (stop, verify, creep, decide), DS_REVERSE (backing:
 *             three-point-turn chunk from a blocked pivot, straight
 *             otherwise), DS_SIDE_AVOID (side rescue), DS_HOLD (sensors dead).
 *
 * Carried-over invariants (HANDOVER §7/§9):
 *   - Turn progress is wrap180-increment accumulation, never
 *     wrap180(now - entry). turn_accum survives backing chunks; turn_leg
 *     resets at every interruption and alone may unlock a course reversal.
 *   - Front-wall decisions need consecutive FRESH samples; a spike or a
 *     dropout latch never brakes, opens a corner or picks a direction.
 *   - Cruise heading references are 90 deg course axes only.
 *   - Every TURN->CRUISE exit passes the course-reversal gate.
 *   - Pivot/rescue duties never drop below the skid breakaway floor.
 *   - IMU-dependent gates keep IMU-independent backstops (motor inrush
 *     brownout kills imu_live).
 * ==========================================================================*/

#include "drive.h"
#include "drive_math.h"
#include "motor.h"
#include "debug.h"

#include <math.h>

volatile DebugMonitor_t dbg;

/* Steer-mode telemetry values (webapp contract, formerly drive_control.h). */
#define STEER_MODE_BOTH      0U
#define STEER_MODE_HEADING   3U
#define STEER_MODE_OPEN      4U
#define STEER_MODE_SINGLE    5U

typedef struct
{
    float integral;
    float previous_progress;
    uint32_t previous_ms;
    uint8_t previous_valid;
} TurnPid;

typedef struct
{
    DriveState state;
    uint32_t state_since_ms;

    uint8_t launching;
    uint32_t launch_since_ms;

    /* course frame (boot-relative; BNO055 absolute heading is never trusted) */
    uint8_t course_latched;
    uint8_t reacquire_pending;
    uint8_t last_imu_live;
    float course_zero;
    float course_heading;
    float heading_ref;

    /* turn */
    uint8_t turn_right;
    uint8_t entry_heading_valid;
    uint8_t flip_used;
    float turn_last_heading;
    float turn_accum;       /* episode total, survives backing chunks */
    float turn_leg;         /* current uninterrupted sweep only (§5.16) */
    float stuck_heading;
    uint32_t stuck_since_ms;
    uint32_t spin_since_ms;
    TurnPid pid;

    /* recover */
    uint8_t back_from_turn;
    uint8_t retry_count;

    /* fresh-confirm counters */
    uint8_t front_wall_n;
    uint8_t front_stop_n;
    uint8_t decide_n;
    uint8_t wall_gone_n;
    uint8_t clear_n;
    uint8_t block_n;
    uint8_t side_clear_n;
    int8_t corner_dir;
    uint8_t corner_n;
    uint8_t corner_dropout;

    /* centering filters */
    uint8_t lp_valid;
    uint8_t l_prev_valid;
    uint8_t r_prev_valid;
    uint8_t err_valid;
    uint8_t hdg_lp_valid;
    uint8_t imu_prev_live;
    float l_lp, r_lp;
    float err_prev;
    float derr_lp;
    float yaw_lp;
    float hdg_err_lp;
    float previous_heading;
    float lateral_cmd;
    float previous_steer;
    uint32_t center_prev_ms;

    /* motion ramp */
    int16_t motion_left;
    int16_t motion_right;
    uint32_t motion_ms;
    uint8_t motion_valid;
} DriveContext;

static DriveContext d = {
    .state = DS_CRUISE,
    .launching = 1U,
    .corner_dir = -1,
};

/* ---------------------------------------------------------------- helpers */

static uint8_t inc_u8(uint8_t v) { return (v < 255U) ? (uint8_t)(v + 1U) : v; }

/* A short echo dropout must not erase the last near-wall observation. */
static uint8_t front_recent_below(const DriveInputs *in, uint16_t cm)
{
    if (in->f_valid) return (uint8_t)(in->f < cm);
    return (uint8_t)(in->front_miss > 0U
        && in->front_miss < FRONT_FAIL_LIMIT
        && in->f < cm);
}

/* No echo within the 6 ms budget means nothing within ~1 m: a persistent
 * timeout counts as open space, an isolated one does not. */
static uint8_t front_open_at(const DriveInputs *in, uint16_t cm)
{
    if (in->f_valid) return (uint8_t)(in->f >= cm);
    return (uint8_t)(in->front_miss >= FRONT_FAIL_LIMIT);
}

static int16_t clamp_pct(int16_t v)
{
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
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

static float approach_f(float current, float target, float max_step)
{
    return drive_clampf(target, current - max_step, current + max_step);
}

/* Motion states share one time-based wheel ramp; brake/stop bypass it. */
static void motion_command(int16_t left, int16_t right, uint32_t now)
{
    left = clamp_pct(left);
    right = clamp_pct(right);

    if (!d.motion_valid)
    {
        d.motion_left = 0;
        d.motion_right = 0;
        d.motion_ms = now - DRIVE_NOMINAL_UPDATE_MS;
        d.motion_valid = 1U;
    }

    uint32_t elapsed = now - d.motion_ms;
    if (elapsed > 100U) elapsed = 100U;
    int16_t step = (int16_t)((DRIVE_WHEEL_SLEW_PCT_PER_S * (float)elapsed / 1000.0f) + 0.5f);
    if (elapsed > 0U && step < 1) step = 1;
    d.motion_left = approach_i16(d.motion_left, left, step);
    d.motion_right = approach_i16(d.motion_right, right, step);

    d.motion_ms = now;
    Motor_SetWheels(d.motion_left, d.motion_right);
    dbg.duty_l = d.motion_left;
    dbg.duty_r = d.motion_right;
}

static void motion_command_immediate(int16_t left, int16_t right, uint32_t now)
{
    d.motion_left = clamp_pct(left);
    d.motion_right = clamp_pct(right);
    d.motion_ms = now;
    d.motion_valid = 1U;
    Motor_SetWheels(d.motion_left, d.motion_right);
    dbg.duty_l = d.motion_left;
    dbg.duty_r = d.motion_right;
}

static void motion_stop(uint32_t now)
{
    Car_Stop();
    d.motion_left = 0;
    d.motion_right = 0;
    d.motion_ms = now;
    d.motion_valid = 1U;
    dbg.duty_l = 0;
    dbg.duty_r = 0;
    dbg.steer = 0.0f;
}

static void motion_brake(uint32_t now)
{
    Car_Brake();
    d.motion_left = 0;
    d.motion_right = 0;
    d.motion_ms = now;
    d.motion_valid = 1U;
    dbg.duty_l = 0;
    dbg.duty_r = 0;
    dbg.steer = 0.0f;
}

static void command_arc(uint8_t outer, uint8_t inner, uint32_t now)
{
    int16_t delta = (int16_t)((outer - inner) / 2U);
    if (d.turn_right)
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
    int16_t left = d.turn_right ? (int16_t)outer : -(int16_t)inner;
    int16_t right = d.turn_right ? -(int16_t)inner : (int16_t)outer;
    if (immediate) motion_command_immediate(left, right, now);
    else motion_command(left, right, now);

    dbg.steer = d.turn_right
        ? -(float)(((int16_t)outer + (int16_t)inner) / 2)
        : (float)(((int16_t)outer + (int16_t)inner) / 2);
}

/* Reverse phase of a three-point turn: retreat while yaw keeps the committed
 * sign, so the backing chunk contributes turn progress instead of only
 * buying centimetres the re-approach immediately spends (§5.14). */
static void command_back_turn(uint32_t now, uint8_t immediate)
{
    int16_t inner = -(int16_t)REV_TURN_INNER;
    int16_t outer = -(int16_t)REV_TURN_OUTER;
    int16_t left = d.turn_right ? inner : outer;
    int16_t right = d.turn_right ? outer : inner;
    if (immediate) motion_command_immediate(left, right, now);
    else motion_command(left, right, now);

    dbg.steer = d.turn_right
        ? -(float)((REV_TURN_OUTER + REV_TURN_INNER) / 2)
        : (float)((REV_TURN_OUTER + REV_TURN_INNER) / 2);
}

/* ------------------------------------------------------------ course frame */

/* Cruise references may only be 90 deg course axes: any finer grid converts
 * a corner overshoot into a committed diagonal reference (§5.10). */
static float course_axis_snap(float heading)
{
    if (!d.course_latched) return heading;
    float index = roundf(drive_wrap180(heading - d.course_zero) / 90.0f);
    return drive_wrap360(d.course_zero + (index * 90.0f));
}

static uint8_t axis_aligned(float heading)
{
    return (uint8_t)(fabsf(drive_wrap180(heading - course_axis_snap(heading)))
        <= TURN_AXIS_ALIGN_DEG);
}

/* Both flanks open (or dead) means there is no corridor whose axis a turn
 * could align to — the plaza facets sit at 45 deg, and holding out for a
 * 90 deg pose there is what churned mappv13/14 (§5.19). */
static uint8_t sides_open_wide(const DriveInputs *in)
{
    uint8_t l_open = (uint8_t)(!in->left_valid || in->l >= TURN_WIDE_EXIT_SIDE_CM);
    uint8_t r_open = (uint8_t)(!in->right_valid || in->r >= TURN_WIDE_EXIT_SIDE_CM);
    return (uint8_t)(l_open && r_open);
}

/* ---------------------------------------------------------- turn progress */

static void turn_progress_reset(float heading)
{
    d.turn_accum = 0.0f;
    d.turn_leg = 0.0f;
    d.turn_last_heading = heading;
}

static float turn_progress_update(float heading)
{
    float delta = drive_wrap180(heading - d.turn_last_heading);
    d.turn_last_heading = heading;
    float signed_delta = d.turn_right ? delta : -delta;
    d.turn_accum += signed_delta;
    d.turn_leg += signed_delta;
    return d.turn_accum;
}

/* course_heading is only trustworthy within a half revolution. A single
 * uninterrupted 180 deg sweep (turn_leg) proves the pocket demanded a course
 * reversal; the episode total must never unlock this — totals preserved
 * across backing chunks reach 180 in an ordinary corner fight (§5.16). */
static uint8_t exit_course_ok(const DriveInputs *in)
{
    if (!in->imu_live) return 1U;
    float dev = fabsf(drive_wrap180(in->heading - d.course_heading));
    dbg.course_dev = dev;
    return (uint8_t)(dev < COURSE_REV_DEG || d.turn_leg >= TURN_LEG_REVERSAL_DEG);
}

/* --------------------------------------------------------------- turn PID */

static void turn_pid_reset(void) { d.pid = (TurnPid){0}; }

static void turn_pid_run(const DriveInputs *in, float progress)
{
    float dt = 0.02f;
    if (d.pid.previous_ms != 0U && in->now > d.pid.previous_ms)
        dt = drive_clampf((float)(in->now - d.pid.previous_ms) / 1000.0f, 0.005f, 0.100f);

    float error = TURN_TARGET_DEG - progress;
    if (error < 0.0f) error = 0.0f;
    if (error < 45.0f)
    {
        d.pid.integral += error * dt;
        d.pid.integral = drive_clampf(d.pid.integral, -TURN_PID_I_MAX, TURN_PID_I_MAX);
    }
    else
    {
        d.pid.integral = 0.0f;
    }

    float rate = d.pid.previous_valid
        ? (progress - d.pid.previous_progress) / dt
        : 0.0f;
    float command = (TURN_PID_KP * error)
                  + (TURN_PID_KI * d.pid.integral)
                  - (TURN_PID_KD * rate);
    float minimum = (error <= TURN_PID_FINE_DEG)
        ? (float)TURN_PID_FINE_MIN_PCT
        : (float)TURN_PID_MIN_PCT;
    command = drive_clampf(command, minimum, (float)TURN_SPEED);

    uint8_t outer = (uint8_t)(command + 0.5f);
    uint8_t inner = (uint8_t)((command * TURN_PID_INNER_RATIO) + 0.5f);
    if (inner < MOTOR_MIN_PCT) inner = MOTOR_MIN_PCT;
    if (inner > TURN_INNER) inner = TURN_INNER;
    command_pivot(outer, inner, in->now, 0U);

    d.pid.previous_progress = progress;
    d.pid.previous_valid = 1U;
    d.pid.previous_ms = in->now;
}

/* ---------------------------------------------------------------- centering */

static void centering_reset(const DriveInputs *in)
{
    d.lp_valid = 0U;
    d.l_prev_valid = 0U;
    d.r_prev_valid = 0U;
    d.err_valid = 0U;
    d.hdg_lp_valid = 0U;
    d.derr_lp = 0.0f;
    d.yaw_lp = 0.0f;
    d.lateral_cmd = 0.0f;
    d.previous_steer = 0.0f;
    d.center_prev_ms = 0U;
    if (in->imu_live)
    {
        d.previous_heading = in->heading;
        d.imu_prev_live = 1U;
    }
    else
    {
        d.imu_prev_live = 0U;
    }
}

static float sanitize_distance(uint16_t cm, uint8_t valid)
{
    if (!valid || cm == 0U || cm > CENTER_SENSOR_MAX_CM)
        return (float)CENTER_SENSOR_MAX_CM;
    return (float)cm;
}

/* Lateral offset shaping: dead inside sensor trust (3 cm), linear in the
 * weave regime, extra slope past the knee so a large offset recenters before
 * the car rides into the repel zone (§5.15). */
static float shaped_center_error(float error_cm)
{
    float mag = fabsf(error_cm);
    if (mag <= CENTER_DEADZONE_CM) return 0.0f;
    float shaped = mag - CENTER_DEADZONE_CM;
    if (mag > CENTER_LATERAL_KNEE_CM)
        shaped += (mag - CENTER_LATERAL_KNEE_CM) * CENTER_LATERAL_KNEE_GAIN;
    return (error_cm >= 0.0f) ? shaped : -shaped;
}

/* Heading P term with extra authority past the knee: the base gain is
 * weave-tuned and alone crawls back from a 25-30 deg off-axis pose (§5.15). */
static float heading_steer_cmd(float error_deg, float kp)
{
    float steer = kp * error_deg;
    float over = fabsf(error_deg) - CENTER_HDG_KNEE_DEG;
    if (over > 0.0f)
        steer += ((error_deg >= 0.0f) ? over : -over) * CENTER_HDG_KP2_PCT_PER_DEG;
    return steer;
}

/* Keep normal centering continuous across the motor floor; near a wall keep
 * both wheels above breakaway and cap the differential to a controlled arc. */
static void mix_substall(float *left, float *right, uint8_t wall_risk)
{
    float slow = fminf(*left, *right);
    if (slow <= 0.0f || slow >= (float)MOTOR_MIN_PCT) return;

    float differential = fabsf(*right - *left);
    if (!wall_risk)
    {
        float lift = (float)MOTOR_MIN_PCT - slow;
        *left += lift;
        *right += lift;
        return;
    }

    float inner = CENTER_WALL_INNER_FLOOR_PCT;
    float turn = fminf(differential, CENTER_WALL_DIFF_MAX_PCT);
    float outer = fmaxf(fmaxf(*left, *right), inner + turn);
    if (*left < *right) { *left = inner; *right = outer; }
    else                { *right = inner; *left = outer; }
}

/* Cruise control frame: centering + heading hold + speed governor. */
static void centering_run(const DriveInputs *in)
{
    float dt = 0.02f;
    if (d.center_prev_ms != 0U && in->now > d.center_prev_ms)
        dt = drive_clampf((float)(in->now - d.center_prev_ms) / 1000.0f, 0.005f, 0.100f);

    /* -- side pair conditioning (perpendicular ToF: l/r are true lateral) */
    float l_raw = sanitize_distance(in->l, in->left_valid);
    float r_raw = sanitize_distance(in->r, in->right_valid);
    if (!d.lp_valid)
    {
        d.l_lp = l_raw;
        d.r_lp = r_raw;
        d.lp_valid = 1U;
    }
    else
    {
        /* invalid -> valid transitions reseed raw (§7-4) */
        if (!in->left_valid) d.l_lp = (float)CENTER_SENSOR_MAX_CM;
        else if (!d.l_prev_valid) d.l_lp = l_raw;
        else d.l_lp += CENTER_LPF_ALPHA * (l_raw - d.l_lp);

        if (!in->right_valid) d.r_lp = (float)CENTER_SENSOR_MAX_CM;
        else if (!d.r_prev_valid) d.r_lp = r_raw;
        else d.r_lp += CENTER_LPF_ALPHA * (r_raw - d.r_lp);
    }
    d.l_prev_valid = in->left_valid;
    d.r_prev_valid = in->right_valid;

    uint8_t left_seen = (uint8_t)(in->left_valid && d.l_lp < (float)CENTER_SENSOR_MAX_CM);
    uint8_t right_seen = (uint8_t)(in->right_valid && d.r_lp < (float)CENTER_SENSOR_MAX_CM);
    uint8_t pair_valid = (uint8_t)(left_seen && right_seen
        && (d.l_lp + d.r_lp) <= SIDE_PAIR_MAX_CM);

    uint8_t left_track = left_seen;
    uint8_t right_track = right_seen;
    if (!pair_valid && left_seen && right_seen)
    {
        if (d.l_lp <= d.r_lp) right_track = 0U;
        else                  left_track = 0U;
    }
    if (left_track && d.l_lp > CENTER_SINGLE_MAX_CM) left_track = 0U;
    if (right_track && d.r_lp > CENTER_SINGLE_MAX_CM) right_track = 0U;

    float err_cm = d.l_lp - d.r_lp;
    float raw_derr = (pair_valid && d.err_valid) ? (err_cm - d.err_prev) / dt : 0.0f;
    if (pair_valid && d.err_valid)
        d.derr_lp += CENTER_DERR_LPF_ALPHA * (raw_derr - d.derr_lp);
    else
        d.derr_lp = 0.0f;
    float derr = drive_clampf(d.derr_lp, -CENTER_DERR_MAX_CMS, CENTER_DERR_MAX_CMS);
    d.err_prev = err_cm;
    d.err_valid = pair_valid;

    /* -- yaw rate (IMU differences only) */
    float raw_yaw = (in->imu_live && d.imu_prev_live)
        ? drive_wrap180(in->heading - d.previous_heading) / dt
        : 0.0f;
    if (in->imu_live && d.imu_prev_live)
        d.yaw_lp += CENTER_YAW_LPF_ALPHA * (raw_yaw - d.yaw_lp);
    else
        d.yaw_lp = 0.0f;
    float yaw = drive_clampf(d.yaw_lp, -CENTER_YAW_RATE_MAX_DPS, CENTER_YAW_RATE_MAX_DPS);
    if (in->imu_live) d.previous_heading = in->heading;
    d.imu_prev_live = in->imu_live;

    /* -- heading error, LPF'd (noise dies here, not in a wide deadband §5.8) */
    float hdg_err = 0.0f;
    if (in->imu_live)
    {
        float raw_err = drive_wrap180(in->heading - d.heading_ref);
        if (!d.hdg_lp_valid)
        {
            d.hdg_err_lp = raw_err;
            d.hdg_lp_valid = 1U;
        }
        else
        {
            d.hdg_err_lp += CENTER_HDG_ERR_LPF_ALPHA
                * drive_wrap180(raw_err - d.hdg_err_lp);
        }
        hdg_err = d.hdg_err_lp;
    }
    else
    {
        d.hdg_lp_valid = 0U;
    }

    /* -- steering */
    float steer = 0.0f;
    uint8_t mode;
    if (pair_valid)
    {
        float lateral_target = (CENTER_LATERAL_KP_DEG_PER_CM * shaped_center_error(err_cm))
                             + (CENTER_LATERAL_KD_DEG_PER_CMS * derr);
        lateral_target = drive_clampf(lateral_target,
            -CENTER_LATERAL_CMD_MAX_DEG, CENTER_LATERAL_CMD_MAX_DEG);
        d.lateral_cmd = approach_f(d.lateral_cmd, lateral_target,
            CENTER_LATERAL_CMD_SLEW_DPS * dt);

        mode = STEER_MODE_BOTH;
        if (in->imu_live)
            steer = heading_steer_cmd(
                drive_deadbandf(hdg_err + d.lateral_cmd, CENTER_HDG_DEADBAND_DEG),
                CENTER_HDG_KP_PCT_PER_DEG);
        else
            steer = CENTER_HDG_KP_PCT_PER_DEG * d.lateral_cmd;
    }
    else if (left_track || right_track)
    {
        d.lateral_cmd = 0.0f;
        float wall_steer = left_track
            ? CENTER_SINGLE_KP * (d.l_lp - CENTER_SINGLE_TARGET_CM)
            : CENTER_SINGLE_KP * (CENTER_SINGLE_TARGET_CM - d.r_lp);
        /* only ever steer AWAY from the tracked wall */
        if (left_track && wall_steer > 0.0f) wall_steer = 0.0f;
        if (!left_track && wall_steer < 0.0f) wall_steer = 0.0f;

        /* curve zone: one wall, the other flank open — the wall is the only
         * trustworthy reference, so a large axis error must not drag the car
         * off the wall-follow line (§5.19) */
        uint8_t other_wide = left_track
            ? (uint8_t)(!right_seen || d.r_lp >= CENTER_WIDE_SIDE_CM)
            : (uint8_t)(!left_seen || d.l_lp >= CENTER_WIDE_SIDE_CM);

        mode = STEER_MODE_SINGLE;
        steer = wall_steer;
        if (in->imu_live
            && !(other_wide && fabsf(hdg_err) > CENTER_OPEN_FREE_DEG))
            steer += CENTER_SINGLE_HDG_BLEND
                   * heading_steer_cmd(
                         drive_deadbandf(hdg_err, CENTER_HDG_DEADBAND_DEG),
                         CENTER_OPEN_HDG_KP_PCT_PER_DEG);
    }
    else if (in->imu_live)
    {
        d.lateral_cmd = 0.0f;
        mode = STEER_MODE_HEADING;
        /* no wall inside 30 cm on either flank here by construction: past
         * CENTER_OPEN_FREE_DEG the axis error is the curve's fault — hold
         * the line with yaw damping only instead of pivot-fighting (§5.19) */
        if (fabsf(hdg_err) <= CENTER_OPEN_FREE_DEG)
            steer = heading_steer_cmd(
                drive_deadbandf(hdg_err, CENTER_HDG_DEADBAND_DEG),
                CENTER_OPEN_HDG_KP_PCT_PER_DEG);
    }
    else
    {
        d.lateral_cmd = 0.0f;
        mode = STEER_MODE_OPEN;
    }

    /* progressive wall repel: the only steering term needing no reference */
    {
        float span = SIDE_SOFT_CM - SIDE_HARD_CM;
        if (left_track && d.l_lp < SIDE_SOFT_CM)
        {
            float depth = SIDE_SOFT_CM - d.l_lp;
            steer -= CENTER_SIDE_REPEL_KP * depth * depth / span;
        }
        if (right_track && d.r_lp < SIDE_SOFT_CM)
        {
            float depth = SIDE_SOFT_CM - d.r_lp;
            steer += CENTER_SIDE_REPEL_KP * depth * depth / span;
        }
    }

    if (in->imu_live)
    {
        float yaw_damp = drive_deadbandf(yaw, CENTER_YAW_RATE_DEADBAND_DPS);
        steer += drive_clampf(CENTER_YAW_KD_PCT_PER_DPS * yaw_damp,
            -CENTER_YAW_DAMP_MAX_PCT, CENTER_YAW_DAMP_MAX_PCT);
    }

    /* off-axis recovery unlocks extra clamp; the weave regime never sees it */
    float steer_limit = (in->imu_live && fabsf(hdg_err) >= CENTER_STEER_ALIGN_ERR_DEG)
        ? CENTER_STEER_MAX_ALIGN_PCT
        : CENTER_STEER_MAX_PCT;
    steer = drive_clampf(steer, -steer_limit, steer_limit);
    steer = approach_f(d.previous_steer, steer, CENTER_STEER_SLEW_PCT_PER_S * dt);
    d.previous_steer = steer;

    /* -- speed governor: lowest cap wins; front cap is the braking budget */
    float speed = SPEED_TOP_PCT;
    if (in->f_valid)
    {
        float cap;
        if ((float)in->f <= SPEED_FRONT_SLOW_CM)
            cap = SPEED_FRONT_MIN_PCT;
        else if ((float)in->f >= SPEED_FRONT_FAST_CM)
            cap = SPEED_TOP_PCT;
        else
            cap = SPEED_FRONT_MIN_PCT
                + ((SPEED_TOP_PCT - SPEED_FRONT_MIN_PCT)
                   * ((float)in->f - SPEED_FRONT_SLOW_CM)
                   / (SPEED_FRONT_FAST_CM - SPEED_FRONT_SLOW_CM));
        if (speed > cap) speed = cap;
    }
    if (left_track || right_track)
    {
        float side_min = left_track ? d.l_lp : d.r_lp;
        if (right_track && d.r_lp < side_min) side_min = d.r_lp;
        float cap;
        if (side_min <= SIDE_HARD_CM)
            cap = SPEED_SIDE_MIN_PCT;
        else if (side_min >= SIDE_SOFT_CM)
            cap = SPEED_TOP_PCT;
        else
            cap = SPEED_SIDE_MIN_PCT
                + ((SPEED_TOP_PCT - SPEED_SIDE_MIN_PCT)
                   * (side_min - SIDE_HARD_CM) / (SIDE_SOFT_CM - SIDE_HARD_CM));
        if (speed > cap) speed = cap;
    }
    if (in->imu_live)
    {
        float mag = fabsf(hdg_err);
        float cap;
        if (mag >= SPEED_HDG_SLOW_DEG) cap = SPEED_HDG_MIN_PCT;
        else if (mag <= SPEED_HDG_FAST_DEG) cap = SPEED_TOP_PCT;
        else cap = SPEED_TOP_PCT
                 - ((SPEED_TOP_PCT - SPEED_HDG_MIN_PCT)
                    * (mag - SPEED_HDG_FAST_DEG)
                    / (SPEED_HDG_SLOW_DEG - SPEED_HDG_FAST_DEG));
        if (speed > cap) speed = cap;

        float ymag = fabsf(yaw);
        if (ymag >= SPEED_YAW_SLOW_DPS) cap = SPEED_YAW_MIN_PCT;
        else if (ymag <= SPEED_YAW_FAST_DPS) cap = SPEED_TOP_PCT;
        else cap = SPEED_TOP_PCT
                 - ((SPEED_TOP_PCT - SPEED_YAW_MIN_PCT)
                    * (ymag - SPEED_YAW_FAST_DPS)
                    / (SPEED_YAW_SLOW_DPS - SPEED_YAW_FAST_DPS));
        if (speed > cap) speed = cap;
    }
    if ((in->now - d.state_since_ms) < SPEED_SETTLE_MS && speed > SPEED_SETTLE_PCT)
        speed = SPEED_SETTLE_PCT;
    speed = drive_clampf(speed, SPEED_MIN_PCT, SPEED_TOP_PCT);

    float left = drive_clampf(speed - steer + (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
    float right = drive_clampf(speed + steer - (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
    uint8_t wall_risk = (uint8_t)((left_track && d.l_lp < SIDE_HARD_CM)
        || (right_track && d.r_lp < SIDE_HARD_CM));
    mix_substall(&left, &right, wall_risk);
    motion_command((int16_t)(left + 0.5f), (int16_t)(right + 0.5f), in->now);

    d.center_prev_ms = in->now;
    dbg.steer = steer;
    dbg.steer_mode = mode;
    dbg.yaw_rate = yaw;
    dbg.hdg_err = hdg_err;
}

/* ------------------------------------------------------------ state entry */

static void state_enter(DriveState state, uint32_t now)
{
    d.state = state;
    d.state_since_ms = now;
}

static void reset_confirm_counters(void)
{
    d.front_wall_n = 0U;
    d.front_stop_n = 0U;
    d.decide_n = 0U;
    d.wall_gone_n = 0U;
    d.clear_n = 0U;
    d.block_n = 0U;
    d.side_clear_n = 0U;
    d.corner_dir = -1;
    d.corner_n = 0U;
    d.corner_dropout = 0U;
}

static void cruise_enter(const DriveInputs *in)
{
    DriveState previous = d.state;
    uint8_t from_turn = (uint8_t)((previous == DS_SPIN || previous == DS_CORNER)
        && d.entry_heading_valid && in->imu_live);
    uint8_t reacquire = (uint8_t)(in->imu_live && d.reacquire_pending);

    centering_reset(in);
    if (in->imu_live)
    {
        if (from_turn || reacquire)
        {
            d.heading_ref = course_axis_snap(in->heading);
            d.course_heading = d.heading_ref;
        }
        else if (d.course_latched)
        {
            d.heading_ref = d.course_heading;
        }
        else
        {
            d.heading_ref = in->heading;
        }
        d.reacquire_pending = 0U;
    }
    else if (previous == DS_SPIN || previous == DS_CORNER)
    {
        /* brownout during the turn: re-snap the axis when the IMU returns */
        d.reacquire_pending = 1U;
    }

    d.retry_count = 0U;
    reset_confirm_counters();
    state_enter(DS_CRUISE, in->now);
    centering_run(in);
}

static void brake_enter(const DriveInputs *in)
{
    reset_confirm_counters();
    motion_brake(in->now);
    state_enter(DS_BRAKE, in->now);
}

/* keep_progress: arc->pivot escalation and post-backing resume keep the
 * episode accumulation; a fresh decide starts from zero. */
static void spin_begin(const DriveInputs *in, uint8_t keep_progress)
{
    reset_confirm_counters();
    if (!keep_progress)
    {
        turn_progress_reset(in->heading);
        d.flip_used = 0U;
    }
    d.entry_heading_valid = in->imu_live;
    d.stuck_heading = in->heading;
    d.stuck_since_ms = in->now;
    d.spin_since_ms = in->now;
    turn_pid_reset();
    state_enter(DS_SPIN, in->now);
    command_pivot(TURN_SPEED, TURN_INNER, in->now, 1U);
}

static void corner_enter(const DriveInputs *in, uint8_t turn_right)
{
    reset_confirm_counters();
    d.turn_right = turn_right;
    d.flip_used = 0U;
    d.retry_count = 0U;    /* a confirmed corner is forward progress */
    turn_progress_reset(in->heading);
    d.entry_heading_valid = in->imu_live;
    turn_pid_reset();
    state_enter(DS_CORNER, in->now);
    command_arc(ARC_OUTER, ARC_INNER, in->now);
}

static void back_enter(const DriveInputs *in, uint8_t from_turn)
{
    reset_confirm_counters();
    d.retry_count = inc_u8(d.retry_count);
    /* budget spent: degrade to straight backing + a fresh decide instead of
     * chaining turn-preserving chunks without bound (§5.15/§5.16) */
    if (d.retry_count > RECOVER_RETRY_MAX) from_turn = 0U;
    d.back_from_turn = from_turn;
    state_enter(DS_REVERSE, in->now);
    if (from_turn) command_back_turn(in->now, 1U);
    else motion_command_immediate(-DRIVE_SPEED, -DRIVE_SPEED, in->now);
    dbg.rev_cnt = d.retry_count;
}

static void side_avoid_enter(const DriveInputs *in)
{
    reset_confirm_counters();
    if (in->left_valid && (!in->right_valid || in->l < in->r))
        d.turn_right = 1U;
    else if (in->right_valid && (!in->left_valid || in->r < in->l))
        d.turn_right = 0U;
    turn_progress_reset(in->heading);
    d.entry_heading_valid = in->imu_live;
    state_enter(DS_SIDE_AVOID, in->now);
    command_pivot(RESCUE_OUTER, RESCUE_INNER, in->now, 1U);
}

static void hold_enter(const DriveInputs *in)
{
    reset_confirm_counters();
    motion_stop(in->now);
    state_enter(DS_HOLD, in->now);
}

/* Direction of a decided pivot. Sides first (with hysteresis); a confirmed
 * wall with tied sides falls back to the course-final rule (the last
 * right-angle before the bump is geometrically symmetric); otherwise keep
 * the previous direction rather than inventing one from noise. */
static void decide_turn_direction(const DriveInputs *in)
{
    if (in->left_valid && in->right_valid)
    {
        if (in->l > (uint16_t)(in->r + SIDE_HYST_CM)) { d.turn_right = 0U; return; }
        if (in->r > (uint16_t)(in->l + SIDE_HYST_CM)) { d.turn_right = 1U; return; }
        if (d.course_latched && in->imu_live)
        {
            float rel = drive_wrap180(in->heading - d.course_zero);
            if (fabsf(drive_wrap180(rel - COURSE_FINAL_ENTRY_DEG))
                <= COURSE_FINAL_ENTRY_TOL_DEG)
            {
                d.turn_right = 1U;
                return;
            }
        }
        return;                      /* tie: keep previous direction */
    }
    if (in->left_valid) { d.turn_right = 1U; return; }
    if (in->right_valid) { d.turn_right = 0U; return; }
}

/* ------------------------------------------------------------- state runs */

static uint8_t side_emergency(const DriveInputs *in)
{
    return (uint8_t)((in->left_valid && in->l < SIDE_AVOID_CM)
        || (in->right_valid && in->r < SIDE_AVOID_CM));
}

static void cruise_run(const DriveInputs *in)
{
    if (in->front_miss >= FRONT_FAIL_LIMIT && !in->left_valid && !in->right_valid)
    {
        hold_enter(in);
        return;
    }

    /* IMU-independent launch window (inrush brownout kills imu_live) */
    if (d.launching)
    {
        if (d.launch_since_ms == 0U) d.launch_since_ms = in->now;
        uint8_t in_window = (uint8_t)((in->now - d.launch_since_ms) < LAUNCH_MS);
        if (in_window && !front_recent_below(in, FRONT_TURN_CM) && !side_emergency(in))
        {
            if (in->imu_live)
            {
                d.heading_ref = course_axis_snap(in->heading);
                centering_reset(in);
            }
            motion_command((int16_t)SPEED_SETTLE_PCT, (int16_t)SPEED_SETTLE_PCT, in->now);
            dbg.launch = 1U;
            dbg.steer = 0.0f;
            return;
        }
        d.launching = 0U;
        dbg.launch = 0U;
    }

    if (front_recent_below(in, FRONT_DANGER_CM))
    {
        brake_enter(in);
        return;
    }

    /* -- corner detection: confirmed approaching wall AND sustained side
     * asymmetry. A weaving straight fakes neither for long (§5.10/§5.11). */
    uint8_t front_wall_now = (uint8_t)(in->f_valid && in->f < FRONT_TURN_CM);
    d.front_wall_n = front_wall_now ? inc_u8(d.front_wall_n) : 0U;
    uint8_t front_wall = (uint8_t)(d.front_wall_n >= FRONT_WALL_CONFIRM_N);

    int8_t direction = -1;
    if (in->left_valid && in->right_valid)
    {
        uint8_t left_open = (uint8_t)(in->l >= SIDE_OPEN_CM
            && in->l >= (uint16_t)(in->r + SIDE_ASYM_CM)
            && in->r >= SIDE_NEAR_SAFE_CM);
        uint8_t right_open = (uint8_t)(in->r >= SIDE_OPEN_CM
            && in->r >= (uint16_t)(in->l + SIDE_ASYM_CM)
            && in->l >= SIDE_NEAR_SAFE_CM);
        if (left_open) direction = 0;
        else if (right_open) direction = 1;
    }

    if (front_wall && direction >= 0)
    {
        if (d.corner_dir != direction)
        {
            d.corner_dir = direction;
            d.corner_n = 1U;
        }
        else
        {
            d.corner_n = inc_u8(d.corner_n);
        }
        d.corner_dropout = 0U;
        d.front_stop_n = 0U;
        if (d.corner_n >= CORNER_CONFIRM_N)
        {
            corner_enter(in, (uint8_t)direction);
            return;
        }
        /* roll the arc while the confirmation frame arrives */
        d.turn_right = (uint8_t)direction;
        command_arc(ARC_OUTER, ARC_INNER, in->now);
        return;
    }
    /* one marginal frame at the open threshold must not restart the
     * confirmation while the wall stays confirmed (§5.15) */
    else if (d.corner_dir >= 0 && d.corner_n > 0U && front_wall
        && d.corner_dropout < CORNER_DROPOUT_TOL_N)
    {
        d.corner_dropout++;
        d.front_stop_n = 0U;
        d.turn_right = (uint8_t)d.corner_dir;
        command_arc(ARC_OUTER, ARC_INNER, in->now);
        return;
    }
    d.corner_dir = -1;
    d.corner_n = 0U;
    d.corner_dropout = 0U;

    /* -- stop line: fresh confirmed only, then the brake-and-decide path */
    if (in->f_valid && in->f < FRONT_STOP_CM)
    {
        d.front_stop_n = inc_u8(d.front_stop_n);
        if (d.front_stop_n >= FRONT_STOP_CONFIRM_N)
        {
            dbg.front_near = d.front_stop_n;
            brake_enter(in);
            return;
        }
    }
    else
    {
        d.front_stop_n = 0U;
    }
    dbg.front_near = d.front_stop_n;

    if (side_emergency(in))
    {
        side_avoid_enter(in);
        return;
    }

    centering_run(in);
}

static void corner_run(const DriveInputs *in)
{
    if (in->imu_live && !d.entry_heading_valid)
    {
        turn_progress_reset(in->heading);
        d.entry_heading_valid = 1U;
    }
    float progress = (in->imu_live && d.entry_heading_valid)
        ? turn_progress_update(in->heading)
        : 0.0f;
    dbg.spin_deg = progress;

    if (side_emergency(in))
    {
        side_avoid_enter(in);
        return;
    }

    /* the arc cannot tighten itself: a close front or a stalled/backward
     * rotation hands the same turn to the pivot, progress preserved */
    uint8_t escalate = front_recent_below(in, CORNER_TIGHTEN_CM);
    if ((in->now - d.state_since_ms) > ARC_MAX_MS) escalate = 1U;
    if (in->imu_live && d.entry_heading_valid
        && (in->now - d.state_since_ms) >= SPIN_COMMIT_MS
        && progress < -TURN_WRONG_DEG)
        escalate = 1U;
    if (escalate)
    {
        spin_begin(in, 1U);
        return;
    }

    if (in->imu_live && d.entry_heading_valid && exit_course_ok(in))
    {
        if (progress >= TURN_TARGET_DEG && front_open_at(in, FRONT_TURN_CM))
        {
            cruise_enter(in);
            return;
        }
        if (in->f_valid && in->f >= FRONT_CLEAR_CM)
            d.clear_n = inc_u8(d.clear_n);
        else if (in->f_valid)
            d.clear_n = 0U;
        if (progress >= TURN_MIN_DEG
            && (axis_aligned(in->heading) || sides_open_wide(in))
            && d.clear_n >= CLEAR_CONFIRM_N)
        {
            cruise_enter(in);
            return;
        }
    }

    float remaining = TURN_TARGET_DEG - progress;
    if (remaining < 0.0f) remaining = 0.0f;
    float outer = (float)ARC_OUTER;
    float inner = (float)ARC_INNER;
    if (in->imu_live && d.entry_heading_valid && remaining <= ARC_APPROACH_DEG)
    {
        float ratio = remaining / ARC_APPROACH_DEG;
        outer = (float)ARC_APPROACH_OUTER
              + (((float)ARC_OUTER - (float)ARC_APPROACH_OUTER) * ratio);
        inner = (float)ARC_APPROACH_INNER
              + (((float)ARC_INNER - (float)ARC_APPROACH_INNER) * ratio);
    }
    command_arc((uint8_t)(outer + 0.5f), (uint8_t)(inner + 0.5f), in->now);
}

static void spin_run(const DriveInputs *in)
{
    float progress = 0.0f;
    uint8_t progress_valid = 0U;
    if (in->imu_live)
    {
        if (!d.entry_heading_valid)
        {
            turn_progress_reset(in->heading);
            d.stuck_heading = in->heading;
            d.stuck_since_ms = in->now;
            d.entry_heading_valid = 1U;
            turn_pid_reset();
        }
        progress = turn_progress_update(in->heading);
        progress_valid = 1U;
        dbg.spin_deg = progress;

        /* one flip per episode: a scrub rebound past that must not
         * ping-pong mirrored half-turns (§5.15) */
        if ((in->now - d.state_since_ms) >= SPIN_COMMIT_MS
            && progress < -TURN_WRONG_DEG && !d.flip_used)
        {
            d.turn_right ^= 1U;
            d.flip_used = 1U;
            turn_progress_reset(in->heading);
            d.stuck_heading = in->heading;
            d.stuck_since_ms = in->now;
            d.spin_since_ms = in->now;
            turn_pid_reset();
            state_enter(DS_SPIN, in->now);
            command_pivot(TURN_SPEED, TURN_INNER, in->now, 1U);
            return;
        }

        /* physically wedged: rotation frozen against a confirmed wall means
         * backing is the only move that changes the pose (§5.15) */
        if ((in->now - d.stuck_since_ms) >= ROT_STUCK_MS)
        {
            if (fabsf(drive_wrap180(in->heading - d.stuck_heading)) < ROT_STUCK_DEG
                && front_recent_below(in, FRONT_STOP_CM))
            {
                back_enter(in, 1U);
                return;
            }
            d.stuck_heading = in->heading;
            d.stuck_since_ms = in->now;
        }
    }

    /* blocked nose or front/side pinch, fresh confirmed */
    uint8_t min_side_close = 0U;
    if (in->left_valid && in->right_valid)
    {
        uint16_t near_side = (in->l < in->r) ? in->l : in->r;
        min_side_close = (uint8_t)(near_side < SIDE_AVOID_CM);
    }
    uint8_t blocked_now = (uint8_t)((in->f_valid && in->f < FRONT_DANGER_CM)
        || (in->f_valid && in->f < FRONT_STOP_CM && min_side_close));
    d.block_n = blocked_now ? inc_u8(d.block_n) : 0U;
    if (d.block_n >= SPIN_BLOCK_CONFIRM_N
        && (in->now - d.state_since_ms) >= SPIN_COMMIT_MS)
    {
        back_enter(in, 1U);
        return;
    }

    /* exit: enough rotation, axis-aligned (or no corridor to align to),
     * open nose, course gate */
    if (progress_valid && progress >= TURN_MIN_DEG
        && (axis_aligned(in->heading) || sides_open_wide(in))
        && front_open_at(in, FRONT_TURN_CM)
        && exit_course_ok(in))
    {
        cruise_enter(in);
        return;
    }

    if ((in->now - d.state_since_ms) > SPIN_MAX_MS)
    {
        back_enter(in, 1U);
        return;
    }

    /* IMU-independent backstop */
    if (!d.entry_heading_valid
        && (in->now - d.spin_since_ms) >= SPIN_BLIND_MS
        && front_open_at(in, FRONT_TURN_CM)
        && (in->left_valid || in->right_valid))
    {
        cruise_enter(in);
        return;
    }

    if (progress_valid) turn_pid_run(in, progress);
    else command_pivot(TURN_SPEED, TURN_INNER, in->now, 0U);
}

static void brake_run(const DriveInputs *in)
{
    if ((in->now - d.state_since_ms) < BRAKE_MS) return;

    if (in->f_valid && in->f < FRONT_DANGER_CM)
    {
        motion_stop(in->now);
        if (!in->left_valid && !in->right_valid) return;
        if (d.retry_count <= RECOVER_RETRY_MAX)
        {
            back_enter(in, 0U);
        }
        else
        {
            /* budget exhausted: pivot from where the car stands rather than
             * cycling brake/back forever */
            decide_turn_direction(in);
            spin_begin(in, 0U);
        }
        return;
    }

    /* the braking front reading may have been a spike: if the wall
     * evaporates this was a false brake — resume cruising */
    if (!front_recent_below(in, FRONT_TURN_CM))
    {
        d.wall_gone_n = inc_u8(d.wall_gone_n);
        if (d.wall_gone_n >= CLEAR_CONFIRM_N)
        {
            cruise_enter(in);
            return;
        }
    }
    else
    {
        d.wall_gone_n = 0U;
    }

    /* user spec §5.9: the direction is decided from a wall firmly at the
     * 20 cm line, consecutive fresh samples, read from a stable pose */
    if (in->f_valid && in->f <= FRONT_DECIDE_CM)
    {
        d.decide_n = inc_u8(d.decide_n);
        if (d.decide_n >= FRONT_DECIDE_CONFIRM_N
            && (in->left_valid || in->right_valid))
        {
            decide_turn_direction(in);
            spin_begin(in, 0U);
            return;
        }
        motion_stop(in->now);
        return;
    }
    d.decide_n = 0U;

    /* wall confirmed but above the decide line: creep to it; a wall that
     * never resolves decides from where the car stands */
    if ((in->now - d.state_since_ms) >= (BRAKE_MS + BRAKE_CREEP_MAX_MS))
    {
        motion_stop(in->now);
        if (!in->left_valid && !in->right_valid) return;
        decide_turn_direction(in);
        spin_begin(in, 0U);
        return;
    }
    motion_command(BRAKE_CREEP_PCT, BRAKE_CREEP_PCT, in->now);
}

static void reverse_run(const DriveInputs *in)
{
    if ((in->now - d.state_since_ms) >= BACK_CHUNK_MS)
    {
        if (d.back_from_turn)
        {
            /* resume the interrupted pivot: same direction, no re-decision,
             * rotation gained while backing folds into the total; the leg
             * restarts because the sweep was interrupted (§5.16) */
            d.back_from_turn = 0U;
            if (in->imu_live && d.entry_heading_valid)
            {
                turn_progress_update(in->heading);
                d.turn_leg = 0.0f;
                spin_begin(in, 1U);
            }
            else
            {
                spin_begin(in, 0U);
            }
        }
        else
        {
            brake_enter(in);
        }
        return;
    }
    if (d.back_from_turn) command_back_turn(in->now, 0U);
    else motion_command(-DRIVE_SPEED, -DRIVE_SPEED, in->now);
}

static void side_avoid_run(const DriveInputs *in)
{
    if (in->f_valid && in->f < FRONT_STOP_CM)
    {
        brake_enter(in);
        return;
    }

    uint8_t wall_valid = d.turn_right ? in->left_valid : in->right_valid;
    uint16_t wall_distance = d.turn_right ? in->l : in->r;
    if (wall_valid && wall_distance >= SIDE_AVOID_CLEAR_CM)
    {
        d.side_clear_n = inc_u8(d.side_clear_n);
        if (d.side_clear_n >= SIDE_AVOID_CLEAR_CONFIRM_N)
        {
            cruise_enter(in);
            return;
        }
    }
    else
    {
        d.side_clear_n = 0U;
    }

    if ((in->now - d.state_since_ms) > SIDE_AVOID_MAX_MS)
    {
        brake_enter(in);
        return;
    }

    float progress = 0.0f;
    uint8_t progress_valid = 0U;
    if (in->imu_live)
    {
        if (!d.entry_heading_valid)
        {
            turn_progress_reset(in->heading);
            d.entry_heading_valid = 1U;
        }
        progress = turn_progress_update(in->heading);
        progress_valid = 1U;
    }

    /* past the useful escape angle, roll straight until clearance is proven:
     * continuing the skid pivot leaves a large heading step (§5.13) */
    if (progress_valid && progress >= SIDE_AVOID_PIVOT_MAX_DEG)
    {
        motion_command(SIDE_AVOID_ESCAPE_PCT, SIDE_AVOID_ESCAPE_PCT, in->now);
        dbg.steer = 0.0f;
        return;
    }
    command_pivot(RESCUE_OUTER, RESCUE_INNER, in->now, 0U);
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
        cruise_enter(in);
        return;
    }
    motion_stop(in->now);
}

/* -------------------------------------------------------------- public API */

void Drive_Init(void)
{
    d = (DriveContext){
        .state = DS_CRUISE,
        .launching = 1U,
        .corner_dir = -1,
    };
    dbg.state = DS_CRUISE;
    dbg.turn_dir = 0U;
    dbg.rev_cnt = 0U;
    dbg.launch = 0U;
    dbg.graze = 0U;
    dbg.reverse = 0U;
}

void Drive_Update(const DriveInputs *in)
{
    if (in == NULL)
    {
        motion_stop(0U);
        return;
    }

    if (!d.course_latched && in->imu_live)
    {
        d.course_zero = in->heading;
        d.course_heading = in->heading;
        d.heading_ref = in->heading;
        d.course_latched = 1U;
    }

    /* front-only messages are safety events, never control frames */
    if (!in->side_valid)
    {
        if (in->f_valid && in->f < FRONT_DANGER_CM
            && (d.state == DS_CRUISE
                || d.state == DS_CORNER
                || d.state == DS_SIDE_AVOID))
        {
            brake_enter(in);
        }
        dbg.state = (uint8_t)d.state;
        return;
    }

    /* IMU returned after a brownout while cruising: re-base the axis */
    if (d.state == DS_CRUISE && in->imu_live && !d.last_imu_live)
    {
        if (d.reacquire_pending)
        {
            d.heading_ref = course_axis_snap(in->heading);
            d.course_heading = d.heading_ref;
            d.reacquire_pending = 0U;
        }
        else
        {
            d.heading_ref = d.course_heading;
        }
        centering_reset(in);
    }

    switch (d.state)
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
        d.state = DS_CRUISE;
        break;
    }

    dbg.state = (uint8_t)d.state;
    dbg.turn_dir = d.turn_right;
    dbg.rev_cnt = d.retry_count;
    d.last_imu_live = in->imu_live;
}
