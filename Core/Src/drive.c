/* ============================================================================
 * PR_CAR drive core v2 (§5.17 rewrite).
 *
 * Three logical layers behind the unchanged seven-state telemetry enum:
 *
 *   CRUISE  — corridor centering (perpendicular ToF pair) + heading hold on
 *             the 90 deg course axis + a speed governor that keeps the car
 *             inside its sensing/braking budget at all times.
 *   TURN    — DS_SPIN pivot only, armed exclusively by a close confirmed
 *             front wall via DS_BRAKE decide (user spec §5.9/§5.24: sides
 *             never start a turn; direction is read from L/R at the line).
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
    uint8_t roll_turn;
    float turn_last_heading;
    float turn_accum;       /* episode total, survives backing chunks */
    float turn_leg;         /* current uninterrupted sweep only (§5.16) */
    float stuck_heading;
    uint32_t stuck_since_ms;
    uint32_t spin_since_ms;

    /* recover */
    uint8_t back_from_turn;
    uint8_t retry_count;
    uint32_t stall_since_ms;    /* cruise ram/stall window (§5.26) */

    /* fresh-confirm counters */
    uint8_t front_stable;    /* this frame's f agrees with the previous (§5.23) */
    uint8_t f_prev_valid;
    uint16_t f_prev;
    uint8_t front_stop_n;
    uint8_t decide_n;
    uint8_t wall_gone_n;
    uint8_t clear_n;
    uint8_t side_clear_n;

    /* centering filters */
    uint8_t lp_valid;
    uint8_t l_prev_valid;
    uint8_t r_prev_valid;
    uint8_t err_valid;
    uint8_t hdg_lp_valid;
    uint8_t imu_prev_live;
    float l_lp, r_lp;
    float l_rate_lp, r_rate_lp;
    float err_prev;
    float derr_lp;
    float yaw_lp;
    float hdg_err_lp;
    float previous_heading;
    float lateral_cmd;
    float previous_steer;
    uint8_t center_act;
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

/* §5.39/§5.40 rolling corner: outer drives, inner drags (reverse torque under
 * the breakaway floor = stalled wheel = brake) — the car sweeps the same
 * heading target while still moving forward. Duty 0 would be a COAST (both
 * IN pins low): no drag, no yaw, straight into the wall. */
static void command_roll(uint8_t outer, uint32_t now, uint8_t immediate)
{
    int16_t left = d.turn_right ? (int16_t)outer : (int16_t)TURN_ROLL_INNER;
    int16_t right = d.turn_right ? (int16_t)TURN_ROLL_INNER : (int16_t)outer;
    if (immediate) motion_command_immediate(left, right, now);
    else motion_command(left, right, now);

    dbg.steer = d.turn_right
        ? -(float)(((int16_t)outer - (int16_t)TURN_ROLL_INNER) / 2)
        : (float)(((int16_t)outer - (int16_t)TURN_ROLL_INNER) / 2);
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

static uint8_t axis_near(float heading, float tol_deg)
{
    return (uint8_t)(fabsf(drive_wrap180(heading - course_axis_snap(heading)))
        <= tol_deg);
}

static uint8_t axis_aligned(float heading)
{
    return axis_near(heading, TURN_AXIS_ALIGN_DEG);
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

static float course_dev_now(const DriveInputs *in)
{
    return fabsf(drive_wrap180(in->heading - d.course_heading));
}

/* course_heading is only trustworthy within a half revolution. A single
 * uninterrupted 180 deg sweep (turn_leg) proves the pocket demanded a course
 * reversal; the episode total must never unlock this — totals preserved
 * across backing chunks reach 180 in an ordinary corner fight (§5.16). */
static uint8_t exit_course_ok(const DriveInputs *in)
{
    if (!in->imu_live) return 1U;
    float dev = course_dev_now(in);
    dbg.course_dev = dev;
    return (uint8_t)(dev < COURSE_REV_DEG || d.turn_leg >= TURN_LEG_REVERSAL_DEG);
}

/* Wide-zone exits skip the axis gate (§5.19), so they need a tighter course
 * leash of their own: a 154 deg overshoot still passes the 115 deg reversal
 * gate and gets committed as a wrong-way cruise (§5.22, mapp_v1 32-36s).
 * The uninterrupted-sweep reversal escape stays available. */
static uint8_t wide_exit_course_ok(const DriveInputs *in)
{
    if (!in->imu_live) return 1U;
    return (uint8_t)(course_dev_now(in) < WIDE_EXIT_COURSE_DEV_DEG
        || d.turn_leg >= TURN_LEG_REVERSAL_DEG);
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
    uint8_t lp_was_valid = d.lp_valid;
    float l_lp_before = d.l_lp;
    float r_lp_before = d.r_lp;
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
    /* §5.37 per-flank closing rate (cm/s, negative = wall approaching).
     * Only accumulated across continuous valid frames — a reseed (§7-4)
     * would otherwise read as a huge fake step. */
    uint8_t l_cont = (uint8_t)(lp_was_valid && in->left_valid && d.l_prev_valid);
    uint8_t r_cont = (uint8_t)(lp_was_valid && in->right_valid && d.r_prev_valid);
    d.l_prev_valid = in->left_valid;
    d.r_prev_valid = in->right_valid;

    if (l_cont)
        d.l_rate_lp += CENTER_ACT_RATE_LPF_ALPHA
            * (((d.l_lp - l_lp_before) / dt) - d.l_rate_lp);
    else
        d.l_rate_lp = 0.0f;
    if (r_cont)
        d.r_rate_lp += CENTER_ACT_RATE_LPF_ALPHA
            * (((d.r_lp - r_lp_before) / dt) - d.r_rate_lp);
    else
        d.r_rate_lp = 0.0f;

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

    /* §5.36 proximity gate: pair centering only acts once a flank is inside
     * CENTER_ACT_SIDE_CM; released with hysteresis when both flanks clear.
     * Above the gate the axis hold alone keeps the line — the corridor
     * position is intentionally unregulated there (user spec).
     * §5.37: engage on the PREDICTED distance so a closing curve facet is
     * met with steering before the raw 14 cm crossing; a parallel straight
     * predicts ~unchanged and keeps the free-running band. */
    {
        float l_pred = d.l_lp + drive_clampf(d.l_rate_lp,
            -CENTER_ACT_RATE_MAX_CMS, 0.0f) * CENTER_ACT_LOOKAHEAD_S;
        float r_pred = d.r_lp + drive_clampf(d.r_rate_lp,
            -CENTER_ACT_RATE_MAX_CMS, 0.0f) * CENTER_ACT_LOOKAHEAD_S;
        uint8_t act_near = (uint8_t)((left_seen && l_pred < CENTER_ACT_SIDE_CM)
            || (right_seen && r_pred < CENTER_ACT_SIDE_CM));
        if (act_near)
        {
            d.center_act = 1U;
        }
        else if (d.center_act)
        {
            float rel = CENTER_ACT_SIDE_CM + CENTER_ACT_HYST_CM;
            if ((!left_seen || d.l_lp > rel) && (!right_seen || d.r_lp > rel))
                d.center_act = 0U;
        }
    }

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
        /* §5.36: outside the proximity gate the target decays to zero and the
         * heading path degenerates to a pure axis hold */
        float lateral_target = 0.0f;
        if (d.center_act)
        {
            lateral_target = (CENTER_LATERAL_KP_DEG_PER_CM * shaped_center_error(err_cm))
                           + (CENTER_LATERAL_KD_DEG_PER_CMS * derr);
            lateral_target = drive_clampf(lateral_target,
                -CENTER_LATERAL_CMD_MAX_DEG, CENTER_LATERAL_CMD_MAX_DEG);
        }
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

        /* single-wall regime (§5.22): only one reference wall means the far
         * flank gives no drift warning — a slow convergence at top speed
         * reaches the wall before the repel band can act (mapp_v1 38-40s:
         * 22 cm at 54% duty -> contact). Budget the speed to the wall distance. */
        if (!(left_track && right_track) && pair_valid == 0U)
        {
            float wd = left_track ? d.l_lp : d.r_lp;
            if (wd <= SPEED_SINGLE_SLOW_CM)
            {
                float scap;
                if (wd <= SIDE_SOFT_CM)
                    scap = SPEED_SINGLE_MIN_PCT;
                else
                    scap = SPEED_SINGLE_MIN_PCT
                         + ((SPEED_TOP_PCT - SPEED_SINGLE_MIN_PCT)
                            * (wd - SIDE_SOFT_CM)
                            / (SPEED_SINGLE_SLOW_CM - SIDE_SOFT_CM));
                if (speed > scap) speed = scap;
            }
        }
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
    d.stall_since_ms = 0U;
}

static void reset_confirm_counters(void)
{
    d.front_stop_n = 0U;
    d.decide_n = 0U;
    d.wall_gone_n = 0U;
    d.clear_n = 0U;
    d.side_clear_n = 0U;
}

static void cruise_enter(const DriveInputs *in)
{
    DriveState previous = d.state;
    uint8_t from_turn = (uint8_t)((previous == DS_SPIN || previous == DS_CORNER)
        && d.entry_heading_valid && in->imu_live);
    uint8_t reacquire = (uint8_t)(in->imu_live && d.reacquire_pending);

    centering_reset(in);
    /* §5.42: a roll turn translates through the corner and lands
     * wall-adjacent (mappf 15s: exit at R43mm) — engage the centering gate
     * immediately instead of free-running into the flank it drifted onto.
     * Hysteresis releases it as soon as both flanks actually clear. */
    if (previous == DS_SPIN && d.roll_turn)
        d.center_act = 1U;
    d.roll_turn = 0U;
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
 * episode accumulation; a fresh decide starts from zero.
 * roll: §5.39 rolling entry (clean stop-line commit only) — every recovery
 * or re-decide path pivots in place for precision. */
static void spin_begin(const DriveInputs *in, uint8_t keep_progress, uint8_t roll)
{
    reset_confirm_counters();
    if (!keep_progress)
    {
        turn_progress_reset(in->heading);
        d.flip_used = 0U;
    }
    d.roll_turn = roll;
    d.entry_heading_valid = in->imu_live;
    d.stuck_heading = in->heading;
    d.stuck_since_ms = in->now;
    d.spin_since_ms = in->now;
    state_enter(DS_SPIN, in->now);
    if (roll) command_roll(TURN_ROLL_OUTER, in->now, 1U);
    else command_pivot(TURN_SPEED, TURN_INNER, in->now, 1U);
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

/* At speed the 5 cm trigger fires with the nose already in the wall — an
 * oblique wall is specular to the ultrasonic (front reads OPEN while
 * closing, mapp_v4 18s: F80/L31mm), so the flank is the only pre-impact
 * signal and it must fire earlier the faster the car pushes (§5.26). */
static uint8_t side_emergency(const DriveInputs *in)
{
    int16_t duty_avg = (int16_t)((d.motion_left + d.motion_right) / 2);
    uint16_t limit = (duty_avg >= SIDE_FAST_DUTY_PCT)
        ? SIDE_AVOID_FAST_CM
        : SIDE_AVOID_CM;
    return (uint8_t)((in->left_valid && in->l < limit)
        || (in->right_valid && in->r < limit));
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

    /* §5.26 ram/stall backstop: an oblique wall returns no ultrasonic echo,
     * so the first sensor to see a nose-first ram is the LEFT ENCODER — duty
     * held high with the wheel speed collapsed is physical contact whatever
     * the front claims (mapp_v4 18s: F=80 "open", duty ~70, spd 4 cm/s).
     * Back straight off the wall, then brake-and-decide picks the turn.
     * Right encoder reads 0 permanently (hardware) — left only. */
    {
        int16_t duty_avg = (int16_t)((d.motion_left + d.motion_right) / 2);
        if (duty_avg >= CRUISE_STALL_DUTY_PCT
            && dbg.v_l < CRUISE_STALL_SPEED_CMS
            && (in->now - d.state_since_ms) >= CRUISE_STALL_GRACE_MS)
        {
            if (d.stall_since_ms == 0U)
            {
                d.stall_since_ms = in->now;
            }
            else if ((in->now - d.stall_since_ms) >= CRUISE_STALL_MS)
            {
                back_enter(in, 0U);
                return;
            }
        }
        else
        {
            d.stall_since_ms = 0U;
        }
    }

    /* §5.24 (user spec): a turn may only START from a close front wall — the
     * brake-and-decide path below. Side asymmetry alone must never initiate
     * a turn; outside the near-wall deadzone the sides only center/repel.
     * The rolling side-asymmetry corner arc (DS_CORNER) is deleted: with the
     * stop line at 34 cm it was reachable only from far side readings, and
     * those turns are what kept committing wrong-way runs. */

    /* -- stop line: fresh confirmed only, then the brake-and-decide path.
     * §5.39: a confirmed wall with an unambiguous L/R gap and a live IMU
     * commits to a ROLLING turn right here — no stop, no creep; direction is
     * still read from the sides at a front-confirmed line (§5.24 arming
     * unchanged; this is NOT the deleted side-initiated DS_CORNER arc).
     * Ties, single-flank reads and dead-IMU cases keep the stop path.
     * (§5.43 "무브레이크 통과 후 20cm 커밋"은 실차 악화로 롤백 — 재시도 금지.) */
    if (in->f_valid && in->f < FRONT_STOP_CM && d.front_stable)
    {
        d.front_stop_n = inc_u8(d.front_stop_n);
        if (d.front_stop_n >= FRONT_STOP_CONFIRM_N)
        {
            dbg.front_near = d.front_stop_n;
            if (in->imu_live && in->left_valid && in->right_valid
                && in->f >= TURN_ROLL_MIN_F_CM
                && (in->l > (uint16_t)(in->r + TURN_ROLL_GAP_CM)
                    || in->r > (uint16_t)(in->l + TURN_ROLL_GAP_CM)))
            {
                d.turn_right = (uint8_t)(in->r > in->l);
                spin_begin(in, 0U, 1U);
                return;
            }
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

static void spin_run(const DriveInputs *in)
{
    /* §5.42: the rolling profile only while there is room to roll — a front
     * inside the decide line or a flank at the hard band means the
     * translation budget is spent (mappf 18s: F 23->2 mid-roll). Finish the
     * sweep as an in-place pivot; profile swap only, never a backout. */
    if (d.roll_turn
        && (front_recent_below(in, FRONT_DECIDE_CM)
            || (in->left_valid && in->l < (uint16_t)SIDE_HARD_CM)
            || (in->right_valid && in->r < (uint16_t)SIDE_HARD_CM)))
    {
        d.roll_turn = 0U;
        command_pivot(TURN_SPEED, TURN_INNER, in->now, 1U);
    }

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
            d.roll_turn = 0U;      /* §5.39: recovery flips pivot in place */
            turn_progress_reset(in->heading);
            d.stuck_heading = in->heading;
            d.stuck_since_ms = in->now;
            d.spin_since_ms = in->now;
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

    /* §5.25: the old blocked-nose backout (front danger or front/side pinch,
     * 2 frames -> REVERSE) is deleted. A pivoting 27 cm car in a 37-45 cm
     * corridor swings its nose within 5-12 cm of the walls as a matter of
     * geometry, so that path kept cutting healthy turns into backing chunks
     * (mapp_v3 28s). A truly jammed pivot freezes its rotation and the wedge
     * check above catches it in 400 ms; everything else finishes the sweep. */

    /* §5.25 (user spec): the turn IS the heading change — reaching the
     * target completes it unconditionally, whatever the nose reads. A close
     * front after a finished 90 is cruise's job (it brakes and decides from
     * a stable pose); holding the exit hostage to "front open + axis pose"
     * is what ground past the target into reversals and wall hits. */
    if (progress_valid && progress >= TURN_TARGET_DEG && exit_course_ok(in))
    {
        cruise_enter(in);
        return;
    }

    /* §5.33 uniform sweep (user spec: every section turns like TARGET):
     * when the target exit stays gated — a recovery chain rotated the course
     * frame, so exit_course_ok holds the door shut — grinding on only lands
     * a deeper angle into the next wall (late-course rams). Past the slack,
     * back out and re-approach instead. Replaces the §5.22 wide-only budget
     * with an every-zone cap; the §5.16 true-reversal pocket now resolves
     * via the retry budget (3 chunks -> straight back + fresh decide). */
    if (progress_valid
        && d.turn_leg > (TURN_TARGET_DEG + TURN_OVER_SLACK_DEG))
    {
        back_enter(in, 1U);
        return;
    }

    /* early exit: enough rotation, open nose, course gate. Alignment may come
     * from the course axis, from having no corridor to align to (§5.19), or —
     * per user spec §5.24 (over-turn guard) — from a STRONGLY opened nose.
     * §5.27: the strong-open exit additionally demands a NEAR-axis pose — in
     * a corner pocket the diagonal across the opening also reads "clear", and
     * exiting there 40 deg short punched the car into the outer wall
     * (mapp_v5 20-21s: exit at +143 vs axis 180 -> L45mm hit -> second
     * pivot). Every non-axis exit keeps the tighter §5.22 course leash. */
    if (progress_valid && progress >= TURN_MIN_DEG
        && (axis_aligned(in->heading)
            || (sides_open_wide(in) && wide_exit_course_ok(in))
            || (front_open_at(in, FRONT_CLEAR_CM)
                && axis_near(in->heading, TURN_EXIT_ALIGN_LOOSE_DEG)
                && wide_exit_course_ok(in)))
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

    /* §5.30 (user spec): no taper — the pivot runs at full turn duty until
     * the target sweep is reached, then cruise takes over instantly. The
     * gradual PID convergence is what read as "slowly settling into the
     * angle"; skid carry-through is absorbed by TURN_TARGET_DEG's lead.
     * §5.39: a rolling entry keeps its rolling profile for the whole sweep. */
    if (d.roll_turn) command_roll(TURN_ROLL_OUTER, in->now, 0U);
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
            spin_begin(in, 0U, 0U);
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
    if (in->f_valid && in->f <= FRONT_DECIDE_CM && d.front_stable)
    {
        d.decide_n = inc_u8(d.decide_n);
        if (d.decide_n >= FRONT_DECIDE_CONFIRM_N
            && (in->left_valid || in->right_valid))
        {
            decide_turn_direction(in);
            spin_begin(in, 0U, 0U);
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
        spin_begin(in, 0U, 0U);
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
                spin_begin(in, 1U, 0U);
            }
            else
            {
                spin_begin(in, 0U, 0U);
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

    /* Front stability chain (§5.23): a real wall approached head-on moves a
     * few cm per frame; floor/bump echoes teleport (3->30->16). Confirm
     * counters that arm brakes, corners and direction decisions only count
     * consecutive agreeing samples — the danger ladder stays ungated. */
    d.front_stable = (uint8_t)(in->f_valid && d.f_prev_valid
        && ((in->f > d.f_prev) ? (in->f - d.f_prev) : (d.f_prev - in->f))
           <= FRONT_STABLE_CM);
    d.f_prev_valid = in->f_valid;
    if (in->f_valid) d.f_prev = in->f;

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
