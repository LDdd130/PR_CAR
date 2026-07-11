#include "drive_control.h"
#include "drive_math.h"

#include <math.h>

typedef struct
{
    uint8_t fresh;
    uint8_t imu_prev_live;
    uint8_t lp_valid;
    uint8_t left_prev_valid;
    uint8_t right_prev_valid;
    uint8_t error_valid;
    uint8_t axis_resnap_count;
    float previous_error;
    float previous_heading;
    float previous_steer;
    float lateral_command;
    float left_lp;
    float right_lp;
    float error_rate_lp;
    float yaw_rate_lp;
    uint32_t previous_ms;
} ControlContext;

typedef struct
{
    float left_cm;
    float right_cm;
    float error_cm;
    float error_rate_cms;
    float yaw_rate_dps;
    float heading_error_deg;
    uint8_t left_track;
    uint8_t right_track;
    uint8_t pair_valid;
    uint8_t front_open;
    DriveCorridorClass corridor;
} ControlSample;

typedef struct
{
    float lateral_kp;
    float lateral_kd;
    float fast_speed;
    float fast_error_cm;
    float fast_heading_deg;
    float settle_speed;
    float heading_fast_deg;
    float heading_min_speed;
    float href_align_error_cm;
} ControlProfile;

static ControlContext control = { .fresh = 1U };

static float sanitize_distance(uint16_t distance_cm, uint8_t valid)
{
    if (!valid || distance_cm == 0U || distance_cm > CENTER_SENSOR_MAX_CM)
        return (float)CENTER_SENSOR_MAX_CM;
    return (float)distance_cm;
}

static float approach(float current, float target, float max_step)
{
    return drive_clampf(target, current - max_step, current + max_step);
}

static float shaped_center_error(float error_cm)
{
    float magnitude = fabsf(error_cm);
    if (magnitude <= CENTER_DEADZONE_CM)
        return error_cm * CENTER_INNER_KP_SCALE;

    float shaped = (magnitude - CENTER_DEADZONE_CM)
                 + (CENTER_DEADZONE_CM * CENTER_INNER_KP_SCALE);
    return (error_cm >= 0.0f) ? shaped : -shaped;
}

/* Keep normal centering continuous across the motor floor. Near a wall, shed
 * common speed so the same differential produces a tighter escape path. */
static void mix_substall(float *left_pct, float *right_pct, uint8_t wall_risk)
{
    float slow = fminf(*left_pct, *right_pct);
    if (slow <= 0.0f || slow >= (float)MOTOR_MIN_PCT) return;

    float differential = fabsf(*right_pct - *left_pct);
    if (!wall_risk)
    {
        float lift = (float)MOTOR_MIN_PCT - slow;
        *left_pct += lift;
        *right_pct += lift;
        return;
    }

    float fast = fmaxf(differential, (float)MOTOR_MIN_PCT);
    if (*left_pct < *right_pct) { *left_pct = 0.0f; *right_pct = fast; }
    else                        { *right_pct = 0.0f; *left_pct = fast; }
}

static float update_samples(const DriveInputs *in, ControlSample *sample)
{
    float left_raw = sanitize_distance(in->l, in->left_valid);
    float right_raw = sanitize_distance(in->r, in->right_valid);

    if (!control.lp_valid)
    {
        control.left_lp = left_raw;
        control.right_lp = right_raw;
        control.lp_valid = 1U;
    }
    else
    {
        if (!in->left_valid)
            control.left_lp = (float)CENTER_SENSOR_MAX_CM;
        else if (!control.left_prev_valid)
            control.left_lp = left_raw;
        else
            control.left_lp = (control.left_lp * (1.0f - CENTER_LPF_ALPHA))
                            + (left_raw * CENTER_LPF_ALPHA);

        if (!in->right_valid)
            control.right_lp = (float)CENTER_SENSOR_MAX_CM;
        else if (!control.right_prev_valid)
            control.right_lp = right_raw;
        else
            control.right_lp = (control.right_lp * (1.0f - CENTER_LPF_ALPHA))
                             + (right_raw * CENTER_LPF_ALPHA);
    }
    control.left_prev_valid = in->left_valid;
    control.right_prev_valid = in->right_valid;

    sample->left_cm = control.left_lp;
    sample->right_cm = control.right_lp;
    sample->error_cm = sample->left_cm - sample->right_cm;

    uint8_t left_seen = (uint8_t)(in->left_valid && sample->left_cm < CENTER_SENSOR_MAX_CM);
    uint8_t right_seen = (uint8_t)(in->right_valid && sample->right_cm < CENTER_SENSOR_MAX_CM);
    sample->pair_valid = (uint8_t)(left_seen && right_seen
        && (sample->left_cm + sample->right_cm) <= CENTER_SIDE_PAIR_MAX_CM);
    sample->corridor = sample->pair_valid
        ? drive_corridor_class(sample->left_cm, sample->right_cm)
        : DRIVE_CORRIDOR_UNKNOWN;
    sample->front_open = (uint8_t)(in->f_valid && in->f >= FRONT_ARC_CM);

    sample->left_track = left_seen;
    sample->right_track = right_seen;
    if (!sample->pair_valid && left_seen && right_seen)
    {
        if (sample->left_cm <= sample->right_cm) sample->right_track = 0U;
        else                                      sample->left_track = 0U;
    }
    if (sample->left_track && sample->left_cm > CENTER_SINGLE_MAX_CM)
        sample->left_track = 0U;
    if (sample->right_track && sample->right_cm > CENTER_SINGLE_MAX_CM)
        sample->right_track = 0U;

    float dt = 0.02f;
    if (control.previous_ms != 0U && in->now > control.previous_ms)
        dt = drive_clampf((float)(in->now - control.previous_ms) / 1000.0f, 0.005f, 0.100f);

    float raw_error_rate = (sample->pair_valid && control.error_valid && control.previous_ms != 0U)
        ? (sample->error_cm - control.previous_error) / dt
        : 0.0f;
    if (sample->pair_valid && control.error_valid)
        control.error_rate_lp += CENTER_DERR_LPF_ALPHA
            * (raw_error_rate - control.error_rate_lp);
    else
        control.error_rate_lp = 0.0f;
    sample->error_rate_cms = drive_clampf(control.error_rate_lp,
        -CENTER_DERR_MAX_CMS, CENTER_DERR_MAX_CMS);

    float raw_yaw_rate = (in->imu_live && control.imu_prev_live && control.previous_ms != 0U)
        ? drive_wrap180(in->heading - control.previous_heading) / dt
        : 0.0f;
    if (in->imu_live && control.imu_prev_live)
        control.yaw_rate_lp += CENTER_YAW_LPF_ALPHA
            * (raw_yaw_rate - control.yaw_rate_lp);
    else
        control.yaw_rate_lp = 0.0f;
    sample->yaw_rate_dps = drive_clampf(control.yaw_rate_lp,
        -CENTER_YAW_RATE_MAX_DPS, CENTER_YAW_RATE_MAX_DPS);

    control.previous_error = sample->error_cm;
    control.error_valid = sample->pair_valid;
    control.previous_ms = in->now;
    if (in->imu_live) control.previous_heading = in->heading;
    control.imu_prev_live = in->imu_live;
    return dt;
}

static ControlProfile select_profile(const ControlSample *sample)
{
    ControlProfile profile = {
        .lateral_kp = CENTER_LATERAL_KP_DEG_PER_CM,
        .lateral_kd = CENTER_LATERAL_KD_DEG_PER_CMS,
        .fast_speed = CENTER_STRAIGHT_FAST_SPEED_PCT,
        .fast_error_cm = CENTER_STRAIGHT_FAST_ERR_CM,
        .fast_heading_deg = CENTER_STRAIGHT_FAST_HDG_DEG,
        .settle_speed = CENTER_SETTLE_SPEED_PCT,
        .heading_fast_deg = CENTER_HDG_FAST_DEG,
        .heading_min_speed = CENTER_HDG_MIN_SPEED_PCT,
        .href_align_error_cm = CENTER_HREF_ALIGN_ERR_CM,
    };

    if (sample->corridor == DRIVE_CORRIDOR_NARROW)
    {
        profile.lateral_kp = CENTER_NARROW_KP_DEG_PER_CM;
        profile.fast_speed = CENTER_NARROW_FAST_SPEED_PCT;
        profile.fast_error_cm = 3.0f;
        profile.fast_heading_deg = 4.0f;
    }
    else if (sample->corridor == DRIVE_CORRIDOR_WIDE)
    {
        profile.lateral_kp = CENTER_WIDE_KP_DEG_PER_CM;
        profile.fast_speed = CENTER_WIDE_FAST_SPEED_PCT;
        profile.fast_error_cm = 7.0f;
        profile.fast_heading_deg = 6.0f;
        profile.settle_speed = CENTER_WIDE_SETTLE_SPEED_PCT;
        profile.heading_fast_deg = 6.0f;
        profile.heading_min_speed = 42.0f;
        profile.href_align_error_cm = 6.0f;
    }
    else if (sample->corridor == DRIVE_CORRIDOR_MID)
    {
        profile.settle_speed = CENTER_MID_SETTLE_SPEED_PCT;
        profile.heading_min_speed = 38.0f;
    }

    if (sample->front_open && sample->pair_valid)
    {
        profile.lateral_kp = CENTER_STRAIGHT_KP_DEG_PER_CM;
        profile.lateral_kd = CENTER_STRAIGHT_KD_DEG_PER_CMS;
        profile.fast_error_cm = CENTER_STRAIGHT_FAST_ERR_CM;
        profile.fast_heading_deg = CENTER_STRAIGHT_FAST_HDG_DEG;
        profile.heading_fast_deg = CENTER_STRAIGHT_FAST_HDG_DEG;
        if (profile.heading_min_speed < 44.0f) profile.heading_min_speed = 44.0f;
    }
    return profile;
}

static float update_heading_reference(const DriveInputs *in,
                                      const ControlSample *sample,
                                      const ControlProfile *profile,
                                      float heading_ref,
                                      float grid_heading,
                                      uint8_t *resnapped)
{
    *resnapped = 0U;
    if (!sample->pair_valid || !in->imu_live)
    {
        control.axis_resnap_count = 0U;
        return heading_ref;
    }

    if (!sample->front_open
        && fabsf(sample->error_cm) <= profile->href_align_error_cm
        && fabsf(sample->heading_error_deg) <= CENTER_HREF_ALIGN_HDG_DEG
        && fabsf(sample->error_rate_cms) <= CENTER_HREF_ALIGN_DERR_CMS
        && fabsf(sample->yaw_rate_dps) <= CENTER_HREF_ALIGN_YAW_DPS)
    {
        heading_ref = drive_wrap360(heading_ref
            + drive_wrap180(in->heading - heading_ref) * CENTER_HREF_BLEND);
    }

    if (fabsf(sample->error_cm) <= (CENTER_HREF_ALIGN_ERR_CM + 1.0f)
        && fabsf(sample->error_rate_cms) <= CENTER_HREF_ALIGN_DERR_CMS
        && fabsf(sample->yaw_rate_dps) <= CENTER_AXIS_RESNAP_YAW_DPS
        && fabsf(sample->heading_error_deg) >= CENTER_AXIS_RESNAP_DEG)
    {
        if (control.axis_resnap_count < 255U) control.axis_resnap_count++;
        if (control.axis_resnap_count >= CENTER_AXIS_RESNAP_N)
        {
            heading_ref = grid_heading;
            control.axis_resnap_count = 0U;
            *resnapped = 1U;
        }
    }
    else
    {
        control.axis_resnap_count = 0U;
    }
    return heading_ref;
}

static float compute_steer(const DriveInputs *in,
                           const ControlSample *sample,
                           const ControlProfile *profile,
                           float dt,
                           float heading_ref,
                           float *guard_risk_cm,
                           uint8_t *mode)
{
    float heading_error = in->imu_live ? drive_wrap180(in->heading - heading_ref) : 0.0f;
    float steer = 0.0f;

    if (sample->pair_valid)
    {
        float lateral_target = (profile->lateral_kp * shaped_center_error(sample->error_cm))
                             + (profile->lateral_kd * sample->error_rate_cms);
        lateral_target = drive_clampf(lateral_target,
            -CENTER_LATERAL_CMD_MAX_DEG, CENTER_LATERAL_CMD_MAX_DEG);
        control.lateral_command = approach(control.lateral_command, lateral_target,
            CENTER_LATERAL_CMD_SLEW_DPS * dt);

        *mode = DRIVE_STEER_BOTH;
        if (in->imu_live)
        {
            float desired_heading = drive_wrap360(heading_ref - control.lateral_command);
            float target_error = drive_wrap180(in->heading - desired_heading);
            steer = CENTER_HDG_KP_PCT_PER_DEG
                  * drive_deadbandf(target_error, CENTER_HDG_DEADBAND_DEG);
        }
        else
        {
            steer = CENTER_HDG_KP_PCT_PER_DEG * control.lateral_command;
        }
    }
    else if (sample->left_track || sample->right_track)
    {
        control.lateral_command = 0.0f;
        float wall_steer = sample->left_track
            ? CENTER_SINGLE_KP * (sample->left_cm - CENTER_SINGLE_TARGET_CM)
            : CENTER_SINGLE_KP * (CENTER_SINGLE_TARGET_CM - sample->right_cm);
        if (sample->left_track && wall_steer > 0.0f) wall_steer = 0.0f;
        if (!sample->left_track && wall_steer < 0.0f) wall_steer = 0.0f;

        *mode = DRIVE_STEER_SINGLE;
        steer = wall_steer;
        if (in->imu_live)
            steer += CENTER_OPEN_HDG_KP_PCT_PER_DEG * CENTER_SINGLE_HDG_BLEND
                   * drive_deadbandf(heading_error, CENTER_HDG_DEADBAND_DEG);
    }
    else if (in->imu_live)
    {
        control.lateral_command = 0.0f;
        *mode = DRIVE_STEER_HEADING;
        steer = CENTER_OPEN_HDG_KP_PCT_PER_DEG
              * drive_deadbandf(heading_error, CENTER_HDG_DEADBAND_DEG);
    }
    else
    {
        control.lateral_command = 0.0f;
        *mode = DRIVE_STEER_OPEN;
    }

    /* Progressive wall repel runs regardless of pair validity: it is the only
     * steering term that needs no heading reference, so it keeps pushing off
     * a wall even when heading_ref itself has drifted toward that wall. With
     * both walls in range the two quadratic terms blend into a centering
     * spring; off-center their difference dominates. */
    {
        float repel_span = CENTER_SIDE_SOFT_CM - CENTER_SIDE_HARD_CM;
        if (sample->left_track && sample->left_cm < CENTER_SIDE_SOFT_CM)
        {
            float depth = CENTER_SIDE_SOFT_CM - sample->left_cm;
            steer -= CENTER_SIDE_REPEL_KP * depth * depth / repel_span;
        }
        if (sample->right_track && sample->right_cm < CENTER_SIDE_SOFT_CM)
        {
            float depth = CENTER_SIDE_SOFT_CM - sample->right_cm;
            steer += CENTER_SIDE_REPEL_KP * depth * depth / repel_span;
        }
    }

    float left_guard = CENTER_NEAR_GUARD_CM;
    float right_guard = CENTER_NEAR_GUARD_CM;
    if (in->imu_live)
    {
        float yaw = drive_clampf(fabsf(heading_error), 0.0f, CENTER_YAW_SWEEP_MAX_DEG);
        float sweep = CENTER_BODY_HALF_LEN_CM * sinf(yaw * 0.0174532925f) * CENTER_YAW_SWEEP_GAIN;
        if (heading_error > CENTER_HDG_DEADBAND_DEG) right_guard += sweep;
        else if (heading_error < -CENTER_HDG_DEADBAND_DEG) left_guard += sweep;
    }

    *guard_risk_cm = 0.0f;
    if (sample->left_track || sample->right_track)
    {
        float left_risk = sample->left_track ? left_guard - sample->left_cm : -1000.0f;
        float right_risk = sample->right_track ? right_guard - sample->right_cm : -1000.0f;
        if (left_risk > 0.0f || right_risk > 0.0f)
        {
            if (left_risk >= right_risk)
            {
                *guard_risk_cm = left_risk;
                steer -= CENTER_NEAR_GUARD_KP * left_risk;
            }
            else
            {
                *guard_risk_cm = right_risk;
                steer += CENTER_NEAR_GUARD_KP * right_risk;
            }
        }
    }

    if (in->imu_live)
    {
        float yaw_control = drive_deadbandf(sample->yaw_rate_dps, CENTER_YAW_RATE_DEADBAND_DPS);
        steer += drive_clampf(CENTER_YAW_KD_PCT_PER_DPS * yaw_control,
            -CENTER_YAW_DAMP_MAX_PCT, CENTER_YAW_DAMP_MAX_PCT);
    }

    steer = drive_clampf(steer, -CENTER_STEER_MAX_PCT, CENTER_STEER_MAX_PCT);
    steer = approach(control.previous_steer, steer, CENTER_STEER_SLEW_PCT_PER_S * dt);
    control.previous_steer = steer;
    return steer;
}

static float compute_speed(const DriveInputs *in,
                           const ControlSample *sample,
                           const ControlProfile *profile,
                           float steer,
                           float guard_risk_cm,
                           uint32_t cruise_elapsed_ms)
{
    float speed_top = CENTER_BASE_SPEED_PCT;
    if (sample->pair_valid && in->f_valid && in->f >= CENTER_FRONT_FAST_CM && guard_risk_cm <= 0.0f
        && fabsf(sample->error_cm) <= profile->fast_error_cm
        && (!in->imu_live || fabsf(sample->heading_error_deg) <= profile->fast_heading_deg))
    {
        speed_top = profile->fast_speed;
    }

    float steer_ratio = fabsf(steer) / CENTER_STEER_MAX_PCT;
    float speed = speed_top - ((speed_top - CENTER_MIN_SPEED_PCT) * steer_ratio);

    if (in->f_valid)
    {
        float front_cap = speed_top;
        if ((float)in->f <= CENTER_FRONT_SLOW_CM)
            front_cap = CENTER_FRONT_MIN_SPEED_PCT;
        else if ((float)in->f < CENTER_FRONT_FAST_CM)
        {
            float ratio = ((float)in->f - CENTER_FRONT_SLOW_CM)
                        / (CENTER_FRONT_FAST_CM - CENTER_FRONT_SLOW_CM);
            front_cap = CENTER_FRONT_MIN_SPEED_PCT
                      + ((speed_top - CENTER_FRONT_MIN_SPEED_PCT) * ratio);
        }
        if (speed > front_cap) speed = front_cap;
    }

    if (sample->left_track || sample->right_track)
    {
        float side_min = sample->left_track ? sample->left_cm : sample->right_cm;
        if (sample->right_track && sample->right_cm < side_min) side_min = sample->right_cm;

        float side_cap = speed_top;
        if (side_min <= CENTER_SIDE_HARD_CM)
            side_cap = CENTER_SIDE_MIN_SPEED_PCT;
        else if (side_min < CENTER_SIDE_SOFT_CM)
        {
            float ratio = (side_min - CENTER_SIDE_HARD_CM)
                        / (CENTER_SIDE_SOFT_CM - CENTER_SIDE_HARD_CM);
            side_cap = CENTER_SIDE_MIN_SPEED_PCT
                     + ((speed_top - CENTER_SIDE_MIN_SPEED_PCT) * ratio);
        }
        if (speed > side_cap) speed = side_cap;

        if (guard_risk_cm > 0.0f)
        {
            float ratio = drive_clampf(guard_risk_cm / CENTER_GUARD_FULL_DEPTH_CM, 0.0f, 1.0f);
            float guard_cap = speed_top
                - ((speed_top - CENTER_GUARD_MIN_SPEED_PCT) * ratio);
            if (speed > guard_cap) speed = guard_cap;
        }
    }

    if (in->imu_live)
    {
        float heading_abs = fabsf(sample->heading_error_deg);
        float heading_cap = speed_top;
        if (heading_abs >= CENTER_HDG_SLOW_DEG)
            heading_cap = profile->heading_min_speed;
        else if (heading_abs > profile->heading_fast_deg)
        {
            float ratio = (heading_abs - profile->heading_fast_deg)
                        / (CENTER_HDG_SLOW_DEG - profile->heading_fast_deg);
            heading_cap = speed_top
                - ((speed_top - profile->heading_min_speed) * ratio);
        }
        if (speed > heading_cap) speed = heading_cap;
    }

    if (cruise_elapsed_ms < CENTER_SETTLE_MS && speed > profile->settle_speed)
        speed = profile->settle_speed;
    return drive_clampf(speed, CENTER_SIDE_MIN_SPEED_PCT, speed_top);
}

void DriveControl_Reset(void)
{
    control = (ControlContext){ .fresh = 1U };
}

void DriveControl_PrimeHeading(float heading_deg)
{
    control.previous_heading = heading_deg;
    control.imu_prev_live = 1U;
    control.fresh = 0U;
}

void DriveControl_Run(const DriveInputs *in,
                      float heading_ref_deg,
                      float grid_heading_deg,
                      uint32_t cruise_elapsed_ms,
                      DriveControlOutput *out)
{
    if (control.fresh)
    {
        if (in->imu_live)
        {
            heading_ref_deg = grid_heading_deg;
            control.previous_heading = in->heading;
            control.imu_prev_live = 1U;
        }
        control.fresh = 0U;
    }
    else if (in->imu_live && !control.imu_prev_live)
    {
        heading_ref_deg = grid_heading_deg;
        control.previous_heading = in->heading;
    }

    ControlSample sample = {0};
    float dt = update_samples(in, &sample);
    sample.heading_error_deg = in->imu_live
        ? drive_wrap180(in->heading - heading_ref_deg)
        : 0.0f;

    ControlProfile profile = select_profile(&sample);
    heading_ref_deg = update_heading_reference(in, &sample, &profile,
        heading_ref_deg, grid_heading_deg, &out->axis_resnapped);
    sample.heading_error_deg = in->imu_live
        ? drive_wrap180(in->heading - heading_ref_deg)
        : 0.0f;

    float guard_risk_cm;
    uint8_t mode;
    float steer = compute_steer(in, &sample, &profile, dt, heading_ref_deg,
        &guard_risk_cm, &mode);
    float speed = compute_speed(in, &sample, &profile, steer, guard_risk_cm,
        cruise_elapsed_ms);

    float left = drive_clampf(speed - steer + (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
    float right = drive_clampf(speed + steer - (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
    mix_substall(&left, &right, (uint8_t)(guard_risk_cm > 0.0f));

    out->left_pct = (int16_t)(left + 0.5f);
    out->right_pct = (int16_t)(right + 0.5f);
    out->steer_pct = steer;
    out->yaw_rate_dps = sample.yaw_rate_dps;
    out->heading_error_deg = sample.heading_error_deg;
    out->heading_ref_deg = heading_ref_deg;
    out->mode = mode;
}
