#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

/* ============================================================================
 * PR_CAR drive core v2 configuration.
 *
 * Rewritten together with drive.c (§5.17): three logical layers — CRUISE
 * (centering + speed governor), TURN (arc primary, pivot fallback) and
 * RECOVER (brake / back / side rescue) — reported through the unchanged
 * seven-state telemetry enum.
 *
 * Side geometry assumes the ToF pair is now mounted PERPENDICULAR (90 deg)
 * to the chassis, so l/r are true lateral clearances. Values marked
 * [REMEASURE] are derived from track drawings (testtrack.drawio: corridor
 * widths 37/45/43/55/50/67/50/60 cm, car 16x27 cm) and must be checked
 * against a manual run before serious tuning.
 * ==========================================================================*/

/* ---- Motor duties (% duty). Measured chassis facts, do not lower. -------
 * Straight-line stall floor is 30%. Pivoting is four-wheel skid steering:
 * breakaway sits far above that floor, and an inner pair commanded below
 * the floor stalls and drags the rotation centre onto the stopped side. */
#define DRIVE_SPEED                     32
#define TURN_SPEED                      56
#define TURN_INNER                      34
#define MOTOR_MIN_PCT                   30
#define MOTOR_TRIM_PCT                   0
#define DRIVE_NOMINAL_UPDATE_MS         20U
#define DRIVE_WHEEL_SLEW_PCT_PER_S    600.0f

/* Forward arc (rolling corner). */
#define ARC_OUTER                       72
#define ARC_INNER                       30
#define ARC_APPROACH_OUTER              56
#define ARC_APPROACH_INNER              30
#define ARC_APPROACH_DEG                18.0f
#define ARC_MAX_MS                     850U

/* Rescue pivot (side escape) and three-point-turn backing. */
#define RESCUE_OUTER                    48
#define RESCUE_INNER                    32
#define REV_TURN_OUTER                  46
#define REV_TURN_INNER                  32

/* ---- Front distance ladder (cm, HC-SR04 — hardware unchanged). ----------
 * DANGER < DECIDE < STOP < TURN < CLEAR < ARC, guarded below.
 * DECIDE 20 is a user-mandated spec: a pivot direction is only decided from
 * a wall firmly confirmed at 20 cm by consecutive fresh samples (§5.9). */
#define FRONT_DANGER_CM                 12
#define FRONT_DECIDE_CM                 20
#define FRONT_STOP_CM                   34
#define FRONT_TURN_CM                   44
#define FRONT_CLEAR_CM                  52
#define FRONT_ARC_CM                    68
#define CORNER_TIGHTEN_CM               30

/* Fresh-confirm counts (frames). Front reading spikes (floor echo, bounce):
 * no single sample may brake, open a corner or decide a direction (§5.9). */
#define FRONT_WALL_CONFIRM_N             3U
#define FRONT_STOP_CONFIRM_N             3U
#define FRONT_DECIDE_CONFIRM_N           2U
#define CLEAR_CONFIRM_N                  3U
#define SPIN_BLOCK_CONFIRM_N             2U

/* ---- Side geometry (cm, perpendicular ToF). [REMEASURE] ------------------
 * Centered lateral clearance by corridor: 10.5 (37 cm) .. 25.5 (67 cm).
 * A corner opening reads the next corridor's depth, well past any centered
 * clearance; an off-center run in the widest zone can still fake large far
 * readings, which is why corner detection additionally REQUIRES the
 * confirmed approaching front wall above. */
#define SIDE_PAIR_MAX_CM                52.0f
#define SIDE_OPEN_CM                    30
#define SIDE_ASYM_CM                    12
#define SIDE_NEAR_SAFE_CM                6
#define SIDE_HYST_CM                     5
#define CORNER_CONFIRM_N                 2U
#define CORNER_DROPOUT_TOL_N             1U

/* Wall proximity bands. [REMEASURE] */
#define SIDE_SOFT_CM                    10.0f
#define SIDE_HARD_CM                     7.0f
#define SIDE_AVOID_CM                    5
#define SIDE_AVOID_CLEAR_CM              7
#define SIDE_AVOID_CLEAR_CONFIRM_N       3U
#define SIDE_AVOID_MAX_MS              360U
#define SIDE_AVOID_PIVOT_MAX_DEG        18.0f
#define SIDE_AVOID_ESCAPE_PCT           36

/* Course-specific fallback: the final right-angle before the speed bump is
 * geometrically symmetric (L≈R), so with a confirmed wall and tied sides the
 * course axis decides. Course axes are boot-relative 90 deg multiples. */
#define COURSE_FINAL_ENTRY_DEG          90.0f
#define COURSE_FINAL_ENTRY_TOL_DEG      20.0f

/* ---- Turn (arc + pivot). --------------------------------------------------
 * Progress is ALWAYS wrap180-increment accumulation (never heading-entry):
 * turn_accum survives backing chunks so exit gates need no re-rotation;
 * turn_leg resets at every interruption and is the ONLY value allowed to
 * unlock a course reversal (>=180 in one uninterrupted sweep, §5.16). */
#define TURN_TARGET_DEG                 88.0f
#define TURN_MIN_DEG                    30.0f
#define TURN_WRONG_DEG                  20.0f
#define TURN_AXIS_ALIGN_DEG             10.0f
/* Wide/curve zone (§5.19): with both sides at least this open the track is
 * not a 90 deg corridor (octagon plaza facets sit at 45 deg), so a turn may
 * exit off-axis as soon as the nose opens — the axis-align gate only binds
 * where a corridor exists to align to. 30 == SIDE_OPEN_CM by design. */
#define TURN_WIDE_EXIT_SIDE_CM          30
#define COURSE_REV_DEG                 115.0f
#define TURN_LEG_REVERSAL_DEG          180.0f
#define SPIN_COMMIT_MS                 100U
#define SPIN_MAX_MS                   1600U
#define SPIN_BLIND_MS                  650U
#define ROT_STUCK_MS                   400U
#define ROT_STUCK_DEG                    8.0f

/* Pivot heading PID. Floors sit above skid breakaway, not the stall floor. */
#define TURN_PID_KP                      0.62f
#define TURN_PID_KI                      0.018f
#define TURN_PID_KD                      0.14f
#define TURN_PID_I_MAX                 130.0f
#define TURN_PID_MIN_PCT                44
#define TURN_PID_FINE_MIN_PCT           40
#define TURN_PID_FINE_DEG               14.0f
#define TURN_PID_INNER_RATIO             0.48f

/* ---- Recover. -------------------------------------------------------------
 * A blocked pivot backs out with a three-point-turn chunk (yaw preserved)
 * and resumes; straight backing is reserved for brake-line walls and dead
 * ends. The retry budget is per turn episode; when spent, backing degrades
 * to straight + a fresh direction decision instead of deadlocking (§5.15). */
#define BRAKE_MS                        80U
#define BRAKE_CREEP_PCT                 32
#define BRAKE_CREEP_MAX_MS             700U
#define BACK_CHUNK_MS                  220U
#define RECOVER_RETRY_MAX                3U
#define LAUNCH_MS                      120U

/* ---- Cruise centering (perpendicular ToF). --------------------------------
 * Noise is removed by LPFs, never by wide deadbands (§5.8): a deadband big
 * enough to hide heading vibration leaves the corridor position unregulated
 * and the car weaves between the repel zones. Knee gains add authority only
 * past the weave regime (§5.15). */
#define CENTER_SENSOR_MAX_CM            80U
#define CENTER_LPF_ALPHA                 0.45f
#define CENTER_DERR_LPF_ALPHA            0.28f
#define CENTER_DERR_MAX_CMS             35.0f
#define CENTER_YAW_LPF_ALPHA             0.22f
#define CENTER_YAW_RATE_MAX_DPS         90.0f
#define CENTER_HDG_ERR_LPF_ALPHA         0.25f

#define CENTER_DEADZONE_CM               3.0f
#define CENTER_LATERAL_KP_DEG_PER_CM     0.45f
#define CENTER_LATERAL_KD_DEG_PER_CMS    0.045f
#define CENTER_LATERAL_KNEE_CM          10.0f
#define CENTER_LATERAL_KNEE_GAIN         1.0f
/* §5.19: the old 12 deg graze/entry band died with the 45-deg mount; the
 * bound is now the recovery authority vs SPEED_HDG_SLOW_DEG budget (the
 * governor is already at its floor before the lateral command saturates). */
#define CENTER_LATERAL_CMD_MAX_DEG      15.0f
#define CENTER_LATERAL_CMD_SLEW_DPS    110.0f

#define CENTER_HDG_KP_PCT_PER_DEG        0.80f
#define CENTER_HDG_KNEE_DEG             10.0f
#define CENTER_HDG_KP2_PCT_PER_DEG       0.85f
#define CENTER_HDG_DEADBAND_DEG          0.6f
#define CENTER_YAW_KD_PCT_PER_DPS        0.34f
#define CENTER_YAW_RATE_DEADBAND_DPS     8.0f
#define CENTER_YAW_DAMP_MAX_PCT          9.0f

#define CENTER_SIDE_REPEL_KP             1.9f
#define CENTER_SINGLE_TARGET_CM         13.0f
#define CENTER_SINGLE_KP                 0.22f
#define CENTER_SINGLE_MAX_CM            30.0f
#define CENTER_SINGLE_HDG_BLEND          0.42f
#define CENTER_OPEN_HDG_KP_PCT_PER_DEG   0.58f

/* Off-axis freewheel (§5.19): with no corridor on either flank a large
 * heading error is the curve's fault, not the car's — past this error the
 * axis-hold P term disengages (yaw damping stays) instead of pivot-fighting
 * the plaza. Corridors never trigger it: a tracked wall (<30 cm) or a valid
 * pair keeps the full axis hold, so §5.10 overshoot recovery is untouched. */
#define CENTER_WIDE_SIDE_CM             30.0f
#define CENTER_OPEN_FREE_DEG            25.0f

#define CENTER_STEER_MAX_PCT            26.0f
#define CENTER_STEER_MAX_ALIGN_PCT      36.0f
#define CENTER_STEER_ALIGN_ERR_DEG      14.0f
#define CENTER_STEER_SLEW_PCT_PER_S    240.0f

/* Sub-stall mixing: both wheels above breakaway near a wall (§5.13). */
#define CENTER_WALL_INNER_FLOOR_PCT     36.0f
#define CENTER_WALL_DIFF_MAX_PCT        18.0f

/* ---- Speed governor. ------------------------------------------------------
 * FIRST-CLASS RULE: speed follows the sensing/braking budget. The front cap
 * alone guarantees the car arrives at the stop line slow enough to brake,
 * creep and decide instead of nosing the wall (the v1 instability root).
 * Every cap is a single linear ramp; the lowest one wins. */
#define SPEED_TOP_PCT                   60.0f
#define SPEED_MIN_PCT                   32.0f
#define SPEED_FRONT_FAST_CM             75.0f
#define SPEED_FRONT_SLOW_CM             30.0f
#define SPEED_FRONT_MIN_PCT             34.0f
#define SPEED_SIDE_MIN_PCT              32.0f
#define SPEED_HDG_FAST_DEG               5.0f
#define SPEED_HDG_SLOW_DEG              14.0f
#define SPEED_HDG_MIN_PCT               40.0f
#define SPEED_YAW_FAST_DPS              12.0f
#define SPEED_YAW_SLOW_DPS              45.0f
#define SPEED_YAW_MIN_PCT               38.0f
#define SPEED_SETTLE_MS                650U
#define SPEED_SETTLE_PCT                46.0f

/* ---- Corridor classes (drive_math.h). [REMEASURE] */
#define COURSE_CAR_WIDTH_CM             16.0f
#define COURSE_NARROW_MAX_CM            47.0f
#define COURSE_WIDE_MIN_CM              58.0f

/* ---- Sensor-task and diagnostics (consumed by freertos.c). Unchanged. */
#define MEAS_WAIT_MS                     6U
#define ULTRA_MAX_CM                    80U
#define FRONT_MED_WIN                    3U
#define SIDE_MED_WIN                     3U
#define FRONT_FAIL_LIMIT                 5U
#define SIDE_FAIL_LIMIT                  3U
#define IMU_FAIL_LIMIT                   5U
#define IMU_RETRY_MS                   500U
#define CALIB_POLL_MS                  500U
#define MOTOR_TEST                       0

/* ---- Invariants (break these and it fails silently). */
#if !(FRONT_DANGER_CM < FRONT_DECIDE_CM && FRONT_DECIDE_CM < FRONT_STOP_CM && \
      FRONT_STOP_CM < FRONT_TURN_CM && FRONT_TURN_CM < FRONT_CLEAR_CM && \
      FRONT_CLEAR_CM < FRONT_ARC_CM)
#error "Front distance ladder must remain strictly ordered"
#endif

#if BRAKE_CREEP_PCT < MOTOR_MIN_PCT
#error "Brake creep duty must stay above the motor stall floor"
#endif

#if ARC_INNER < MOTOR_MIN_PCT || ARC_APPROACH_INNER < MOTOR_MIN_PCT
#error "Arc inner duties must stay above the motor stall floor"
#endif

#if TURN_INNER < MOTOR_MIN_PCT || \
    TURN_PID_MIN_PCT < MOTOR_MIN_PCT || TURN_PID_FINE_MIN_PCT < MOTOR_MIN_PCT
#error "Pivot duties must stay above the skid breakaway floor"
#endif

#if RESCUE_OUTER < MOTOR_MIN_PCT || RESCUE_INNER < MOTOR_MIN_PCT || \
    REV_TURN_OUTER < MOTOR_MIN_PCT || REV_TURN_INNER < MOTOR_MIN_PCT || \
    SIDE_AVOID_ESCAPE_PCT < MOTOR_MIN_PCT
#error "Rescue/backing duties must stay above the motor stall floor"
#endif

#if FRONT_MED_WIN > 16U || SIDE_MED_WIN > 16U
#error "median_n supports at most 16 samples"
#endif

#endif /* DRIVE_CONFIG_H */
