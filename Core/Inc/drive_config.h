#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

/* Motor commands (% duty).
 * Pivoting is skid steering: all four wheels scrub sideways, so the breakaway
 * duty is far above the straight-line stall floor. TURN_INNER is the inner
 * pair actively reversing — below MOTOR_MIN_PCT it stalls and the "pivot"
 * degrades into a wide one-sided arc that sweeps the nose into the wall. */
#define DRIVE_SPEED                     32
#define TURN_SPEED                      56
#define TURN_INNER                      34
#define MOTOR_MIN_PCT                   30
#define MOTOR_TRIM_PCT                   0
#define DRIVE_NOMINAL_UPDATE_MS          20U
#define DRIVE_WHEEL_SLEW_PCT_PER_S    600.0f

/* Forward arc. Keep normal-corner wheels above the measured stall floor. */
#define ARC_OUTER                       72
#define ARC_INNER                       30
#define ARC_APPROACH_OUTER              56
#define ARC_APPROACH_INNER              30
#define ARC_APPROACH_DEG                18.0f
#define ARC_MAX_MS                     850U
/* Active rescue pivot. A zero-duty inner side drags on this four-wheel skid
 * chassis, so under-rotation must reverse the inner pair around the car centre. */
#define CORNER_RESCUE_OUTER             48
#define CORNER_RESCUE_INNER             32

/* Front/side geometry (cm). Keep DANGER < DECIDE < STOP < TURN < CLEAR < ARC. */
#define FRONT_DANGER_CM                 12
#define CORNER_ABORT_CM                 16
/* The HC-SR04 front reading spikes (floor echo, dropouts), so a pivot's turn
 * direction is only decided from a wall that is firmly at FRONT_DECIDE_CM:
 * consecutive fresh (non-latched) samples, reached by creeping after braking.
 * A braking event whose wall evaporates instead resumes cruise — a spike must
 * never pick a turn direction. */
#define FRONT_DECIDE_CM                 20
#define FRONT_DECIDE_CONFIRM_N           2U
#define BRAKE_CREEP_PCT                 32
#define BRAKE_CREEP_MAX_MS             700U
#define FRONT_STOP_CM                   34
#define FRONT_TURN_CM                   44
#define FRONT_CLEAR_CM                  52
#define FRONT_ARC_CM                    68
#define SIDE_BLOCK_CM                    8
#define SIDE_HYST                        6
#define SIDE_OPEN_CM                    26
#define CORNER_ASYM_CM                  10
#define CORNER_ASYM_OPEN_CM             28
#define CORNER_NEAR_SAFE_CM              7
#define CORNER_CONFIRM_N                 2
#define CORNER_ENTRY_HDG_MAX_DEG        12.0f
#define CORNER_TIGHTEN_CM               30
#define CORNER_TIGHT_RELEASE_DEG        55.0f
#define CORNER_PROGRESS_GRACE_MS       180U
#define CORNER_MIN_PROGRESS_DPS         75.0f
#define CORNER_PROGRESS_SLACK_DEG        8.0f
#define CORNER_SIDE_ONLY_CONFIRM_N        3U
/* 36: an off-center run in the wide straight reads the far wall at ~33 cm,
 * which must stay below this; the real front-lost corners read 36-50 cm. */
#define CORNER_SIDE_ONLY_OPEN_CM         36U
#define CORNER_SIDE_ONLY_WIDE_OPEN_CM    46U
#define CORNER_SIDE_ONLY_NEAR_CM         20U
#define CORNER_SIDE_ONLY_ASYM_CM         12U
#define CORNER_SIDE_ONLY_WIDE_ASYM_CM    20U
#define CORNER_SYMMETRIC_MAX_DIFF_CM      6U
#define CORNER_SYMMETRIC_CONFIRM_N        3U
#define COURSE_FINAL_ENTRY_DEG           90.0f
#define COURSE_FINAL_ENTRY_TOL_DEG       20.0f

/* Wide corridors need stronger evidence before an opening becomes a corner. */
#define CORNER_WIDE_OPEN_CM             30U
#define CORNER_WIDE_ASYM_OPEN_CM        30U
#define CORNER_WIDE_ASYM_CM             10U
#define CORNER_WIDE_NEAR_CM             15U

/* Emergency side escape. Normal centering should act before this state. */
#define SIDE_AVOID_CM                    6
#define SIDE_AVOID_CLEAR_CM              8
#define SIDE_AVOID_CLEAR_CONFIRM         3
#define SIDE_AVOID_MAX_MS              360U
#define SIDE_ESCAPE_OUTER               40
#define SIDE_ESCAPE_INNER               12

/* HC-SR04 side-wall grazing model. */
#define FRONT_BEAM_HALF_SIN              0.26f
#define FRONT_CONFIRM_N                  3
#define FRONT_OFFAXIS_DEG               25.0f

/* Recovery state timing. All waits are non-blocking. */
#define BRAKE_MS                         80U
#define SPIN_COMMIT_MS                  100U
#define CLEAR_CONFIRM                    3
/* Minimum rotation before a clear front may end a spin. Post-corner wall
 * encounters often need only a 25-40 deg correction; forcing more converts
 * every small misalignment into an overshoot the next state must undo. */
#define TURN_MIN_DEG                    30.0f
#define TURN_TARGET_DEG                 88.0f
#define TURN_CUTOFF_DEG                 96.0f
#define TURN_WRONG_DEG                  20.0f
#define COURSE_REV_DEG                 115.0f
#define SWAP_LIMIT                       1
#define SPIN_MAX_MS                    1600U
#define SPIN_BLIND_MS                   650U
#define LAUNCH_MS                       120U
#define BACK_CHUNK_MS                   220U
#define REV_MAX_CHUNKS                   2
#define ROT_STUCK_MS                    400U
#define ROT_STUCK_DEG                    8.0f

/* Pivot heading controller. Floors sit above the skid-steer breakaway duty,
 * not the straight-line stall floor — a pivot that slows below breakaway
 * near the target simply stops rotating and times out. */
#define TURN_PID_KP                      0.62f
#define TURN_PID_KI                      0.018f
#define TURN_PID_KD                      0.14f
#define TURN_PID_I_MAX                 130.0f
#define TURN_PID_MIN_PCT                44
#define TURN_PID_FINE_MIN_PCT           40
#define TURN_PID_FINE_DEG               14.0f
#define TURN_PID_INNER_RATIO             0.48f

/* 45-degree course-grid exits. */
#define CORNER_GRID_EXIT_CM              60
#define CORNER_GRID_EXIT_MIN_DEG         35.0f
#define CORNER_GRID_EXIT_MAX_DEG         65.0f
#define CORNER_GRID_ALIGN_DEG            10.0f
#define CORNER_EXIT_MIN_DEG              80.0f

/* Corridor centering input conditioning. */
#define CENTER_SENSOR_MAX_CM             80U
#define CENTER_LPF_ALPHA                  0.45f
#define CENTER_DERR_LPF_ALPHA             0.28f
#define CENTER_DERR_MAX_CMS              35.0f
#define CENTER_YAW_LPF_ALPHA              0.22f
#define CENTER_YAW_RATE_MAX_DPS          90.0f
/* Side ToF pair is only trustworthy to about the cm; below this difference
 * the corridor position is taken as "already centered" and the controller
 * holds heading instead of chasing sensor wiggle. */
#define CENTER_DEADZONE_CM                3.0f
#define CENTER_INNER_KP_SCALE             0.0f
#define CENTER_SIDE_PAIR_MAX_CM          58.0f
#define CENTER_SINGLE_MAX_CM             32.0f
#define COURSE_CAR_WIDTH_CM              16.0f
#define COURSE_NARROW_MAX_CM             47.0f
#define COURSE_WIDE_MIN_CM               58.0f

/* Outer loop: lateral error -> desired heading offset (degrees). */
#define CENTER_LATERAL_KP_DEG_PER_CM      0.32f
#define CENTER_LATERAL_KD_DEG_PER_CMS     0.018f
#define CENTER_STRAIGHT_KP_DEG_PER_CM     0.30f
#define CENTER_STRAIGHT_KD_DEG_PER_CMS    0.015f
#define CENTER_NARROW_KP_DEG_PER_CM       0.30f
#define CENTER_WIDE_KP_DEG_PER_CM         0.12f
#define CENTER_LATERAL_CMD_MAX_DEG        8.0f
#define CENTER_LATERAL_CMD_SLEW_DPS      70.0f

/* Inner loop: heading target -> differential wheel duty.
 * Vibration jitters the raw heading a few degrees even on a clean straight.
 * That noise is removed by CENTER_HDG_ERR_LPF_ALPHA, not by the deadband:
 * a deadband wide enough to hide it would leave the corridor position
 * unregulated (or shift the settling point to the band edge) and the car
 * weaves between the wall-repel zones. Keep the deadband small. */
#define CENTER_HDG_ERR_LPF_ALPHA          0.25f
#define CENTER_HDG_KP_PCT_PER_DEG         0.50f
#define CENTER_HDG_DEADBAND_DEG           0.6f
#define CENTER_YAW_KD_PCT_PER_DPS         0.24f
#define CENTER_YAW_RATE_DEADBAND_DPS     10.0f
#define CENTER_YAW_DAMP_MAX_PCT           4.0f
#define CENTER_STEER_MAX_PCT             18.0f
#define CENTER_STEER_SLEW_PCT_PER_S     140.0f

/* Single-wall/open-space fallback. */
#define CENTER_SINGLE_TARGET_CM          16.0f
#define CENTER_SINGLE_KP                  0.22f
#define CENTER_SINGLE_HDG_BLEND           0.42f
#define CENTER_OPEN_HDG_KP_PCT_PER_DEG    0.58f

/* Progressive wall protection. The 37 cm lane has about 10.5 cm centered clearance. */
#define CENTER_SIDE_SOFT_CM              11.0f
#define CENTER_SIDE_HARD_CM               8.0f
#define CENTER_SIDE_REPEL_KP              1.35f
#define CENTER_SIDE_MIN_SPEED_PCT        32.0f
#define CENTER_BODY_HALF_LEN_CM          13.5f
#define CENTER_NEAR_GUARD_CM              9.0f
#define CENTER_NEAR_GUARD_KP              1.2f
#define CENTER_YAW_SWEEP_MAX_DEG         14.0f
#define CENTER_YAW_SWEEP_GAIN             0.85f
#define CENTER_GUARD_FULL_DEPTH_CM        2.0f
#define CENTER_GUARD_MIN_SPEED_PCT       30.0f

/* Heading reference maintenance. */
#define CENTER_HREF_BLEND                 0.006f
#define CENTER_HREF_ALIGN_ERR_CM          3.0f
#define CENTER_HREF_ALIGN_HDG_DEG         8.0f
#define CENTER_HREF_ALIGN_DERR_CMS       18.0f
#define CENTER_HREF_ALIGN_YAW_DPS         8.0f
#define CENTER_AXIS_RESNAP_DEG           25.0f
#define CENTER_AXIS_RESNAP_YAW_DPS       12.0f
#define CENTER_AXIS_RESNAP_N             12U

/* Cruise speed scheduling (% duty). */
#define CENTER_BASE_SPEED_PCT            60.0f
#define CENTER_STRAIGHT_FAST_SPEED_PCT   64.0f
#define CENTER_NARROW_FAST_SPEED_PCT     60.0f
#define CENTER_WIDE_FAST_SPEED_PCT       68.0f
#define CENTER_STRAIGHT_FAST_ERR_CM       8.0f
#define CENTER_STRAIGHT_FAST_HDG_DEG      8.0f
#define CENTER_MIN_SPEED_PCT             36.0f
#define CENTER_FRONT_FAST_CM             72.0f
#define CENTER_FRONT_SLOW_CM             44.0f
#define CENTER_FRONT_MIN_SPEED_PCT       36.0f
#define CENTER_SETTLE_MS                 90U
#define CENTER_SETTLE_SPEED_PCT          56.0f
#define CENTER_MID_SETTLE_SPEED_PCT      60.0f
#define CENTER_WIDE_SETTLE_SPEED_PCT     66.0f
#define CENTER_HDG_FAST_DEG               4.0f
#define CENTER_HDG_SLOW_DEG              14.0f
#define CENTER_HDG_MIN_SPEED_PCT         40.0f

/* Sensor-task and diagnostics configuration. */
#define MEAS_WAIT_MS                      6U
#define ULTRA_MAX_CM                     80U
#define FRONT_MED_WIN                     3U
#define SIDE_MED_WIN                      3U
#define FRONT_FAIL_LIMIT                  5U
#define SIDE_FAIL_LIMIT                   3U
#define IMU_FAIL_LIMIT                    5U
#define IMU_RETRY_MS                    500U
#define CALIB_POLL_MS                   500U
#define MOTOR_TEST                        0

#if !(FRONT_DANGER_CM < FRONT_STOP_CM && FRONT_STOP_CM < FRONT_TURN_CM && \
      FRONT_TURN_CM < FRONT_CLEAR_CM && FRONT_CLEAR_CM < FRONT_ARC_CM)
#error "Front distance thresholds must remain strictly ordered"
#endif

#if !(FRONT_DANGER_CM < FRONT_DECIDE_CM && FRONT_DECIDE_CM < FRONT_STOP_CM)
#error "Turn-decide distance must sit between the danger and stop thresholds"
#endif

#if BRAKE_CREEP_PCT < MOTOR_MIN_PCT
#error "Brake creep duty must stay above the motor stall floor"
#endif

#if ARC_INNER < MOTOR_MIN_PCT || ARC_APPROACH_INNER < MOTOR_MIN_PCT
#error "Normal corner arc commands must stay above the motor stall floor"
#endif

#if TURN_PID_MIN_PCT < MOTOR_MIN_PCT || TURN_PID_FINE_MIN_PCT < MOTOR_MIN_PCT
#error "Pivot PID outer duty must stay above the motor stall floor"
#endif

#if TURN_INNER < MOTOR_MIN_PCT
#error "Pivot inner reverse duty must stay above the motor stall floor"
#endif

#if CORNER_RESCUE_OUTER < MOTOR_MIN_PCT || CORNER_RESCUE_INNER < MOTOR_MIN_PCT
#error "Corner rescue pivot duties must stay above the skid breakaway floor"
#endif

#if !(CORNER_ABORT_CM < CORNER_TIGHTEN_CM && CORNER_TIGHTEN_CM < FRONT_CLEAR_CM)
#error "Corner rescue distance thresholds must remain ordered"
#endif

#if FRONT_MED_WIN > 16U || SIDE_MED_WIN > 16U
#error "median_n supports at most 16 samples"
#endif

#endif /* DRIVE_CONFIG_H */
