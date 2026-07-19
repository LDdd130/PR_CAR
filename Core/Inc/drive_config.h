#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

/* ============================================================================
 * PR_CAR drive core v2 configuration.
 *
 * Rewritten together with drive.c (§5.17): three logical layers — CRUISE
 * (centering + speed governor), TURN (§5.24: pivot only, armed exclusively
 * by the brake-and-decide path at the front decide line) and
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
#define TURN_SPEED                      64    /* §5.34: 68→64 — 풀레이트 관성 오버슈트 미세 완화 (TARGET 65 사용자 튜닝과 짝) */
#define TURN_INNER                      36
#define MOTOR_MIN_PCT                   30
#define MOTOR_TRIM_PCT                   0
#define DRIVE_NOMINAL_UPDATE_MS         20U
#define DRIVE_WHEEL_SLEW_PCT_PER_S    600.0f

/* Rescue pivot (side escape) and three-point-turn backing. */
#define RESCUE_OUTER                    48
#define RESCUE_INNER                    32
#define REV_TURN_OUTER                  46
#define REV_TURN_INNER                  32

/* ---- Front distance ladder (cm, HC-SR04 — hardware unchanged). ----------
 * DANGER < DECIDE < STOP < TURN < CLEAR, guarded below.
 * DECIDE 17 is a user-mandated spec (§5.9 20cm -> §5.24 16~17cm): a pivot
 * direction is only decided from a wall firmly confirmed at the decide line
 * by consecutive fresh AND stable samples. Turns start nowhere else. */
#define FRONT_DANGER_CM                 12
#define FRONT_DECIDE_CM                 16
#define FRONT_STOP_CM                   30    /* §5.27: 34→30 — 코너 접근 36cm 호버링이 만드는 경계선 브레이크(멈칫) 컷 */
#define FRONT_TURN_CM                   44
#define FRONT_CLEAR_CM                  52

/* Fresh-confirm counts (frames). Front reading spikes (floor echo, bounce):
 * no single sample may brake, open a corner or decide a direction (§5.9). */
#define FRONT_STOP_CONFIRM_N             3U
#define FRONT_DECIDE_CONFIRM_N           2U
/* §5.23: confirm samples must also AGREE — a head-on wall closes a few cm
 * per frame, floor/bump echoes teleport (mapp_v2: 3->30->16). Consecutive
 * samples differing more than this restart the confirm chain. The danger
 * ladder (FRONT_DANGER_CM) stays ungated: safety beats phantom rejection. */
#define FRONT_STABLE_CM                  4U
#define CLEAR_CONFIRM_N                  3U

/* ---- Side geometry (cm, perpendicular ToF). [REMEASURE] ------------------
 * Centered lateral clearance by corridor: 10.5 (37 cm) .. 25.5 (67 cm).
 * A corner opening reads the next corridor's depth, well past any centered
 * clearance; an off-center run in the widest zone can still fake large far
 * readings, which is why corner detection additionally REQUIRES the
 * confirmed approaching front wall above. */
#define SIDE_PAIR_MAX_CM                52.0f
#define SIDE_HYST_CM                     5

/* Wall proximity bands. [REMEASURE] */
#define SIDE_SOFT_CM                    10.0f
#define SIDE_HARD_CM                     7.0f
#define SIDE_AVOID_CM                    5
/* §5.26: at high duty the flank must fire earlier — an oblique wall gives
 * the front ultrasonic no echo, so the side ToF is the only warning. */
#define SIDE_FAST_DUTY_PCT              52
#define SIDE_AVOID_FAST_CM               9

/* §5.26 cruise ram/stall backstop (left encoder only; right reads 0 —
 * hardware). Duty high + wheel collapsed for the window = nose in a wall
 * the ultrasonic cannot see (specular) — back off and decide. Grace skips
 * spin-up after launch/state entry. */
#define CRUISE_STALL_DUTY_PCT           42
#define CRUISE_STALL_SPEED_CMS           6.0f
#define CRUISE_STALL_MS                350U
#define CRUISE_STALL_GRACE_MS          400U
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
/* §5.30: 풀레이트 피벗(테이퍼 없음)의 스키드 관성 리드 — 컷 후 관성으로 수 °
 * 더 돈다. §5.31 실차: 85 컷도 과회전 체감 → 80으로. 잔여 ~10°는 크루즈
 * 축유지가 흡수. 덜 돌면 85로 복귀, 더 줄일 땐 TURN_MIN(30)과 간격 유지. */
#define TURN_TARGET_DEG                 65.0f
#define TURN_MIN_DEG                    30.0f
#define TURN_WRONG_DEG                  20.0f
#define TURN_AXIS_ALIGN_DEG             10.0f
/* Wide/curve zone (§5.19): with both sides at least this open the track is
 * not a 90 deg corridor (octagon plaza facets sit at 45 deg), so a turn may
 * exit off-axis as soon as the nose opens — the axis-align gate only binds
 * where a corridor exists to align to. */
#define TURN_WIDE_EXIT_SIDE_CM          30
/* §5.22 course leash on off-axis exits (mapp_v1 wrong-way run): tighter than
 * the 115 deg reversal gate — a 154 deg overshoot passed 115 and cruised
 * backwards. §5.33: the §5.22 wide-only over-rotation budget is replaced by
 * the every-zone sweep cap below (uniform turns, user spec). */
#define WIDE_EXIT_COURSE_DEV_DEG        60.0f
/* §5.33: 한 sweep이 TARGET+slack을 넘도록 exit이 안 열리면(복구 연쇄로 코스
 * 프레임이 돌아간 구간) 더 돌지 말고 후진 재접근 — 후반 구간 "더 꺾고 박기"의
 * 상한. 진짜 반전 포켓은 retry 예산(3회 → 직후진+재판정)으로 풀린다. */
#define TURN_OVER_SLACK_DEG             22.0f
/* §5.27: 강개방(F>=CLEAR) 조기 exit의 축근접 허용오차 — 코너 포켓의 대각선
 * 개구도 "열림"으로 읽히므로, 축에서 이 이상 벗어난 강개방 exit은 턴 미완성. */
#define TURN_EXIT_ALIGN_LOOSE_DEG       25.0f
#define COURSE_REV_DEG                 115.0f
#define TURN_LEG_REVERSAL_DEG          180.0f
#define SPIN_COMMIT_MS                 100U
#define SPIN_MAX_MS                   1600U
#define SPIN_BLIND_MS                  650U
#define ROT_STUCK_MS                   400U
#define ROT_STUCK_DEG                    8.0f

/* ---- Recover. -------------------------------------------------------------
 * A blocked pivot backs out with a three-point-turn chunk (yaw preserved)
 * and resumes; straight backing is reserved for brake-line walls and dead
 * ends. The retry budget is per turn episode; when spent, backing degrades
 * to straight + a fresh direction decision instead of deadlocking (§5.15). */
#define BRAKE_MS                        80U
#define BRAKE_CREEP_PCT                 36    /* §5.32: 32→36 — 정지선→판정선(16cm) 크립 단축 */
#define BRAKE_CREEP_MAX_MS             900U   /* §5.24: decide 20->17cm, 크립 예산 +200ms */
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

/* §5.36 (user spec): straight-line lateral correction is proximity-gated —
 * pair centering only engages once a flank drops inside ~14 cm (135-140 mm);
 * above that the car just holds the course axis and keeps running. Hysteresis
 * stops the gate chattering at the threshold. The narrow corridor (37 cm,
 * 10.5 cm centered clearance) sits inside the gate permanently, so its
 * behaviour is unchanged; only wider zones gain the free-running band. */
#define CENTER_ACT_SIDE_CM              14.0f
#define CENTER_ACT_HYST_CM               2.0f

#define CENTER_DEADZONE_CM               3.0f
#define CENTER_LATERAL_KP_DEG_PER_CM     0.52f  /* §5.28: 0.45→0.52 — 고속 오프센터 평형(좌 10cm 고착) 해소 */
#define CENTER_LATERAL_KD_DEG_PER_CMS    0.045f
#define CENTER_LATERAL_KNEE_CM          10.0f
#define CENTER_LATERAL_KNEE_GAIN         1.0f
/* §5.19: the old 12 deg graze/entry band died with the 45-deg mount; the
 * bound is now the recovery authority vs SPEED_HDG_SLOW_DEG budget (the
 * governor is already at its floor before the lateral command saturates). */
#define CENTER_LATERAL_CMD_MAX_DEG      17.0f  /* §5.28: 15→17 */
#define CENTER_LATERAL_CMD_SLEW_DPS    110.0f

#define CENTER_HDG_KP_PCT_PER_DEG        0.80f
#define CENTER_HDG_KNEE_DEG             10.0f
#define CENTER_HDG_KP2_PCT_PER_DEG       0.85f
#define CENTER_HDG_DEADBAND_DEG          0.6f
#define CENTER_YAW_KD_PCT_PER_DPS        0.34f
#define CENTER_YAW_RATE_DEADBAND_DPS     8.0f
#define CENTER_YAW_DAMP_MAX_PCT          9.0f

#define CENTER_SIDE_REPEL_KP             2.3f   /* §5.28: 1.9→2.3 — 커브 안쪽벽 조기 밀어내기 */
#define CENTER_SINGLE_TARGET_CM         14.0f  /* §5.36: 13→14 — 단일벽 repel도 140mm 게이트에 정렬 */
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
/* §5.30 (user spec): 전방 거리만이 주 감속 게이트 — 전방이 FAST(78cm) 안으로
 * 들어오기 전엔 최고속으로 달린다. 감속 램프(78→30cm, TOP%→38%)가 제동 예산의
 * 전부이므로 FRONT_FAIL/timeout-open 구간의 과속은 §5.26 스톨 백스톱과
 * DANGER(12) 사다리가 받친다.
 * §5.32 랩타임 공략(21s→19s 목표): TOP 78→85. 램프 양끝(78cm 시작, 정지선
 * 도달속도)은 검증값 유지 — 중간 구간만 가팔라짐. 회귀 롤백 순서:
 * TOP 85→78→69 → FRONT_MIN 38→36. */
#define SPEED_TOP_PCT                   85.0f
#define SPEED_MIN_PCT                   34.0f
/* §5.35 (user spec): 전방 ~30cm대 진입 시 확실히 한 김 죽인다 — 바닥 도달을
 * 30→34cm로 당기고 바닥을 38→34%로. 85% 관성 때문에 30cm 시점 실속도가 높아
 * 정지선을 뚫던 것 보정. 긴 직선(>60cm)은 영향 미미. */
#define SPEED_FRONT_FAST_CM             78.0f
#define SPEED_FRONT_SLOW_CM             34.0f
#define SPEED_FRONT_MIN_PCT             34.0f
#define SPEED_SIDE_MIN_PCT              34.0f
/* Single-wall regime (§5.22): one reference wall = no drift warning from the
 * far flank; speed follows the wall distance so a slow convergence never
 * outruns the repel band (SIDE_SOFT..SLOW ramp). */
#define SPEED_SINGLE_SLOW_CM            25.0f
#define SPEED_SINGLE_MIN_PCT            44.0f
/* §5.27 직진 멈칫 완화: 주행 중 좌우 보정은 "가면서" 잡는다 — 보정 중 감속
 * 램프가 너무 일찍 물리면 매 보정마다 출렁임(멈칫). §5.19 조향 권한 상향이
 * 보정 자체를 빠르게 끝내므로 감속 개입은 더 큰 오차/요레이트로 미룬다. */
#define SPEED_HDG_FAST_DEG               6.0f
#define SPEED_HDG_SLOW_DEG              18.0f
#define SPEED_HDG_MIN_PCT               48.0f   /* §5.32 */
#define SPEED_YAW_FAST_DPS              15.0f
#define SPEED_YAW_SLOW_DPS              60.0f
#define SPEED_YAW_MIN_PCT               48.0f   /* §5.32 */
#define SPEED_SETTLE_MS                200U    /* §5.32 펀치아웃 4차: 250→200 */
#define SPEED_SETTLE_PCT                66.0f   /* §5.32: 62→66 */

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
      FRONT_STOP_CM < FRONT_TURN_CM && FRONT_TURN_CM < FRONT_CLEAR_CM)
#error "Front distance ladder must remain strictly ordered"
#endif

#if BRAKE_CREEP_PCT < MOTOR_MIN_PCT
#error "Brake creep duty must stay above the motor stall floor"
#endif

#if TURN_INNER < MOTOR_MIN_PCT
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
