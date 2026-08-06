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
#define DRIVE_WHEEL_SLEW_PCT_PER_S    800.0f  /* §5.37: 600→800 — 조향/피벗 체결 즉각화. 회귀(기동 브라운아웃·IMU 드랍 증가) 시 600 복귀 */

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
/* §5.39 rolling corner (user spec: goals.mp4 — steer while moving, real-car
 * feel). When the stop-line confirm lands with a CLEAR L/R direction and a
 * live IMU, the car skips the brake+creep stop and rolls straight into the
 * turn. Ambiguous direction, dead IMU or any recovery re-entry still uses
 * the stop-and-decide path — §5.24 front-primary arming is unchanged (same
 * confirmed stop line, direction from L/R there).
 * §5.40 inner semantics (measured: driver duty 0 = both IN pins low = COAST):
 * a freewheeling inner gives almost no yaw moment and the car plows straight
 * (first-track ram). The inner must carry REVERSE torque BELOW the breakaway
 * floor: the wheel stalls into a drag brake, the rotation centre lands on
 * the inner track and the car turns hard while still rolling. -1..-29 = drag
 * regime (more negative = stronger drag); <= -30 spins the pair backwards =
 * full pivot, no longer rolling. Positive values are forbidden. */
#define TURN_ROLL_OUTER                 64   /* §5.50: 70→64 — 곡선 롤 관성/외측 드리프트 축소 (안정성 상향, 실차 곡선 접촉 지속) */
/* §5.41: -24→-20. §5.47: -20→-14 — 롤 반경 확대. 드래그를 약하게 하면 회전중심이
 * 안쪽 바퀴에서 차체 중앙 쪽으로 이동 → 회전반경↑ = 안쪽 벽에서 더 떨어져 돎(벽 긁힘
 * 완화), 리어 스윙 반경↓, 각속도↓로 관성 오버런(22s 과회전)도 완화. 회전력은 스톨
 * 드래그로 유지(가드 (-30,0]). 덜 돌아 코너 못 빠지면 -16, 여전히 붙으면 -12. */
#define TURN_ROLL_INNER                -14
#define TURN_ROLL_GAP_CM                 6
/* §5.47 (mapp_v6 6s 오조향): a turn direction must not flip on a single-frame
 * VL53L0X L<->R swap. cruise_run stabilizes the open-flank sign over this many
 * consecutive frames; the roll commit uses that, not the raw instant. */
#define TURN_DIR_STABLE_N                3U
/* IMG_3188: the wide curve is made of consecutive facets. After one rolling
 * sweep, the next front facet can make the OUTSIDE flank look more open and
 * falsely request the opposite turn. Keep the proven rolling direction only
 * across that short inter-facet cruise; a normal straight expires the latch. */
#define TURN_ROLL_CONTINUE_MS          1100U
/* Closed-loop radius trim during a roll. The outer command remains 70 (no
 * speed reduction); only the sub-stall inner drag moves inside [-24, 0].
 * Near the inside wall it approaches coast to widen the radius, while a near
 * outside wall adds drag to tighten it. Deadband + slew reject ToF facet
 * chatter instead of making the chassis wag through the curve. */
/* §5.53 (여전한 진입 긁힘, IMG_3190 대비 완화만): the closed-loop trim only
 * engaged below TARGET-DEADBAND(13cm) and corrected at 160%/s (24-unit swing
 * = 150ms) — at entry speed/heading momentum that window closes before the
 * radius widens enough. Widen the target so avoidance starts with more
 * buffer, and react harder/faster once it does. */
#define TURN_ROLL_WALL_TARGET_CM        18.0f  /* §5.53: 15→18 — 여유 확보, 더 일찍 반응 시작 */
#define TURN_ROLL_WALL_DEADBAND_CM       2.0f
#define TURN_ROLL_WALL_KP                2.6f  /* §5.53: 2.0→2.6 — cm당 반경 보정력 상향 */
#define TURN_ROLL_INNER_MIN            -24
#define TURN_ROLL_INNER_MAX              0
#define TURN_ROLL_INNER_SLEW_PCT_PER_S 220.0f  /* §5.53: 160→220 — 보정 도달시간 150→109ms */
/* §5.42 (IMG_3149/mappf): a rolling turn TRANSLATES while it sweeps, so it
 * needs headroom to spend — committing with the wall already close plowed the
 * nose to 8 cm / 2 cm (fast approach: F collapsed 42->8 within the confirm
 * window). Rolling entry requires at least this much front room at commit;
 * anything closer stops and pivots in place (§5.24 path). Mid-roll the same
 * budget is watched: front inside the decide line or a flank at the hard
 * band demotes the roll to an in-place pivot (profile swap only — never a
 * backout, §5.25).
 * §5.43(정지선 무브레이크 통과 후 20cm 커밋)은 실차에서 악화 — 롤백됨. 재시도 금지:
 * 30→20cm 무브레이크 접근은 확정 지연/관성과 겹쳐 커밋 창을 자주 건너뛴다. */
#define TURN_ROLL_MIN_F_CM              24   /* §5.47: 20→24 — 벽에서 더 멀리서 롤 시작, 완만 진입 (거리 안정성) */
/* §5.52 (IMG_3190: 벽 직전까지 직진 후 턴 → 진입 아크가 늦음): roll-eligible
 * 코너는 확정 창을 정지선(30) 위에서 열어 미리 아크 진입. 비적격 전방은
 * 이 창에선 계속 주행(거버너 감속)하고 30cm 정지선에서만 브레이크 — §5.27
 * 경계선 멈칫 재발 없음. §5.43(정지선 지나 지연 커밋)과 반대 방향 = 허용.
 * 확정도 fresh+stable ×3 + 피치 게이트라 §5.11류 원거리 오발은 차단.
 * §5.55 (사용자 명세: "더 멀리서 반응"): 36→40 — §5.54로 커브 접근 속도가
 * 이미 올라간 상태(FRONT_MIN_ROLL 54%)라, 예전 36cm 창은 confirm(×3, 접근
 * 중 프레임 소모)이 진행되는 동안 F가 MIN_F_CM(24) 바닥까지 빠르게 줄어
 * 커밋이 24~28cm 근처로 밀렸다 — 그게 "아슬아슬한 진입"의 실제 원인. 창을
 * 40cm로 넓혀 confirm이 더 먼 거리(대략 36~40cm)에서 끝나도록 여유 확보.
 * FRONT_TURN_CM(44) 미만 가드 유지 — 44에 너무 붙지 않도록 40에서 멈춤. */
#define TURN_ROLL_COMMIT_CM             40
/* §5.49 (user spec: "코너링에서도 딱 중앙쪽으로 안전하게"): mid-roll the flank
 * demotion used the repel-band SIDE_HARD_CM(7) — by the time a rolling arc is
 * 7 cm off a wall it is already scraping. Own threshold, set wider, so the
 * roll degrades to an in-place pivot (centre rotation, small radius) while
 * there is still clearance. Must stay under the steering gate so normal
 * centering gets to act first. */
#define TURN_ROLL_SIDE_MIN_CM           10.0f  /* §5.50: 9→10 — 롤 중 피벗 강등 더 일찍 (조향 게이트 13 미만 유지) */
/* §5.45 (IMG_3185/mapp_v5 리어 긁힘 + 일관성): a rolling turn's rotation centre
 * sits on the inner wheel (eccentric), so the REAR outer corner sweeps a
 * ~31 cm radius — fine in the wide plaza curve (open flank 46-50 cm) but the
 * rear scrapes the wall in a tight 37-cm right-angle corridor (open flank
 * ~36 cm). A pivot's centre is the chassis middle (~16 cm max radius), safe
 * in the corridor. So roll ONLY when the corner's open side proves a wide
 * curve; every tighter corner pivots in place — which also removes the
 * roll/pivot toss-up that made cornering inconsistent (user: "일관성 없음").
 * Measured split: plaza 46-50 vs corridor 36 -> 42 cm boundary. */
#define TURN_ROLL_OPEN_SIDE_CM          42
/* §5.57 (user spec: "롤링턴도 특정 각도까지 가면 그만 턴하게" — confirmed sound
 * and applied): a roll's exit used the SAME axis/wide-open heuristics as a
 * right-angle pivot (axis_aligned, wide_exit_course_ok, front-clear-strong-
 * open) — those assume a clean 90 deg corridor to align to. A continuously
 * curving corridor doesn't present that cleanly, so roll exit timing rode on
 * whichever heuristic happened to fire, sometimes late (over-rotate into the
 * outer wall) — IMG_3203's repeatable last-curve hit. Roll now gets its own
 * fixed target and a tighter over-rotation slack, and skips the heuristic
 * branch entirely (deterministic: run to this angle, stop). Equal to the
 * hand-tuned pivot TARGET_DEG for now — tune independently from here. */
#define TURN_ROLL_TARGET_DEG            65.0f
#define TURN_ROLL_OVER_SLACK_DEG        12.0f

#define TURN_TARGET_DEG                 65.0f
/* §5.56 (user spec + traced bug, "마지막 방지턱 전 커브 항상 과회전"): that corner
 * is the geometrically-symmetric tie (course-axis fallback, decide_turn_
 * direction — sides read equal so there is nothing to aim a full pivot at).
 * §5.54's roll-speed relaxation used a looser sign threshold (SIDE_HYST_CM,
 * side_bias) than the actual roll commit gap (TURN_ROLL_GAP_CM) — sensor
 * noise at this near-symmetric wide corridor crossed the loose threshold
 * often enough to grant roll-level cruise speed while the corner almost
 * always still fell through to the tied-pivot path (gap never reached the
 * stricter roll threshold), so the pivot kept entering faster than tuned and
 * overshooting past TARGET_DEG (fixed in centering_run's roll_likely, now
 * gated on the SAME gap as the commit). On top of that fix, user's own
 * diagnosis is applied structurally: a tie-broken pivot has no measured
 * angle to aim for, so it turns decisively to this SHORTER target only, then
 * cruise's course_axis_snap heading-hold finishes the alignment straight —
 * "턴을 한번 하고 가면서 좌우를 맞추는" exactly as requested. Confidently
 * decided pivots (clear L/R winner) keep the full TARGET_DEG unchanged. */
/* §5.58 (IMG_3206, 간헐 역주행): course_axis_snap()은 heading을 가장 가까운
 * 90° 배수로 반올림한다 — TARGET_TIE_DEG를 45°로 뒀던 건 정확히 0°/90° 반올림
 * 경계(중간점)였다. 실제 exit heading은 IMU 필터 지연·스킵 관성 편차로 44~47°
 * 사이 어디든 뜰 수 있고, 44° vs 46°는 반올림이 구축(0°, 원래축) vs 신축(90°,
 * 새 축)으로 갈린다 — 코스 프레임이 실제 물리 회전과 반대로 기록되는 순간이
 * "가끔" 나오는 원인. 반올림 경계에서 확실히 벗어난 값으로 상향(58°: 44~47
 * 흔들림이 전부 90° 쪽으로 반올림되도록 여유 확보). 65°(일반 피벗) 보다는
 * 여전히 짧아 "턴 한번+좌우는 가면서"(§5.56) 취지는 유지. */
#define TURN_TARGET_TIE_DEG             58.0f
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
#define BRAKE_MS                        60U   /* §5.38: 80→60 — 턴 arming 지연 단축 (진입속도는 §5.35 바닥 34%라 여유) */
#define BRAKE_CREEP_PCT                 40    /* §5.32: 32→36, §5.38: 36→40 — 정지선→판정선(16cm) 크립 단축 */
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
 * pair centering only engages once a flank drops inside the gate; above that
 * the car holds the course axis and keeps running. Hysteresis stops the gate
 * chattering at the threshold.
 * §5.46 (user spec 전환): "최대한 좌우를 잡는 방향" — the free-running band is
 * dropped. pair_valid means (L+R) <= SIDE_PAIR_MAX(52), so min(L,R) <= 26 by
 * construction; setting the gate to 26 makes centering active for EVERY valid
 * pair (both walls seen) — the car always trims toward centre instead of
 * drifting to a flank. Wide single-wall zones still free-run (no pair, the
 * §5.19 curve logic owns them). The §5.8 weave stays absent: heading LPF
 * (0.25) + 0.6° deadband + 3 cm |L-R| deadzone still absorb the sensor
 * jitter the user accepts ("가운데 둬도 좌우 떨림 있는 건 확인"). */
#define CENTER_ACT_SIDE_CM              26.0f  /* §5.46: 13→26 — pair면 상시 센터링 (자유주행 밴드 제거) */
#define CENTER_ACT_HYST_CM               3.0f  /* §5.46: 2→3 — 해제 29cm, pair 소멸점과 정합 */
/* §5.37: the gate must LEAD, not lag — at speed a curve facet closes a flank
 * tens of cm/s, and waiting for the raw 14 cm crossing leaves no reaction
 * time (curve-entry rams). The gate therefore engages on the PREDICTED
 * distance (flank + closing_rate * lookahead); a parallel straight closes at
 * ~0 cm/s so prediction changes nothing there. Release stays on the actual
 * distance with hysteresis. */
#define CENTER_ACT_LOOKAHEAD_S           0.35f
#define CENTER_ACT_RATE_LPF_ALPHA        0.30f
#define CENTER_ACT_RATE_MAX_CMS         60.0f

/* §5.49 (user spec): "데드존보다는 계속 좌우 값 비슷하게 밸런싱" — the 3 cm
 * dead zone was a §5.8-era policy (ToF not trusted to 0.5 cm), but it lets the
 * car settle anywhere inside a 3 cm offset band and drift onto a flank before
 * anything acts. Narrowed to 1.5 cm so the pair trim runs continuously; the
 * sensor jitter it used to mask is already handled upstream by the side LPF
 * (0.45) and the 0.6° heading deadband, so this does NOT reopen §5.8 weaving
 * (that failure was a HEADING deadband swallowing the command, not this). */
#define CENTER_DEADZONE_CM               1.5f
#define CENTER_LATERAL_KP_DEG_PER_CM     0.72f  /* IMG_3188: 0.52→0.64, §5.58: →0.72 — 데드존(1.5cm) 밖 좌우 밸런싱 강화. 데드존 안(사용자: 안 치우쳤으면 직진 유지)은 불변 */
#define CENTER_LATERAL_KD_DEG_PER_CMS    0.045f
#define CENTER_LATERAL_KNEE_CM          10.0f
#define CENTER_LATERAL_KNEE_GAIN         1.0f
/* §5.19: the old 12 deg graze/entry band died with the 45-deg mount; the
 * bound is now the recovery authority vs SPEED_HDG_SLOW_DEG budget (the
 * governor is already at its floor before the lateral command saturates). */
#define CENTER_LATERAL_CMD_MAX_DEG      17.0f  /* §5.28: 15→17 */
#define CENTER_LATERAL_CMD_SLEW_DPS    200.0f  /* §5.37: 110→200 — 즉각 반응 (풀권한 도달 155→85ms) */

#define CENTER_HDG_KP_PCT_PER_DEG        0.80f
#define CENTER_HDG_KNEE_DEG             10.0f
#define CENTER_HDG_KP2_PCT_PER_DEG       0.85f
#define CENTER_HDG_DEADBAND_DEG          0.6f
#define CENTER_YAW_KD_PCT_PER_DPS        0.34f
#define CENTER_YAW_RATE_DEADBAND_DPS     8.0f
#define CENTER_YAW_DAMP_MAX_PCT          9.0f

#define CENTER_SIDE_REPEL_KP             2.3f   /* §5.28: 1.9→2.3 — 커브 안쪽벽 조기 밀어내기 */
#define CENTER_SINGLE_TARGET_CM         15.0f  /* §5.45: 13→15 — 곡선존 단일벽 추종 클리어런스 (7초 바깥벽 밀착). 좁은 복도는 pair라 무영향 */
#define CENTER_SINGLE_KP                 0.40f  /* §5.44: 0.22→0.40 — 단일벽 peel-off 권한 (긁기 방어, §5.42 예고분) */
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
#define CENTER_STEER_SLEW_PCT_PER_S    480.0f  /* §5.37: 240→480 — 풀조향 도달 ~110→54ms; 요 댐퍼가 오버슛 흡수 */

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
#define SPEED_TOP_PCT                   90.0f  /* §5.51: 85→88, §5.54: →90 — 커브 진입 완화(FRONT_MIN_ROLL)로 확보한 여유만큼 소폭 추가 상향 */
#define SPEED_MIN_PCT                   34.0f
/* §5.35 (user spec): 전방 ~30cm대 진입 시 확실히 한 김 죽인다 — 바닥 도달을
 * 30→34cm로 당기고 바닥을 38→34%로. 85% 관성 때문에 30cm 시점 실속도가 높아
 * 정지선을 뚫던 것 보정. 긴 직선(>60cm)은 영향 미미. */
#define SPEED_FRONT_FAST_CM             78.0f
#define SPEED_FRONT_SLOW_CM             36.0f  /* §5.51: 34→36 — TOP +3% 관성 보상, 감속 바닥 2cm 선행 */
#define SPEED_FRONT_MIN_PCT             34.0f
/* §5.54 (IMG_3198: "부드러운 롤링턴인데 멈췄다 가는 느낌"): frame-diff motion
 * check found no literal v=0 stop — the front-distance governor bleeds speed
 * down to the 34% floor by the COMMIT line (both sit at ~36cm) regardless of
 * what kind of corner is coming, then the roll's fixed 64% outer duty snaps
 * back up. That crawl-then-surge at EVERY corner reads as stop-and-go. A
 * corner that will actually roll (open flank + stable side_bias already
 * proven while still cruising, same predicate as the roll commit) does not
 * need the deep floor meant for a hard brake-and-pivot stop — it carries
 * through into the arc. Only the front-distance cap gets the relaxed floor;
 * side/heading/yaw caps and the narrow-corner (pivot) path are untouched, so
 * the stop-line braking margin for non-roll corners is unaffected. */
#define SPEED_FRONT_MIN_PCT_ROLL        54.0f
#define SPEED_SIDE_MIN_PCT              34.0f
/* §5.44 (IMG_3182/mappf_v4 긁기): the flank speed ramp used to start only at
 * SIDE_SOFT_CM(10) — §5.36 free-running raised typical flank-approach speeds,
 * and 85% inertia punches straight through a 3 cm ramp into the 7-10 cm band
 * (sustained scraping, 24s: R61mm at speed). The governor band now starts at
 * the steering gate line (13 cm): the moment the gate engages, speed bleeds
 * with it, so the repel authority wins before contact. Thresholds themselves
 * stay untouched (track-geometry rule: slow down, don't widen). */
#define SPEED_SIDE_SLOW_CM              13.0f
/* Single-wall regime (§5.22): one reference wall = no drift warning from the
 * far flank; speed follows the wall distance so a slow convergence never
 * outruns the repel band (SIDE_SOFT..SLOW ramp). */
#define SPEED_SINGLE_SLOW_CM            30.0f  /* §5.50: 25→30 — 곡선(단일벽) 감속을 더 일찍 시작 (거리 문턱이 아니라 속도 예산, §트랙기하 원칙 부합) */
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
#define SPEED_SETTLE_MS                150U    /* §5.32: 250→200, §5.38: →150 — 턴 직후 직진 체결 단축 */
#define SPEED_SETTLE_PCT                76.0f   /* §5.32: 62→66, §5.38: →72, §5.51: →76 (launch 듀티 겸용) */

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

#if TURN_ROLL_INNER > 0 || TURN_ROLL_INNER <= -(MOTOR_MIN_PCT)
#error "Rolling-turn inner must sit in the drag-brake regime: 0 .. -(MOTOR_MIN_PCT-1)"
#endif

#if TURN_ROLL_INNER_MIN <= -(MOTOR_MIN_PCT) || TURN_ROLL_INNER_MAX > 0 || \
    TURN_ROLL_INNER < TURN_ROLL_INNER_MIN || TURN_ROLL_INNER > TURN_ROLL_INNER_MAX
#error "Rolling radius trim must remain in the coast/sub-stall drag regime"
#endif

#if !(FRONT_STOP_CM < TURN_ROLL_COMMIT_CM && TURN_ROLL_COMMIT_CM < FRONT_TURN_CM)
#error "Roll commit window must sit between the stop line and the far-echo turn line"
#endif

#if !(FRONT_DECIDE_CM < TURN_ROLL_MIN_F_CM && TURN_ROLL_MIN_F_CM < FRONT_STOP_CM)
#error "Roll headroom floor must sit between the pivot decide line and the stop line"
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
