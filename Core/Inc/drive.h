/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drive.h
  * @brief   주행 제어기 공개 API + 튜닝 노브 (비블로킹 상태머신 + 복도 센터링 조향)
  *          상태: CRUISE(조향 주행) → CORNER(전진성 아크 코너) / BRAKE(능동 제동) → SPIN(제자리 피벗 폴백)
  *                ↔ REVERSE(막다른곳 후진) / HOLD(정면 센서 상실 안전정지) / SIDE_AVOID(측벽 비상회피)
  *          drive.c는 HAL 타이머/I2C 미접촉 — DriveInputs 스냅샷만 소비, motor.h만 호출.
  *          (센서 측정/필터/IMU 폴링은 main.c 담당)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __DRIVE_H__
#define __DRIVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============ 튜닝 가능한 값 (여기만 바꿔가며 조정) ============ */
/* --- 속도 (0~100%) : 좁은 미로 안정속도로 하향 평준화 (영상형 저속 피벗 주행) --- */
#define DRIVE_SPEED     40   /* 후진(REVERSE) 기본 구동 속도. 30→40: 구동력 상향(바닥 마찰 극복·스톨 방지).
                                ⚠직진 속도는 이 값이 아님 — 실제 직진은 drive.c CENTER_BASE_SPEED_PCT(44) */
#define TURN_SPEED      44   /* [후진성 피벗·폴백] 회전 '바깥' 바퀴 전진 속도. 52→44: 출발 IMU-blind 구간 과회전(역주행) 차단 — ω∝(44+54)=98로 환원(SPIN_BLIND_MS 원캘리브 ≈90°/800ms 복원).
                                v=(44-54)/2=-5 (거의 제자리), ω∝(44+54)=98. 안돌면 둘 다 ↑(대칭 유지) */
#define TURN_INNER      54   /* [후진성 피벗·폴백] 회전 '안쪽' 바퀴 후진 속도. 64→54(비례): TURN_SPEED(44)에 근접 유지 → v=-5(거의 제자리).
                                ω∝(44+54)=98로 동반 감속(과회전 억제). 회전 너무 느리면 둘 다 ↑(대칭 유지) */
/* ── 아크(Arc) 코너 부활: 전진성 곡선 주행으로 90° 코너 통과 (제자리 피벗의 대각 끼임/충돌 회피).
 *    Car_ArcLeft/Right(outer,inner) = 양바퀴 '전진', 안쪽만 감속 → 곡선 반경 R≈(T/2)·(O+I)/(O−I) (T=윤거).
 *    피벗(제자리)은 아크 실패/막다른곳 폴백 전용으로 강등. */
#define ARC_OUTER        55   /* 아크 '바깥' 바퀴 전진 [%duty]. 45→55(고속 비례). ★반경은 O/I '비율'에만 의존 → O,I 동배율 상향 = 반경 불변, 통과만 빨라짐 */
#define ARC_INNER        32   /* 아크 '안쪽' 바퀴 전진 [%duty]. 27→32(영상 코너 inner-wall hug → O/I 2.04→1.72로 반경↑=코너 더 넓게 돔). O−I 클수록 반경↓. 안쪽 스톨이면 ↑, 안쪽벽 여전히 붙으면 ↑(35), 코너 너무 크게 돌면 ↓(29) */
#define ARC_MAX_MS       2000 /* 아크 타임아웃 백스톱 → BRAKE(피벗 폴백). IMU 사망+개활지서 무한 아크 방지 */
#define MOTOR_MIN_PCT   30   /* TT모터 스톨 하한(개념 기준). 모든 구동 속도는 이 값 이상이어야 실제로 굴러감.
                                (현 코드 spillover 미구현 — 속도 노브를 이 값 이상으로 두는 가이드로 사용) */
#define MOTOR_TRIM_PCT  0    /* ★직진 편향 보정 [%duty, CRUISE 직진 전용]: 차가 좌로 쏠리면 +(좌바퀴↑/우바퀴↓), 우로 쏠리면 −. 0=보정없음. 바닥서 직진 쏠림 관찰 후 ±1~3 조정 */

/* --- 거리 임계 (cm) : 핵심 민감도 (관계 유지: DANGER < STOP < TURN < CLEAR < ARC). ★고속화로 관성↑ → 제동선 선제 확대 --- */
#define FRONT_STOP_CM   20   /* ★미만이면 즉시 능동 제동(비상). 15→20: 직진 속도/관성 보상 — 제동거리 d∝v² 증가분 선제 흡수. HOLD/SIDE_AVOID/아크 폴백의 최후 정면 제동선. 너무 일찍 서면 ↓ */
#define FRONT_TURN_CM   30   /* ★정면 이 거리 미만 + 측면 트임 없음 = 제동(브레이크)→피벗 폴백. 22→30: 고속 관성 제동거리 확보(아크 코너는 FRONT_ARC가 먼저 처리, 이건 막다른길/실표적 제동선).
                                박으면 ↑(34), 측벽 끼면 ↓ */
#define FRONT_CLEAR_CM  38   /* 이상이 CLEAR_CONFIRM회 연속이면 회전/아크 종료→직진 복귀. 30→38: 진입(TURN 30)보다 +8cm(히스테리시스) 유지 = 직진복귀 직후 즉시 재회전 방지.
                                짧은 복도서 38까지 안 트이면 회전각(TURN_TARGET~85°/CUTOFF 100°) 탈출이 백업이라 데드락 없음. 미로 넓으면 ↑ */
#define FRONT_ARC_CM    55   /* ★정면 이 거리 미만 + 한쪽 측면 트임(≥SIDE_OPEN) = 전진성 아크 코너 진입. 45→55: 고속서 곡선 시작 공간을 더 일찍 확보(아크는 전진 반경 필요). 아크가 외벽에 붙으면 ↑, 직선서 헛아크면 ↓ */
#define FRONT_DANGER_CM 8    /* 코앞 경계(막다른곳 후진 판정). 12→8: 후진 판정 거리 축소 = 코너서 불필요한 후진 오작동 차단 */
#define SIDE_BLOCK_CM   4    /* 측면 막힘 판정. 6→4: 회전 중 측면 좁아짐을 '막다른길'로 오판해 후진하던 것 차단(더 바짝 붙어야만 막힘).
                                트랙39/스팬19 중앙여유~10cm라 충분히 작아야 중앙주행을 막힘으로 오판 안 함 */
#define SIDE_HYST       5    /* 좌우 트임 차 데드밴드(cm): 이만큼 차이나야 회전 방향 결정/재평가 (좁은 트랙→축소) */
#define SIDE_OPEN_CM    28   /* ★측면 이 거리 이상 = 그쪽이 '트임'(코너 회전로) → 아크 진입 트리거(트인 쪽으로 곡선). 35→28: 완만곡선 바깥벽(~25cm)도 트임 인정해 곡선 코너 진입. front_near(f<ARC) 동반조건이라 직진 오트리거 억제. 만료센티넬(ULTRA_MAX_CM)도 트임 */
#define CORNER_ASYM_CM  12   /* ★[fix③] 좌우 측거리 비대칭 ≥이값(cm)이면 절대 트임 미달도 곡선코너로 인지(더 트인 쪽 아크) + graze 면제. 곡선 못 잡으면 ↓(10), 직진서 헛코너면 ↑(14) */

/* --- 조향 (CRUISE 복도 센터링) ---
 * ⚠ 살아있는 P/D/슬루/데드존/속도 노브는 전부 drive.c의 CENTER_* 블록에 있음.
 *   여기 둘만 drive.c가 직접 참조: heading-hold P게인 + yaw-rate 댐핑 D게인. */
#define KP_HDG          0.8f /* heading-hold P게인 [%duty/deg]. IMU 사망 시 자동 open-loop 직진 */
#define KD_YAW          0.30f/* yaw-rate 댐핑 D게인 [%duty per deg/s]. 좌우 흔들림(리밋사이클) 상쇄 —
                                gyro 회전속도를 음의 피드백으로. 0.26→0.30(영상 직진 weave 잔존 → 댐핑↑. 채터(과댐핑 공진)나면 ↓(0.26), 흔들림 남으면 ↑(0.34)).
                                imu_live일 때만 적용(사망 시 P-only로 자동 강등) */

/* --- 측면 비상 회피 (조향으로 못 막은 측벽 박힘: 멈춤+박힌쪽 반대로 제자리 피벗) --- */
#define SIDE_AVOID_CM       4  /* ★진입: 한쪽 측면 이 거리[cm] 미만 = 박기 직전 → SIDE_AVOID 진입(전진0 피벗).
                                  8→4: 민감도 대폭 하향 = 정상 조향 주행 중 잦은 비상 정지(멈칫) 방지. 박히면 ↑(6) */
#define SIDE_AVOID_CLEAR_CM 6  /* 탈출: 측면 min(l,r)이 이 거리[cm] 이상 회복 시 CRUISE 복귀. 9→6: 진입(4)보다 크게 유지(채터 방지)하되 빨리 복귀 = 멈칫 시간↓ */
#define SIDE_AVOID_MAX_MS   600 /* 한 방향 피벗 후 방향 재평가(flip) 백스톱[ms] — 한쪽으로 못 빠지면 반대로 시도 */

/* --- 전면 오인식 필터 (좁은 복도 빔 그레이징을 실제 정면벽으로부터 수학적으로 구분) ---
 * 문제: HC-SR04 빔이 원뿔(반각 θ_b)이라 좁은 복도서 측벽을 스쳐(graze) 짧은 거리를 반환 → 정면벽 오인식.
 * 핵심 기하: 측방거리 w인 벽이 빔에 잡히는 최단 슬랜트거리 R_min = w / sin(θ_b).
 *           ⇒ 측정 f ≥ w/sin(θ_b) 이면 그 측벽 그레이징으로 설명 가능(가짜 의심),
 *             f < w/sin(θ_b) 이면 측벽으론 못 만드는 더 가까운 실제 표적(진짜 정면벽). */
#define FRONT_BEAM_HALF_SIN 0.26f /* sin(θ_b), θ_b≈15°(HC-SR04 유효 반각). 판정식 'w_near ≤ f·sinθ_b'면 근접측벽 그레이징 가능=가짜. 빔 넓다고 느끼면 ↑(0.30≈17°), 좁으면 ↓(0.21≈12°) */
#define FRONT_CONFIRM_N     3     /* 그레이징 아닌 '진짜' 정면근접이 N회 연속(유효 측정)일 때만 BRAKE 확정 — 단발 빔간섭 스파이크 오제동 차단. 저속이라 N=3(~21ms)의 추가 접근거리 무시가능 */
#define FRONT_OFFAXIS_DEG   25.0f /* [IMU] 복도축(h_ref) 대비 차체가 이 각 초과로 틀어지면 전방빔이 축이탈→측벽 직격 의심(가짜). 정상 센터링 조향보다 충분히 커야 진짜 정면벽을 안 흘림 */

/* --- 회피/제동 (전부 비블로킹: HAL_GetTick 타임스탬프, 매 루프 센서 갱신) --- */
#define BRAKE_MS        120  /* 능동 제동(L298N short-brake) 유지 시간. 0 = 제동 끄고 코스트(레거시) */
#define SPIN_COMMIT_MS  150  /* 스핀 방향 재평가/탈출 자격까지 최소 유지(블라인드 아님 — 센서는 계속 갱신) */
#define CLEAR_CONFIRM   5    /* 정면 트임 N회 연속(유효 측정만 카운트)이어야 직진 복귀. 3→5: ~6-9ms/loop라 ~35ms 확인 = 코너 대각선 순간트임 무시(조기탈출 차단). 너무 느리면 ↓(4) */
#define TURN_MIN_DEG    60.0f/* [IMU] 스핀 front-트임 탈출 최소 회전각. 25→60: 대각선 초음파 '가짜 트임'에 90°코너서 조기탈출(덜 돌고 직진복귀)하던 것 차단 — 60° 이상 돌아야 front-clear 탈출 허용(TURN_TARGET 85°가 상한 백업). 과회전이면 ↓(50) */
#define TURN_TARGET_DEG 85.0f/* ★[IMU][피벗] 회전각 완료 탈출 목표 (~90° 코너). 정면이 FRONT_CLEAR까지 안 트이는 짧은 미로 복도에서도
                                이 각 돌고 정면이 벽 아님(≥FRONT_TURN)이면 직진 복귀 = 한 번에 깔끔히 꺾기. 덜 돌면 ↑(90), 과회전이면 ↓(80) */
#define TURN_CUTOFF_DEG 100.0f/* ★[req1][IMU] 과회전/역주행 컷오프(>TARGET). 정면 거리 조건이 끝내 충족 안 돼 스핀이 안 끝날 때, 누적 회전각이 이 각 초과면
                                정면 무시하고 강제 종료→CRUISE. 90° 코너서 전면벽<FRONT_TURN라 180° 역주행하던 것 차단. 최대 과회전≈(이값−90). 역주행 잔존 시 ↓(95), 코너 덜 돌면 ↑ */
#define COURSE_REV_DEG  135.0f/* ★[req4][IMU] 역주행 게이트: 현재 leg(course_heading) 대비 |편차|≥이 각이면 ~역방향 → spin→cruise 탈출 금지(계속 회전해 정방향 복귀 후 탈출). 90° 코너(편차~90)는 허용, ~180° 역주행만 차단. 코너 못 빠지면 ↑, 역주행 새면 ↓ */
#define SWAP_LIMIT      1    /* 스핀 1회당 방향 flip 허용 횟수 (좌우 진동 방지) */
#define SPIN_MAX_MS     2500 /* 스핀 타임아웃 → 후진 (IMU 사망 시 스턱 커버) */
#define SPIN_BLIND_MS   800  /* ★[fix] 스핀 진입 후 총 경과 상한[ms] (IMU·swap 무관). 초과 시 강제 종료→CRUISE. 모터 인러시 I2C 글리치로 imu_live=0→각도컷(TURN_CUTOFF) 무력화돼도 역주행 차단. 1000→800: 피벗 ω 98로 낮춰 800ms≈90° 블라인드캡(출발 역주행 차단). dbg.spin_deg 관측해 ±. SPIN_MAX_MS보다 작게 */
#define LAUNCH_MS       800  /* ★[fix①] 자율 시작 직후 ★순수직진(벽센터링 금지) 커밋 시간[ms]: 이 동안 코너/스핀 판단 억제 → 인러시 brownout(imu_live=0→yaw댐핑0) 창 통과 + heading 수렴 + 시작점 즉시 스핀(역주행) 봉쇄. 단 f<FRONT_TURN 또는 측벽 위급이면 즉시 해제해 정상 안전판정으로 복귀 */
#define BACK_CHUNK_MS   350  /* 후진 펄스 1회: 후진→제동→재평가 반복. 120→350: 스턱 시 확실히 후진해 회전반경 재확보(짧으면 공간 못 벌고 재끼임). ⚠뒤는 무센서라 무한정 금지 — REV_MAX_CHUNKS(3)로 총량 제한 */
#define REV_MAX_CHUNKS  3    /* 직진 복귀 없이 연속 후진 chunk 상한 → 초과 시 방향 flip하고 스핀 강행 */
#define HOLD_MAX_MS     1500 /* 정면 무에코 정지(HOLD) 상한 → 스핀으로 탈출(회전이 specular 기하를 바꿔 에코 회복) */
#define ROT_STUCK_MS    400  /* [IMU] 스턱 판정 윈도: 이 시간 동안 회전 명령에도 heading 변화가 */
#define ROT_STUCK_DEG   8.0f /*       이 각도 미만이면 바퀴 헛돎/걸림 → 후진 */

/* --- 초음파 측정 --- */
#define MEAS_WAIT_MS    25   /* 정면 에코 대기 상한(ms). 에코 오면 즉시 복귀(적응형) — 200cm=11.6ms라 충분 */
#define SIDE_WAIT_MS    6    /* 측면 에코 대기 상한(ms). 6ms≈1m — 측면 최대 ~20cm라 충분. 짧을수록
                                무에코(벽 트임/경사) 시 다음 센서로 빨리 넘어가 루프 빨라짐 */
#define ULTRA_MAX_CM    200  /* 먼거리 clear 센티넬 겸 클램프 상한 */
#define FRONT_MIN_WIN   3    /* 정면 min3 윈도(샘플수). 고정 권장 — 키우면 stale 근접값 물어 정지 지연 */
#define SIDE_MED_WIN    3    /* ★측면 median 윈도(샘플수): '한번씩 튀는 값'(스파이크) 제거 강도. 7→3: 위상지연(group delay) 축소가 목적.
                                median은 비선형이라 단발 스파이크는 여전히 제거(3=단발 제거)하면서 지연은 최소(~1샘플) → 헌팅 위상여유 회복.
                                ⚠ FRONT_MIN_WIN(3) 이상·홀수 유지 (hist 배열 크기 겸용) — 3이 하한 */
#define FRONT_FAIL_LIMIT 5   /* 정면 에코 연속 미회신 N회 → HOLD (fail-open 금지; CRUISE에서만 적용,
                                스핀/후진은 무이동·무전방노출이라 면제 — 경사벽 회전 중 freeze 방지) */
#define SIDE_FAIL_LIMIT  3   /* ★측면 에코 연속 미회신 N회 → 해당 측면을 ULTRA_MAX_CM(트임)으로 만료.
                                안 하면 hist가 옛 근접값을 무한 유지 → 코너서 stale 측면값으로 오조향/오회피. 3회≈20-30ms */

/* --- IMU(BNO055) 폴링 --- */
#define IMU_FAIL_LIMIT  5    /* 읽기 연속 실패 N회 → 사망 선언(루프당 10ms timeout 낭비 차단) */
#define IMU_RETRY_MS    500  /* 사망 후 재시도 주기. 복구되면 자동 복귀 */
#define CALIB_POLL_MS   500  /* CALIB_STAT 디버그 읽기 주기 (매 루프 읽을 가치 없음).
                                IMU 모드(gyro+accel)라 mag 비트[1:0]는 항상 0 — 정상 */

/* --- 모터 점검 모드 --- */
#define MOTOR_TEST      0    /* 1=센서 무시, 전진→제동→좌턴→우턴→후진 반복(바퀴/제동 눈 확인). 0=정상 주행 */
/* ============================================================ */

/* 제어기 입력 스냅샷: main.c가 매 루프 채워서 Drive_Update에 전달 */
typedef struct
{
    uint16_t f, l, r;     /* 필터된 거리 cm (정면=min3, 측면=median3) */
    uint8_t  f_valid;     /* 이번 사이클 정면 에코 유효(v_snap[0]) — 탈출 카운트는 유효 측정만 */
    uint8_t  side_valid;  /* l/r이 이번 사이클 측면 측정까지 반영한 값이면 1. early front-only 프레임은 0 */
    uint8_t  front_miss;  /* 정면 연속 무에코 횟수 (HOLD 판정용) */
    float    heading;     /* BNO055 heading [deg 0~360, CW+]. imu_live=0이면 무시됨 */
    uint8_t  imu_live;    /* 이번 제어 프레임에서 heading이 신선하면 1. 실패 프레임은 거리-only로 강등 */
    uint32_t now;         /* HAL_GetTick() */
} DriveInputs;

typedef enum
{
    DS_CRUISE  = 0,   /* 조향 주행 (벽 센터링/등거리/heading-hold) */
    DS_BRAKE   = 1,   /* 능동 제동 BRAKE_MS → 막다른곳이면 REVERSE, 아니면 SPIN */
    DS_SPIN    = 2,   /* 제자리 회전 회피 (전진 이동 0 — 아크 실패/막다른곳 폴백 전용) */
    DS_REVERSE = 3,   /* 막다른곳 후진 chunk */
    DS_HOLD    = 4,   /* 정면 센서 상실 안전정지 (HOLD_MAX_MS 후 SPIN 탈출) */
    DS_SIDE_AVOID = 5,/* 측면 박기직전 비상 회피 (멈춤+박힌쪽 반대 제자리 피벗 → 측면 트이면 CRUISE 복귀) */
    DS_CORNER  = 6    /* 이동 아크 코너 통과 (멈춤 없이 굴러서 90° 코너; 못 돌면 BRAKE→SPIN 자동 폴백) */
} DriveState;

void       Drive_Init(void);                     /* 상태 리셋 (main USER CODE 2에서 1회) */
void       Drive_Update(const DriveInputs *in);  /* 매 루프 1회 — 호출당 정확히 한 번 모터 명령 */

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_H__ */
