/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drive.h
  * @brief   주행 제어기 공개 API + 튜닝 노브 (비블로킹 상태머신 + 복도 센터링 조향)
  *          상태: CRUISE(조향 주행) → BRAKE(능동 제동) → SPIN(제자리 회전 회피)
  *                ↔ REVERSE(막다른곳 후진) / HOLD(정면 센서 상실 안전정지)
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
/* --- 속도 (0~100%) --- */
#define DRIVE_SPEED     32   /* 직진 기본 속도. 흔들림/측벽충돌 완화 위해 40→32 저속화(안정 우선) */
#define DRIVE_MIN_PCT   30   /* 감속 하한(STOP 직전 속도). MOTOR_MIN_PCT(30)에 정렬, DRIVE_SPEED 이하 유지 */
#define TURN_SPEED      40   /* [스핀] 회전 '바깥' 바퀴 전진 속도 */
#define TURN_INNER      30   /* [스핀] 회전 '안쪽' 바퀴 후진 속도. 0=정지(마찰로 안돎), 소량=깔끔한 선회(권장) */
#define MOTOR_MIN_PCT   30   /* TT모터 부하 시 스톨 하한. 조향으로 바퀴가 (0,이값) 구간에 떨어지면
                                이값으로 올리고 부족분을 반대 바퀴에 가산(spillover) → 회전율 보존 */

/* --- 거리 임계 (cm) : 핵심 민감도 (관계 유지: DANGER < STOP < CLEAR, 갭 ≥15) --- */
#define FRONT_STOP_CM   20   /* 미만이면 즉시 능동 제동 + 회피 진입. 빠른 속도/큰 관성이면 ↑ */
#define FRONT_CLEAR_CM  38   /* 이상이 CLEAR_CONFIRM회 연속(+회전각 충족)이어야 직진 복귀 (히스테리시스) */
#define FRONT_DANGER_CM 12   /* 코앞 경계: 좌우도 막혔으면(막다른곳) 후진 */
#define FRONT_SLOW_CM   45   /* 이 거리부터 STOP까지 DRIVE_SPEED→DRIVE_MIN_PCT 선형 감속(정지거리 마진 확보) */
#define SIDE_BLOCK_CM   6    /* 측면 막힘 판정. 트랙39/스팬19 → 중앙 여유 10cm라 반드시 10 미만이어야
                                중앙 주행을 '막힘'으로 오판 안 함. 바짝 붙었을 때만 막힘=6 */
#define SIDE_HYST       5    /* 좌우 트임 차 데드밴드(cm): 이만큼 차이나야 회전 방향 결정/재평가 (좁은 트랙→축소) */

/* --- 조향 (CRUISE 복도 센터링: 양벽→센터링, 한벽→등거리, 무벽→IMU heading-hold) --- */
#define KP_WALL         0.22f/* 벽 조향 P게인 [%duty/cm]. 발진(좌우 흔들림) 억제 위해 0.5→0.30→0.22. 반응 둔하면 ↑ (D항이 반응 보완) */
#define KP_HDG          0.8f /* heading-hold P게인 [%duty/deg]. IMU 사망 시 자동 open-loop 직진 */
#define KD_YAW          0.15f/* yaw-rate 댐핑 D게인 [%duty per deg/s]. 좌우 흔들림(리밋사이클) 상쇄 —
                                gyro 회전속도를 음의 피드백으로. 0.10→0.15(댐핑 강화), 잔존 시 ↑(0.20), 조향 둔해지면 ↓.
                                imu_live일 때만 적용(사망 시 P-only로 자동 강등) */
#define STEER_MAX_PCT   8    /* 조향량 상한 [%duty]. 바퀴差 최대치 — 급조향/과보정 차단 위해 12→8 */
#define STEER_SLEW_PCT  1    /* 루프당 조향 변화 상한 [%duty/루프]. ← 사용자가 말한 '딜레이': 작을수록 조향이
                                루프마다 조금씩만 변해 느리고 부드럽게 반응(발진 억제). 너무 둔하면 ↑(2~3) */
#define WALL_DB_CM      3    /* 양벽 센터링 데드밴드 [cm]: |l-r| 이내면 직진. 중앙 여유 ~12cm라 ±6이면 절반.
                                4→6으로 중앙 불감대 확대 = 미세 좌우차 무시. 흔들림 남으면 ↑(8) */
#define SIDE_TRACK_CM   14   /* ★좌우 인식거리 [cm]: 벽이 이 거리 이내여야 '보임'=조향 입력. 28→14로 축소 —
                                중앙(양쪽~12cm)에선 약하게만 양벽 반응, 한쪽 14cm 이내 접근 시 그쪽만 밀어냄.
                                ★더 줄이면(예 10) 중앙선 측면조향 OFF→IMU heading-hold 직진(가장 안정, imu_live 시) */
#define SIDE_KEEP_CM    8    /* 한벽 모드 목표거리 하한(중앙 여유 10보다 약간 아래). 좁은 트랙이라 축소 */
#define SIDE_SET_MAX_CM 14   /* 한벽 모드 목표거리 상한 [SIDE_KEEP_CM, 이값]. 한벽붙음 범위 내로 */

/* --- 측면 비상 회피 (조향으로 못 막은 측벽 박힘: 멈춤+박힌쪽 반대로 제자리 피벗) --- */
#define SIDE_AVOID_CM       5  /* ★진입: 한쪽 측면 이 거리[cm] 미만 = 박기 직전 → SIDE_AVOID 진입(전진0 피벗).
                                  ↑(7)하면 더 일찍 개입(안전↑·멈칫↑), ↓(4)면 더 늦게(매끄럽지만 박힐 위험) */
#define SIDE_AVOID_CLEAR_CM 8  /* 탈출: 측면 min(l,r)이 이 거리[cm] 이상 회복 시 CRUISE 복귀. 진입(5)보다 커야 채터 방지 */
#define SIDE_AVOID_MAX_MS   600 /* 한 방향 피벗 후 방향 재평가(flip) 백스톱[ms] — 한쪽으로 못 빠지면 반대로 시도 */

/* --- 회피/제동 (전부 비블로킹: HAL_GetTick 타임스탬프, 매 루프 센서 갱신) --- */
#define BRAKE_MS        120  /* 능동 제동(L298N short-brake) 유지 시간. 0 = 제동 끄고 코스트(레거시) */
#define SPIN_COMMIT_MS  150  /* 스핀 방향 재평가/탈출 자격까지 최소 유지(블라인드 아님 — 센서는 계속 갱신) */
#define CLEAR_CONFIRM   3    /* 정면 트임 N회 연속(유효 측정만 카운트)이어야 직진 복귀 */
#define TURN_MIN_DEG    25.0f/* [IMU] 스핀 탈출 최소 회전각: 경사벽 specular '가짜 트임' 조기탈출 차단 */
#define SWAP_LIMIT      1    /* 스핀 1회당 방향 flip 허용 횟수 (좌우 진동 방지) */
#define SPIN_MAX_MS     2500 /* 스핀 타임아웃 → 후진 (IMU 사망 시 스턱 커버) */
#define BACK_CHUNK_MS   120  /* 후진 펄스 1회: 후진→제동→재평가 반복(조금씩 빠져나오기, 뒤는 무센서) */
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
#define SIDE_MED_WIN    5    /* ★측면 median 윈도(샘플수): '한번씩 튀는 값'(스파이크) 제거 강도.
                                3=단발만 / 5=2연속까지 / 7=3연속까지 제거(반응 약간 느려짐). 흔들림 남으면 ↑.
                                ⚠ FRONT_MIN_WIN(3) 이상·홀수 유지 (hist 배열 크기 겸용) */
#define FRONT_FAIL_LIMIT 5   /* 정면 에코 연속 미회신 N회 → HOLD (fail-open 금지; CRUISE에서만 적용,
                                스핀/후진은 무이동·무전방노출이라 면제 — 경사벽 회전 중 freeze 방지) */

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
    uint8_t  front_miss;  /* 정면 연속 무에코 횟수 (HOLD 판정용) */
    float    heading;     /* BNO055 heading [deg 0~360, CW+]. imu_live=0이면 무시됨 */
    uint8_t  imu_live;    /* IMU 데이터 신뢰 가능 여부 (사망 시 거리-only로 자동 강등) */
    uint32_t now;         /* HAL_GetTick() */
} DriveInputs;

typedef enum
{
    DS_CRUISE  = 0,   /* 조향 주행 (벽 센터링/등거리/heading-hold) */
    DS_BRAKE   = 1,   /* 능동 제동 BRAKE_MS → 막다른곳이면 REVERSE, 아니면 SPIN */
    DS_SPIN    = 2,   /* 제자리 회전 회피 (전진 이동 0 — arc 폐지) */
    DS_REVERSE = 3,   /* 막다른곳 후진 chunk */
    DS_HOLD    = 4,   /* 정면 센서 상실 안전정지 (HOLD_MAX_MS 후 SPIN 탈출) */
    DS_SIDE_AVOID = 5 /* 측면 박기직전 비상 회피 (멈춤+박힌쪽 반대 제자리 피벗 → 측면 트이면 CRUISE 복귀) */
} DriveState;

void       Drive_Init(void);                     /* 상태 리셋 (main USER CODE 2에서 1회) */
void       Drive_Update(const DriveInputs *in);  /* 매 루프 1회 — 호출당 정확히 한 번 모터 명령 */
DriveState Drive_GetState(void);                 /* main의 선제 제동 힌트용 (CRUISE 게이트) */

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_H__ */
