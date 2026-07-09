/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drive.h
  * @brief   주행 제어기 공개 API + 튜닝 노브 (비블로킹 상태머신 + 복도 센터링 조향)
  *          상태: CRUISE(조향 주행) → CORNER(전진성 아크 코너) / BRAKE(능동 제동) → SPIN(제자리 피벗 폴백)
  *                ↔ REVERSE(막다른곳 후진) / HOLD(센서 상실 안전정지) / SIDE_AVOID(측벽 비상회피)
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
#define DRIVE_SPEED     32   /* 후진(REVERSE) 기본 구동 속도. 뒤 센서가 없으므로 짧고 약하게만 쓴다.
                                직진 속도는 drive.c CENTER_BASE_SPEED_PCT */
#define TURN_SPEED      42   /* IMU PID 피벗 바깥 바퀴 상한. 직선 속도 상향분을 코너 안정성으로 보상 */
#define TURN_INNER      20   /* IMU 사망 시 블라인드 피벗 안쪽 바퀴 후진 속도 */
/* ── 아크(Arc) 코너 부활: 전진성 곡선 주행으로 90° 코너 통과 (제자리 피벗의 대각 끼임/충돌 회피).
 *    Car_ArcLeft/Right(outer,inner) = 양바퀴 '전진', 안쪽만 감속 → 곡선 반경 R≈(T/2)·(O+I)/(O−I) (T=윤거).
 *    피벗(제자리)은 아크 실패/막다른곳 폴백 전용으로 강등. */
#define ARC_OUTER        60   /* 코너 아크 바깥 바퀴 전진 [%duty]. [IMG_2999] 50→60: 코너를 '굴러서' 통과 —
                                 안쪽바퀴 동반 상향으로 반경은 유지(R≈17cm)하고 평균속도만 +20% → 멈추는 느낌 제거.
                                 (testtrack 코너 8개 × ~1s → 20s 목표의 코너 몫. 아크 중단선은 CORNER_ABORT_CM이 별도 방어) */
#define ARC_INNER        25   /* 코너 아크 안쪽 바퀴 전진 [%duty]. [IMG_2999] 21→25: outer와 비례 상향 →
                                 반경 유지한 채 안쪽바퀴가 안 멈춰 '제자리 회전' 인상 제거(부드러운 롤) */
#define ARC_APPROACH_OUTER 48 /* 목표각 근접 시 감속 아크 바깥 바퀴. [IMG_2999] 36→48: 출구 크롤 완화 */
#define ARC_APPROACH_INNER 20 /* 목표각 근접 시 감속 아크 안쪽 바퀴. [IMG_2999] 16→20 */
#define ARC_APPROACH_DEG 26.0f /* 목표각까지 이 각도 이내부터 선형 감속 램프. [IMG_2999] 34→26: 감속 구간 축소 → 코너 속도 유지 */
#define ARC_MAX_MS       850  /* 아크 타임아웃 백스톱 → BRAKE(피벗 폴백). 저속 아크에 맞춰 여유 복원 */
#define MOTOR_MIN_PCT   30   /* TT모터 스톨 하한(개념 기준). CRUISE 전진 PWM은 이 값 아래로 떨어지면 바닥값을 적용 */
#define MOTOR_TRIM_PCT  0    /* ★직진 편향 보정 [%duty, CRUISE 직진 전용]: 차가 좌로 쏠리면 +(좌바퀴↑/우바퀴↓), 우로 쏠리면 −. 0=보정없음. 바닥서 직진 쏠림 관찰 후 ±1~3 조정 */

/* --- 거리 임계 (cm) : 핵심 민감도 (관계 유지: DANGER < STOP < TURN < CLEAR < ARC). ★고속화로 관성↑ → 제동선 선제 확대 --- */
#define FRONT_STOP_CM   28   /* 미만이면 즉시 능동 제동(비상). 차 길이 27cm + base 66% 관성이라 정면 여유를 한 박자 더 둔다 */
#define CORNER_ABORT_CM 16   /* ★아크 '중' 전용 정면 중단선. 직선용 STOP(28)을 아크에 그대로 쓰면 안 되는 기하 근거:
                                37×37cm 코너 정션에서 정상 90° 아크도 회전 중 전방빔이 외벽을 스치며
                                정션중심→외벽 37·cos45°≈26cm − 전방 오버행 ~10cm ≈ 16cm까지 '정상적으로' 떨어진다.
                                아크 평균 duty(≈34%)는 직선(66%)보다 관성이 낮아 → DANGER(12)+4cm면 충분.
                                28로 두면 멀쩡한 아크를 BRAKE→SPIN(제자리, ~1.5s 손실)으로 강등 (IMG_2974 4.5~5.0s) */
#define FRONT_TURN_CM   36   /* 정면 이 거리 미만 + 측면 트임 없음 = 제동(브레이크)→피벗 폴백.
                                박으면 ↑(36), 측벽 끼면 ↓ */
#define FRONT_CLEAR_CM  44   /* 이상이 CLEAR_CONFIRM회 연속이면 회전/아크 종료→직진 복귀. 진입(TURN)보다 +8cm 히스테리시스 유지.
                                짧은 복도서 여기까지 안 트이면 회전각(TURN_TARGET/CUTOFF) 탈출이 백업이라 데드락 없음. 미로 넓으면 ↑ */
#define FRONT_ARC_CM    56   /* 정면 이 거리 미만 + 한쪽 측면 트임/비대칭 = 전진성 아크 코너 진입.
                                [IMG_2999] 52→56: 직선 속도를 올린 만큼 코너 진입을 조금 앞당겨 급턴/벽 밀기 대신 부드럽게 들어간다.
                                CORNER_GRID_EXIT_CM(60)보다 낮아 90° 조기탈출 오발과는 무관 */
#define FRONT_DANGER_CM 12   /* 코앞 경계(막다른곳 후진 판정) */
#define SIDE_BLOCK_CM   8    /* 측면 막힘 판정. 최협 37cm 중앙 여유 10.5cm보다 낮게 둬 정상 주행을 막힘으로 보지 않는다 */
#define SIDE_HYST       6    /* 좌우 트임 차 데드밴드(cm): 이만큼 차이나야 회전 방향 결정/재평가 */
#define SIDE_OPEN_CM    22   /* 측면 이 거리 이상 = 그쪽이 트임. 55cm 이상 구간도 코너 여유로 잡는다 */
#define CORNER_ASYM_CM  10   /* 좌우 측거리 비대칭 ≥이값이면 더 트인 쪽 곡선코너로 인지 */
#define CORNER_ASYM_OPEN_CM 24 /* 비대칭 코너 인정 시 더 먼 쪽 최소 거리. 단순 센서 튐을 코너로 보지 않는다 */
#define CORNER_NEAR_SAFE_CM 7  /* 비대칭 코너 인정 최소 근접측 여유. [헤딩 단독] 10→7: 구 10은 '중앙주행(여유 10.5cm)'
                                  전제라 횡 센터링이 빠진 헤딩 단독 모드서 중앙 1cm만 이탈해도 코너 후보가 기각됨
                                  (턴 지연/미발동 원인). 7 미만 위급은 SIDE_AVOID(9cm)가 별도 계층에서 방어 */
#define CORNER_CONFIRM_N   2   /* 코너 후보 연속 확인. front/ToF 단발 튐으로 직선에서 아크 진입하는 것을 차단 */
#define CORNER_ENTRY_HDG_MAX_DEG 10.0f /* 코너 후보라도 복도축 대비 이 각도 이상 틀어져 있으면 그레이징/대각 주행으로 본다 */

/* --- 조향 아키텍처 선택 --- */
#define DRIVE_HEADING_PRIMARY 1 /* 1 = 헤딩 단독 조향: CRUISE 조향은 BNO055 heading-hold만 사용.
                                     센서(전방 초음파/측면 ToF)는 ①코너(좌/우회전) 판별 ②근접 가드·
                                     비상회피(안전) ③속도캡 에만 쓰인다. 직진 = h_ref(시작 180° → 그리드축)
                                     추적, 코너 = 판별 순간 아크로 돌고 탈출 시 h_ref가 다음 45°/90° 축으로
                                     점프 → 새 축을 다시 heading-hold. IMU 사망 프레임은 자동으로 벽 센터링
                                     (레거시)으로 강등되므로 주행이 끊기지 않는다.
                                   0 = 레거시 벽 센터링 혼합(등거리 P/D + heading 보조) */

/* --- 조향 (CRUISE 복도 센터링) ---
 * ⚠ 살아있는 P/D/슬루/데드존/속도 노브는 전부 drive.c의 CENTER_* 블록에 있음.
 *   여기 둘만 drive.c가 직접 참조: heading-hold P게인 + yaw-rate 댐핑 D게인. */
#define KP_HDG          0.82f/* heading-hold P게인 [%duty/deg]. [헤딩 단독] 0.90 과조향(리밋사이클) ↔ 0.72 유지력부족(좁은길
                                벽박기) 사이 절충 0.82. 정상상태 잔차는 CENTER_HDG I항, 위치는 폭인지 센터링 바이어스가 맡고
                                P는 헤딩 유지 담당. KD_YAW 댐핑 상향과 합쳐 좁은길에서도 헤딩을 단단히 물되 헌팅은 억제.
                                (DRIVE_HEADING_PRIMARY=0 레거시로 되돌리면 blend와 곱해지니 그때 0.58 복원) */
#define KD_YAW          0.36f/* yaw-rate 댐핑 D게인 [%duty per deg/s]. [헤딩 단독] 0.30→0.36: KP_HDG/센터링 상향에 맞춰
                                감쇠도 키워 좌우 흔들림(리밋사이클)을 확실히 잡는다. 좌우 흔들림(리밋사이클) 상쇄 —
                                gyro 회전속도를 음의 피드백으로. drive.c에서 deadband/상한을 두고,
                                전방이 열린 직진에서는 BNO055 진행각을 우선해 대각 주행을 빨리 접는다.
                                imu_live일 때만 적용(사망 시 P-only로 자동 강등) */

/* --- 측면 비상 회피 (조향으로 못 막은 측벽 박힘: 박힌쪽 반대로 전진 아크 이탈) --- */
#define SIDE_AVOID_CM       6   /* 진입: 한쪽 측면이 이 값 미만 = 진짜 벽 스침 직전 → 비상 회피(불연속).
                                   [헤딩 단독] 9→6: 구 9는 최협 중앙여유 10.5cm에서 1.5cm만 흘러도 발동 →
                                   직진 중 '멈추고 틀기'(불연속 회피 아크)의 주범. 6~10cm 구간은 CRUISE의 부드러운
                                   센터링 바이어스 + 근접가드(8cm)가 달리면서 처리 → 정말 부딪히기 직전(6cm)만 회피 */
#define SIDE_AVOID_CLEAR_CM 8   /* 탈출: 근접가드 유효권(8cm)까지 회복되면 CRUISE 부드러운 보정에 넘긴다 */
#define SIDE_AVOID_CLEAR_CONFIRM 3 /* 탈출 조건 연속 확인. 진입선을 9cm로 올린 만큼 조기 복귀 핑퐁을 더 막는다 */
#define SIDE_AVOID_MAX_MS   360 /* 전진 아크 회피 백스톱 */
#define SIDE_ESCAPE_OUTER   40  /* 측면 회피 아크 바깥 바퀴 */
#define SIDE_ESCAPE_INNER   12  /* 측면 회피 아크 안쪽 바퀴. 벽에 붙은 상태에서는 더 작게 돌아 빠져나온다 */

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
#define SPIN_COMMIT_MS  100  /* 스핀 방향 재평가/탈출 자격까지 최소 유지 */
#define CLEAR_CONFIRM   3    /* 정면 트임 N회 연속(유효 측정만 카운트)이면 직진 복귀 */
#define TURN_MIN_DEG    45.0f/* 스핀 front-트임 탈출 최소 진행각. 방향 기준 진행각으로 판정한다 */
#define TURN_TARGET_DEG 88.0f/* 목표 회전각. PID 감속으로 90도에 가깝게 끊는다 */
/* ── 45° 코스 그리드 heading: testtrack 레그축은 전부 시작방위+45°×k. 회전/아크/회피 탈출 시
 *    h_ref를 '현재 자세'나 'h_entry±90'이 아니라 시작방위 기준 가장 가까운 45° 그리드축으로 스냅.
 *    코너 진입 시 대각 자세(δ)가 다음 레그 기준으로 유전되는 것과, 45° 코너에서 ±90 고정 스냅이
 *    기준을 45° 오염시키는 것을 동시에 차단 (IMG_2986 직진 지그재그/45°코너 과회전의 구조 원인) */
#define CORNER_GRID_EXIT_CM      60   /* 45° 코너 조기탈출 전방 트임. 52→60: 이론상 90° 정션 중간(대각) 외벽 빔은
                                         ~26..47cm지만 실주행(웹앱)에서 90° 코너가 45°에서 조기탈출하는 관찰 →
                                         빗면 산란/개구부로 52를 넘는 사례 존재. 47cm 상한 대비 마진을 13cm로 확대.
                                         진짜 45° 출구는 새 레그 축방향 정반사(레그 길이 ≥0.5m)라 60도 쉽게 넘는다 */
#define CORNER_GRID_EXIT_MIN_DEG 35.0f/* 그리드 조기탈출 최소 회전각. 미회전 상태 오탈출 차단 */
#define CORNER_GRID_EXIT_MAX_DEG 65.0f/* 그리드 조기탈출 최대 회전각. 이 밖(≥65°)은 90° 코너로 보고 exit A에 위임 */
#define CORNER_GRID_ALIGN_DEG    10.0f/* 그리드축 정렬 판정 허용각 */
#define CORNER_EXIT_MIN_DEG 80.0f/* 아크 코너 front-clear 탈출 최소각. 조기탈출로 대각 직진하는 현상 방지 */
#define TURN_CUTOFF_DEG 96.0f/* 과회전 컷오프. 이 각을 넘으면 정면 조건보다 과회전 방지를 우선 */
#define TURN_WRONG_DEG  20.0f/* 명령 반대 방향으로 이 각 이상 돌면 방향 래치를 뒤집는다 */
#define TURN_PID_KP     0.62f/* 피벗 heading PID: 남은 각도 P [%duty/deg] */
#define TURN_PID_KI     0.018f/* 피벗 heading PID: 저속 마찰 보상 I */
#define TURN_PID_KD     0.14f/* 피벗 heading PID: 회전속도 감쇠 D [%duty/(deg/s)] */
#define TURN_PID_I_MAX  130.0f/* 피벗 I항 적분 클램프 [deg*s] */
#define TURN_PID_MIN_PCT 28  /* 피벗 외륜 최저 듀티 */
#define TURN_PID_FINE_MIN_PCT 22 /* 목표 근처 피벗 외륜 최저 듀티 */
#define TURN_PID_FINE_DEG 14.0f  /* 이 각도 이내에서 fine 최소 듀티 사용 */
#define TURN_PID_INNER_RATIO 0.48f /* 피벗 내륜 역구동 비율. 작을수록 후진성/벽 긁힘 감소 */
#define COURSE_REV_DEG  115.0f/* 현재 leg 대비 이 각 이상이면 역방향 위험 */
#define SWAP_LIMIT      1    /* 스핀 1회당 방향 flip 허용 횟수 (좌우 진동 방지) */
#define SPIN_MAX_MS     1600 /* 스핀 타임아웃 → 후진. PID 감속 여유 */
#define SPIN_BLIND_MS   650  /* IMU 사망 시 시간 기반 회전 상한 */
#define LAUNCH_MS       120  /* 자율 시작 직후 순수직진 커밋 시간. 긴 런치는 측벽 충돌을 만든다 */
#define BACK_CHUNK_MS   220  /* 후진 펄스 1회. 뒤 센서가 없으므로 짧게 */
#define REV_MAX_CHUNKS  1    /* 직진 복귀 없이 연속 후진 chunk 상한 */
#define HOLD_MAX_MS     1500 /* 정면 무에코 정지(HOLD) 상한 → 스핀으로 탈출(회전이 specular 기하를 바꿔 에코 회복) */
#define ROT_STUCK_MS    400  /* [IMU] 스턱 판정 윈도: 이 시간 동안 회전 명령에도 heading 변화가 */
#define ROT_STUCK_DEG   8.0f /*       이 각도 미만이면 바퀴 헛돎/걸림 → 후진 */

/* --- 초음파 측정 --- */
#define MEAS_WAIT_MS    6    /* 정면 에코 대기 상한(ms). 80cm echo≈4.64ms라 코스용 상한만 기다린다 */
#define SIDE_WAIT_MS    6    /* 측면 에코 대기 상한(ms). 80cm echo≈4.64ms + RTOS tick 여유 */
#define ULTRA_MAX_CM    80   /* 먼거리 clear 센티넬 겸 클램프 상한. testtrack 폭(37~67cm) 기준 */
#define FRONT_MED_WIN   3    /* 정면 median3 윈도(샘플수). 단발 low outlier가 가짜 코너/제동을 만들지 않게 한다 */
#define SIDE_MED_WIN    3    /* ★측면 median 윈도(샘플수): '한번씩 튀는 값'(스파이크) 제거 강도. 7→3: 위상지연(group delay) 축소가 목적.
                                median은 비선형이라 단발 스파이크는 여전히 제거(3=단발 제거)하면서 지연은 최소(~1샘플) → 헌팅 위상여유 회복.
                                ⚠ FRONT_MED_WIN(3) 이상·홀수 유지 (hist 배열 크기 겸용) — 3이 하한 */
#define FRONT_FAIL_LIMIT 5   /* 정면 에코 연속 미회신 N회 → front hist를 MAX로 비워 stale 근접값 제거.
                                CRUISE HOLD는 측면까지 MAX일 때만 적용한다. front-left-front-right라 약 2.5루프 */
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
    uint16_t f, l, r;     /* 필터된 거리 cm (정면=median3, 측면=median3) */
    uint8_t  f_valid;     /* 이번 사이클 정면 에코 유효(v_snap[0]) — 탈출 카운트는 유효 측정만 */
    uint8_t  side_valid;  /* l/r이 이번 사이클 측면 측정까지 반영한 값이면 1. early front-only 프레임은 0 */
    uint8_t  front_miss;  /* 정면 연속 무에코 횟수 (stale front clear / 센서 상실 상태 입력) */
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
    DS_HOLD    = 4,   /* 센서/트랙 상실 안전정지 (HOLD_MAX_MS 후 SPIN 탈출) */
    DS_SIDE_AVOID = 5,/* 측면 박기직전 비상 회피 (박힌쪽 반대 전진 아크 → 측면 트이면 CRUISE 복귀) */
    DS_CORNER  = 6    /* 이동 아크 코너 통과 (멈춤 없이 굴러서 90° 코너; 못 돌면 BRAKE→SPIN 자동 폴백) */
} DriveState;

void       Drive_Init(void);                     /* 상태 리셋 (main USER CODE 2에서 1회) */
void       Drive_Update(const DriveInputs *in);  /* 매 제어 입력 1회. early front-only 프레임은 직전 PWM을 유지할 수 있음 */

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_H__ */
