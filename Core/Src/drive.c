/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drive.c
  * @brief   주행 제어기 구현 — 비블로킹 상태 머신 + 복도 센터링 조향
  *
  *          설계 원칙 (벽 박힘 근절):
  *          - HAL_Delay 없음: 모든 유지시간은 HAL_GetTick 타임스탬프 → 매 루프 센서 갱신
  *          - 코너: 전진성 아크(DS_CORNER, Car_Arc*)로 굴러서 통과 — 좁은 미로 대각 끼임/충돌 회피.
  *            제자리 피벗(SPIN)은 아크 실패/막다른곳 폴백으로만 사용 (전진 이동 0)
  *          - 모든 회전/정지에 피드백: 능동 제동(BRAKE), IMU 회전각 탈출 게이트,
  *            스턱 감지 — 시간×듀티 개루프 없음 → 배터리 sag 면역
  *          - IMU 사망 시 전 기능이 거리-only로 자동 강등 (주행 중단 없음)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "drive.h"
#include "motor.h"
#include "debug.h"
#include <math.h>

/* ---- 디버그 미러 인스턴스 (SWD Live Expressions 대상) ----
 * 구조체/선언은 debug.h, 정의는 여기 1곳. 모든 미러는 dbg.<member>로 접근 */
volatile DebugMonitor_t dbg;

/* ---- 조향 모드 (dbg.steer_mode 인코딩) ---- */
#define SM_BOTH   0   /* 양벽 센터링: u = CENTER_KP × (l − r) */
#define SM_HDG    3   /* 무벽: IMU heading-hold */
#define SM_OPEN   4   /* 무벽 + IMU 사망: open-loop 직진 (레거시 거동) */
#define SM_SINGLE 5   /* 한쪽 벽 추종 + heading-hold 보조 */

/* ---- Narrow-course centering control ---- */
#define CENTER_LPF_ALPHA           0.74f   /* median 후 low-pass 새 샘플 가중치. 직진 흔들림을 빨리 잡기 위해 ToF 반응을 더 연다.
                                              직선에서도 좌우 거리 변화가 곧바로 조향으로 증폭되므로 median3 뒤에
                                              약한 LPF를 다시 둬 한 샘플 튐/개구부 스침을 흡수한다 */
#define CENTER_DEADZONE_CM         2.0f    /* 좌우 등거리 소프트존. hard-zero가 아니라 아래 INNER_SCALE로 계속 약보정해
                                              중앙에서 방치→한쪽 벽 접근→반대조향 반복되는 직선 헌팅을 줄인다 */
#define CENTER_INNER_KP_SCALE      0.35f   /* |l-r|<=DEADZONE 안쪽의 센터링 P 축소율. 0이 아니므로 직진 중에도
                                              양쪽 거리 균형을 계속 천천히 맞춘다 */
#define CENTER_KP                  0.11f   /* 측면거리 등거리 P gain. IMG_2985는 좌우로 밀리는 시간이 길어 기본 복귀권한을 소폭 회복.
                                              반대벽까지 밀어내는 패턴이라 우선 감쇠비를 높인다 */
#define CENTER_KD                  0.004f  /* 측면거리 D gain. ToF/median 계단 변화가 duty 킥으로 커지지 않게 제한 */
#define CENTER_STEER_MAX_PCT       24.0f   /* 직선 조향 상한. straight-lock에서 빠르게 중앙 복귀할 여유 확보 */
#define CENTER_STEER_SLEW_PCT      5.0f    /* 조향 변화율 상한 [%duty/loop]. 직진 복귀 지연을 줄이되 급반전은 제한 */
#define CENTER_BASE_SPEED_PCT      66.0f   /* 일반 직선 기본 속도. IMG_2985는 제어가 느려 속도도 못 나는 상태라 heading-lock 기반으로 상향 */
#define CENTER_STRAIGHT_FAST_SPEED_PCT 70.0f /* front가 트이고 heading/좌우균형이 맞을 때 쓰는 직선 최고속 */
#define CENTER_NARROW_FAST_SPEED_PCT 66.0f /* 37/43/45cm 레그도 straight-lock이면 base까지는 허용 */
#define CENTER_WIDE_FAST_SPEED_PCT 70.0f /* 60/67cm 레그는 여유가 크므로 자세가 맞으면 더 빠르게 통과 */
#define CENTER_STRAIGHT_FAST_ERR_CM 8.0f   /* 좌우 거리차가 이 안이면 빠른 직선으로 본다 */
#define CENTER_STRAIGHT_FAST_HDG_DEG 8.0f  /* heading 오차가 이 안일 때만 최고속 허용 */
#define CENTER_MIN_SPEED_PCT       36.0f   /* 큰 조향 시 속도 하한 */
#define CENTER_FRONT_FAST_CM       64.0f   /* 정면이 이 거리 이상이면 직선 속도 허용. 72cm 감속은 IMG_2983에서 너무 일찍 닫힘 */
#define CENTER_FRONT_SLOW_CM       42.0f   /* 정면이 이 거리 이하이면 선제 감속 하한으로 제한 */
#define CENTER_FRONT_MIN_SPEED_PCT 34.0f   /* 정면 근접 선제 감속 하한. 상태머신 BRAKE 전에 관성만 줄인다 */
#define CENTER_SIDE_SOFT_CM        11.0f   /* ★벽 반발/감속 시작선. '최협 37cm 중앙 주행은 벌점 없어야 한다'가 설계 제약:
                                              중앙여유 (37−16)/2 = 10.5cm인데 구 13cm는 최협·43cm 레그서 완벽 중앙에서도
                                              상시 감속캡(46%)을 물림 → IMG_2974 9~14.5s 최장직선 0.29m/s 저속의 주범.
                                              10.5+0.5cm로 내려 중앙 주행 무벌점, 8~11cm 접근 구간만 감속 */
#define CENTER_SIDE_HARD_CM        8.0f    /* 실제 벽 스침 위험선. 정상 최협 중앙주행을 위험으로 묶지 않는다 */
#define CENTER_SIDE_REPEL_KP       1.35f   /* 측면 근접 반발 조향 [%duty/cm] */
#define CENTER_SIDE_MIN_SPEED_PCT  32.0f   /* 측면 근접 시 속도 하한. 벽 붙음 상태에서는 탈출보다 속도 저감이 먼저다 */
#define CENTER_SINGLE_TARGET_CM    16.0f   /* 한쪽 벽만 보일 때 유지할 목표 측면 거리.
                                              ★반발존(SOFT 11cm) 밖에 둔다: 구 12cm는 센서 지연/흔들림 때 반발존에 쉽게 들어가
                                              추종P(당김)와 반발KP(밀침)가 대립 → 벽 hug + 측면 속도캡 고착(IMG_2973 16.5~17.5s 우벽 hug 원인).
                                              단일벽 모드는 넓은 leg(43~67cm) 전용(최협 37cm는 양벽 SM_BOTH) → 16cm여도 반대측 여유 ≥11cm */
#define CENTER_SINGLE_KP           0.22f   /* 한쪽 벽 추종 P gain [%duty/cm] */
#define CENTER_SINGLE_HDG_BLEND    0.42f   /* 단일벽 추종 중 heading-hold 보조 비율 */
#define CENTER_BOTH_HDG_BLEND      0.32f   /* 양벽 센터링 중 BNO055 heading 보조 비율. 직선에서는 벽 등거리 제어가 1순위,
                                              heading은 차체가 중앙 근처일 때만 자세를 다듬는다 */
#define CENTER_STRAIGHT_HDG_BLEND  0.78f   /* front가 트인 직선에서는 BNO055 heading 기준을 강하게 유지한다 */
#define CENTER_STRAIGHT_SIDE_KP    0.18f   /* straight-lock 좌우 ToF 등거리 P. 직선 중 양쪽값을 빠르게 같게 만든다 */
#define CENTER_STRAIGHT_SIDE_KD    0.006f  /* straight-lock 좌우 ToF D. 좌우로 붙기 시작하는 속도를 바로 꺾는다 */
#define CENTER_NARROW_KP           0.13f   /* 37/43/45cm 레그: 측면 여유가 작아 좌우거리 균형을 더 강하게 잡는다 */
#define CENTER_NARROW_HDG_BLEND    0.45f   /* 좁은 레그에서도 heading 기준을 유지해 한쪽으로 기울어 달리지 않게 한다 */
#define CENTER_WIDE_KP             0.07f   /* 60/67cm 레그: 벽거리 오차를 과하게 쫓지 말고 축 유지 위주 */
#define CENTER_WIDE_HDG_BLEND      0.46f   /* 넓은 레그에서는 heading-hold 권한을 키워 대각 주행을 빨리 접는다 */
#define CENTER_WIDE_STRAIGHT_HDG_BLEND 0.60f /* 넓고 전방이 트인 직선의 heading 복귀 boost */
#define CENTER_STRAIGHT_HDG_GATE_CM 4.0f   /* 좌우 거리차가 이 안일 때만 직선 heading boost 적용 */
#define CENTER_BOTH_HDG_MIN_SCALE  0.20f   /* 좌우 거리 오차가 클 때도 heading 보조를 완전히 끄지는 않고 이 비율만 남긴다 */
#define CENTER_BOTH_HDG_GATE_CM    8.0f    /* |l-r|가 DEADZONE→이 값으로 커질수록 heading 보조를 줄여 벽 균형이 우선한다 */
#define CENTER_HDG_DEADBAND_DEG    2.0f    /* 이 이하 heading 오차는 조향하지 않는다. 직선 복귀를 조금 앞당기되 gyro 미세진동은 차단 */
#define CENTER_HREF_BLEND          0.006f  /* 양벽 안정 시 heading 기준을 실제 복도축으로 보정하는 속도.
                                              그리드 스냅 기준이 실제 leg축과 미세하게 어긋나면(트랙 비직각/자이로 드리프트)
                                              phantom hdg_err가 heading 속도캡을 물고 늘어질 때만 천천히 보정한다. 흔들리는
                                              차체각을 기준으로 따라가면 대각 주행이 기준화되어 벽으로 간다 */
#define CENTER_HREF_ALIGN_ERR_CM   3.0f    /* 거의 중앙일 때만 h_ref 보정. 최협 중앙여유 10.5cm에서 3cm 밖은 이미 자세 교정 우선 */
#define CENTER_HREF_ALIGN_HDG_DEG  8.0f    /* 이 heading 오차 안에서만 h_ref를 현재 복도축으로 보정 */
#define CENTER_HREF_ALIGN_DERR_CMS 18.0f   /* 좌우 오차 변화가 작을 때만 h_ref 보정 */
#define CENTER_HREF_ALIGN_YAW_DPS  8.0f    /* 회전 중에는 h_ref가 차체 yaw를 따라가지 못하게 고정 */
#define CENTER_AXIS_RESNAP_DEG     25.0f   /* ★레그축 재스냅 진입각: 양벽 균형·저요레이트로 '벽은 직진'인데 h_ref와
                                              이만큼 이상 어긋나면 기준 자체가 틀린 것(코너 이벤트 없이 지난 45° 굽이,
                                              과회전 잔재). 정상 제어 편차(HDG_SLOW 14°, SWEEP 14°)보다 충분히 크게 */
#define CENTER_AXIS_RESNAP_YAW_DPS 12.0f   /* 재스냅 판정 중 허용 yaw-rate. 회전 과도 상태에서는 판정하지 않는다 */
#define CENTER_AXIS_RESNAP_N       12U     /* 연속 확인 루프 수(~0.3-0.6s). 코너 탈출 과도/단발 튐 오스냅 차단 */
#define CENTER_SIDE_PAIR_MAX_CM    58.0f   /* 코스 최대폭 67cm - 차폭 16cm = 51cm + 여유. 초과면 한쪽 값 튐/개구부로 본다 */
#define CENTER_SINGLE_MAX_CM       32.0f   /* 한쪽 벽 추종에 사용할 최대 거리. 그 이상은 heading-hold로 처리 */
#define CENTER_BODY_HALF_LEN_CM    13.5f   /* 차량 길이 27cm의 절반. 복도축 대비 yaw가 있으면 앞/뒤 모서리가
                                              측면센서 위치보다 벽으로 더 튀어나온다 */
#define CENTER_NEAR_GUARD_CM       10.1f   /* 정적 근접벽 보호선. 최협 중앙 여유 10.5cm보다 살짝 낮아 직선 중앙은 살리고,
                                              10cm 안쪽으로 붙으면 SIDE_AVOID 전 CRUISE에서 먼저 밀어낸다 */
#define CENTER_NEAR_GUARD_KP       3.0f    /* 근접벽 보호 조향 [%duty/cm]. IMG_2982의 벽 평행접촉을 줄이기 위해 상향 */
#define CENTER_YAW_SWEEP_MAX_DEG   14.0f   /* corner sweep 계산에 쓸 heading 오차 상한. 이 이상은 이미 감속캡 영역 */
#define CENTER_YAW_SWEEP_GAIN      0.85f   /* half_len*sin(|hdg_err|) 중 보호선에 반영할 비율. 센서 위치/차폭 여유 보수 */
#define CENTER_GUARD_FULL_DEPTH_CM 2.0f    /* 보호선 안쪽으로 이만큼 들어오면 근접벽 속도 하한까지 낮춘다 */
#define CENTER_GUARD_MIN_SPEED_PCT 30.0f   /* 벽 접촉 임박 시 CRUISE 최저 속도. TT모터 스톨 하한과 같은 안전 크롤 */
#define CENTER_YAW_RATE_DEADBAND_DPS 4.0f  /* 자이로 미세 흔들림은 damping 입력으로 쓰지 않는다 */
#define CENTER_YAW_DAMP_MAX_PCT    9.0f    /* yaw-rate damping 상한. IMU 순간 변화가 CRUISE 조향을 포화시키지 못하게 함 */
#define CENTER_SETTLE_MS           220U    /* 코너/회피 직후 직선 복귀 안정화 시간. 코너를 늦춘 만큼 직선 복귀는 조금 당긴다 */
#define CENTER_SETTLE_SPEED_PCT    44.0f   /* 복귀 안정화 중 속도 상한 */
#define CENTER_MID_SETTLE_SPEED_PCT 48.0f  /* 50/55cm 레그 복귀 속도 상한 */
#define CENTER_WIDE_SETTLE_SPEED_PCT 52.0f /* 60/67cm 레그 복귀 속도 상한 */
#define CENTER_HDG_FAST_DEG        4.0f    /* 이 heading 오차 안에서는 최고속 허용 */
#define CENTER_HDG_SLOW_DEG        14.0f   /* 이 heading 오차 이상이면 직진축 복구를 위해 감속.
                                              기하 근거: 14° 편차의 횡접근 = 전진 10cm당 2.5cm → 최협 중앙여유 10.5cm 소진까지 전진 42cm
                                              = 제어루프(~10ms) 수십 회 여유. 12°를 위험 취급하던 과보수 캡이 코너 탈출 크롤의 공범 */
#define CENTER_HDG_MIN_SPEED_PCT   40.0f   /* heading 오차가 큰 상태의 속도 상한. 너무 낮추면 직진 복귀 전에 속도가 죽는다 */
#define CENTER_SENSOR_MAX_CM       80U     /* Clamp side sensor spikes */
#define COURSE_CAR_WIDTH_CM        16.0f   /* testtrack.drawio 폭 판정용 차량 폭 */
#define COURSE_NARROW_MAX_CM       47.0f   /* drawio 37/43/45cm 레그 */
#define COURSE_WIDE_MIN_CM         58.0f   /* drawio 60/67cm 레그. 50/55cm는 중간 레그 */
#define CORNER_WIDE_OPEN_CM        34U     /* 넓은 직선에서 단순 오프센터를 코너로 보지 않기 위한 side-open 기준 */
#define CORNER_WIDE_ASYM_OPEN_CM   36U
#define CORNER_WIDE_ASYM_CM        16U

/* 차체 기준: 약 27cm x 16cm. testtrack 최협 37cm에서는 중앙 기준 측면 여유가 약 10.5cm뿐이다. */

/* ---- 내부 상태 (MotorTask 단독 소유 — 단일 태스크 문맥이라 비volatile/뮤텍스 불요) ----
 * dbg.state 미러는 Drive_Update 말미에서 단일 태스크가 복사 → volatile 불필요 */
static DriveState st = DS_CRUISE;
static uint32_t   t_state  = 0;    /* 현재 상태 진입 시각 */
static uint8_t    turn_dir = 0;    /* 0=좌 1=우 */
static uint8_t    swap_cnt = 0;    /* 이번 SPIN에서 방향 flip 횟수 */
static uint8_t    rev_cnt  = 0;    /* CRUISE 복귀 없이 누적된 후진 chunk (CRUISE 진입 시 리셋) */
static uint8_t    clear_cnt = 0;   /* 정면 트임 연속(유효 측정) 카운트 */
static uint8_t    front_near_cnt = 0; /* [req1] 그레이징 아닌 정면근접 연속 카운트 — BRAKE 확정 게이트 */
static uint8_t    corner_near_cnt = 0; /* 코너 후보 연속 확인 — 직선 그레이징/ToF 튐으로 아크 진입 방지 */
static uint8_t    grid_clear_cnt = 0;  /* 아크 중 전방 그리드탈출선(52cm) 연속 트임 — 45° 코너 조기탈출 게이트 */
static uint8_t    side_clear_cnt = 0;  /* SIDE_AVOID 탈출 연속 확인 — 회피/복귀 핑퐁 방지 */
static uint8_t    launching = 1U;  /* [fix] 자율 시작 직후 런치 직진 커밋 활성 (시작점 역주행 봉쇄) */
static uint32_t   t_launch  = 0;   /* [fix] 런치 시작 시각(첫 cruise 프레임에 래치, 0=미래치) */
static uint32_t   t_spin0   = 0;   /* [fix] SPIN 블라인드 시간컷 기준. 방향 재래치 시 갱신 */
static float      h_entry  = 0.0f; /* SPIN 진입 heading (회전각 게이트/미러 기준) */
static uint8_t    h_entry_valid = 0U; /* h_entry가 신선한 IMU 프레임에서 래치됐는지 */
static float      h_ref    = 0.0f; /* heading-hold 기준 */
static float      h_stuck  = 0.0f; /* 스턱 감지 윈도 시작 heading */
static uint32_t   t_stuck  = 0;
/* 센터링 제어 컨텍스트 */
typedef struct {
    uint8_t  fresh;       /* 구 center_reset: 첫 루프 heading 래치 트리거 */
    uint8_t  imu_prev_live; /* 직전 제어 프레임에서 heading이 신선했는지 */
    uint8_t  lp_valid;    /* median 후 LPF 초기화 여부 */
    uint8_t  error_valid; /* 양벽 센터링 D항 이력이 유효한지 */
    uint8_t  axis_resnap_cnt; /* 레그축 재스냅 연속 확인 카운트 */
    float    prev_error;  /* 초음파 D항 직전 오차 */
    uint32_t prev_ms;     /* 미분 dt 기준 시각 */
    float    steer_prev;  /* slew 연속성용 직전 조향량 */
    float    l_lp;
    float    r_lp;
} CenteringContext;
static CenteringContext ctx_center = { .fresh = 1U };

typedef struct {
    float    integ;
    float    prev_prog;
    uint8_t  prev_valid;
    uint32_t prev_ms;
} TurnPidContext;
static TurnPidContext ctx_turn;

static float      h_prev   = 0.0f; /* yaw-rate D항: 직전 heading */
/* [req4] 역주행 절대방위 앵커: 시작 시 최초 1회 절대 래치 + 코너(cruise_enter)마다 현재 leg로 재래치 */
static float      course_heading = 0.0f;
static float      course_zero    = 0.0f; /* 시작 방위 = 45° 그리드 원점. 모든 레그축 = course_zero + 45°×k */
static uint8_t    is_start_latch = 0;
/* DS_CORNER(아크) 부활하되 상태는 무상태(h_entry/clear_cnt 재사용) — 구 역주행-앵커
 * (course_heading/course_valid/rev_latched/t_corner_exit)는 불요라 미복원 */

/* 임의 각도차 → (-180, 180]. 입력은 0~360 값들의 차라 루프는 1~2회로 종료 */
/* req3: while→단일 분기 + static inline. 입력은 0~360 heading 차(범위 (-360,360))라
 * 1회 보정으로 (-180,180] 보장 — 호출 오버헤드 제거, 수치 동일 */
static inline float wrap180(float a)
{
    if      (a >   180.0f) a -= 360.0f;
    else if (a <= -180.0f) a += 360.0f;
    return a;
}

static inline float wrap360(float a)
{
    if      (a >= 360.0f) a -= 360.0f;
    else if (a <    0.0f) a += 360.0f;
    return a;
}

/* 45° 코스 그리드 스냅: h에서 가장 가까운 (course_zero + 45°×k) 축을 반환.
 * testtrack 레그는 전부 시작방위 기준 45° 배수축이므로, 물리 heading이 실제 레그축 ±22.5° 안이면
 * 스냅 결과가 정확한 레그축이다 — 코너 진입 대각(δ), 45° 코너 ±90 오스냅, 흔들리는 차체각 학습을
 * 전부 무효화한다. 시작 래치 전(is_start_latch=0)에는 그리드 원점이 없어 h 그대로 통과 */
static float course_grid_snap(float h)
{
    if (!is_start_latch) return h;
    float k = roundf(wrap180(h - course_zero) / 45.0f);
    return wrap360(course_zero + (k * 45.0f));
}

static float turn_progress_deg(float heading)
{
    float d = wrap180(heading - h_entry);   /* heading은 CW+ */
    return (turn_dir == 0U) ? -d : d;       /* 좌회전(CCW)은 음의 heading 변화가 정상 진행 */
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float deadbandf(float v, float band)
{
    if (v > band)  return v - band;
    if (v < -band) return v + band;
    return 0.0f;
}

static float forward_drive_floor(float pct)
{
    if (pct > 0.0f && pct < (float)MOTOR_MIN_PCT)
        return (float)MOTOR_MIN_PCT;
    return pct;
}

static uint16_t center_sanitize_cm(uint16_t cm)
{
    if (cm == 0U) return CENTER_SENSOR_MAX_CM;
    if (cm > CENTER_SENSOR_MAX_CM) return CENTER_SENSOR_MAX_CM;
    return cm;
}

typedef enum
{
    CW_UNKNOWN = 0,
    CW_NARROW,
    CW_MID,
    CW_WIDE
} CorridorWidthClass;

static CorridorWidthClass corridor_class_from_width(float corridor_width_cm)
{
    if (corridor_width_cm <= COURSE_NARROW_MAX_CM) return CW_NARROW;
    if (corridor_width_cm >= COURSE_WIDE_MIN_CM)   return CW_WIDE;
    return CW_MID;
}

static float corridor_width_from_sides(float left_cm, float right_cm)
{
    return left_cm + right_cm + COURSE_CAR_WIDTH_CM;
}

/* 센터링 컨텍스트 리셋: LPF/이력/slew 0, fresh=1로 다음 루프 heading 재래치 예약.
 * heading은 in이 없는 Drive_Init에서도 부르므로 여기서 안 만짐 — fresh 블록이 latch */
static void Centering_Reset(CenteringContext *c)
{
    c->prev_error = 0.0f;
    c->prev_ms    = 0U;
    c->steer_prev = 0.0f;
    c->lp_valid   = 0U;
    c->error_valid = 0U;
    c->axis_resnap_cnt = 0U;
    c->l_lp       = 0.0f;
    c->r_lp       = 0.0f;
    c->imu_prev_live = 0U;
    c->fresh      = 1U;
}

static void TurnPid_Reset(void)
{
    ctx_turn.integ = 0.0f;
    ctx_turn.prev_prog = 0.0f;
    ctx_turn.prev_valid = 0U;
    ctx_turn.prev_ms = 0U;
}

static void TurnPid_Run(const DriveInputs *in, float turn_prog)
{
    float dt = 0.02f;
    if (ctx_turn.prev_ms != 0U && in->now > ctx_turn.prev_ms)
        dt = clampf((float)(in->now - ctx_turn.prev_ms) / 1000.0f, 0.005f, 0.100f);

    float err = TURN_TARGET_DEG - turn_prog;
    if (err < 0.0f) err = 0.0f;

    if (err < 45.0f)
    {
        ctx_turn.integ += err * dt;
        ctx_turn.integ = clampf(ctx_turn.integ, -TURN_PID_I_MAX, TURN_PID_I_MAX);
    }
    else
    {
        ctx_turn.integ = 0.0f;   /* 큰 각도에서는 I항이 과회전 에너지로 쌓이지 않게 한다 */
    }

    float rate = (ctx_turn.prev_valid) ? (turn_prog - ctx_turn.prev_prog) / dt : 0.0f;
    float cmd = (TURN_PID_KP * err) + (TURN_PID_KI * ctx_turn.integ) - (TURN_PID_KD * rate);
    float min_cmd = (err <= TURN_PID_FINE_DEG) ? (float)TURN_PID_FINE_MIN_PCT : (float)TURN_PID_MIN_PCT;
    cmd = clampf(cmd, min_cmd, (float)TURN_SPEED);

    uint8_t outer = (uint8_t)(cmd + 0.5f);
    uint8_t inner = (uint8_t)((cmd * TURN_PID_INNER_RATIO) + 0.5f);
    if (inner > TURN_INNER) inner = TURN_INNER;

    if (turn_dir == 0U) Car_PivotLeft(outer, inner);
    else                Car_PivotRight(outer, inner);

    ctx_turn.prev_prog = turn_prog;
    ctx_turn.prev_valid = 1U;
    ctx_turn.prev_ms = in->now;
    dbg.duty_l = (turn_dir == 0U) ? -(int16_t)inner : (int16_t)outer;
    dbg.duty_r = (turn_dir == 0U) ?  (int16_t)outer : -(int16_t)inner;
}

static void Drive_CenteringPD_Run(const DriveInputs *in)
{
    if (ctx_center.fresh)
    {
        /* Centering_Reset가 이력을 0으로 둠 → IMU가 신선할 때만 heading 기준 래치.
           기준은 흔들리는 차체각이 아니라 45° 그리드축 — 대각 자세가 기준화되지 않는다 */
        if (in->imu_live)
        {
            h_ref  = course_grid_snap(in->heading);
            h_prev = in->heading;
            ctx_center.imu_prev_live = 1U;
        }
        ctx_center.fresh = 0U;
    }

    if (in->imu_live && !ctx_center.imu_prev_live)
    {
        /* IMU 사망/실패 프레임 뒤 첫 신선한 heading은 기준과 yaw 미분 이력을 다시 잡는다.
         * 오래된 heading과 새 heading을 한 루프 dt로 나눠 생기는 yaw spike를 차단. */
        h_ref  = course_grid_snap(in->heading);
        h_prev = in->heading;
    }

    float left_cm  = (float)center_sanitize_cm(in->l);
    float right_cm = (float)center_sanitize_cm(in->r);

    if (!ctx_center.lp_valid)
    {
        ctx_center.l_lp = left_cm;
        ctx_center.r_lp = right_cm;
        ctx_center.lp_valid = 1U;
    }
    else
    {
        ctx_center.l_lp = (ctx_center.l_lp * (1.0f - CENTER_LPF_ALPHA)) + (left_cm * CENTER_LPF_ALPHA);
        ctx_center.r_lp = (ctx_center.r_lp * (1.0f - CENTER_LPF_ALPHA)) + (right_cm * CENTER_LPF_ALPHA);
    }
    left_cm  = ctx_center.l_lp;
    right_cm = ctx_center.r_lp;

    float error = left_cm - right_cm;
    uint8_t left_seen  = (left_cm  < (float)CENTER_SENSOR_MAX_CM);
    uint8_t right_seen = (right_cm < (float)CENTER_SENSOR_MAX_CM);
    uint8_t side_pair_valid = (uint8_t)(left_seen && right_seen &&
        ((left_cm + right_cm) <= CENTER_SIDE_PAIR_MAX_CM));
    float corridor_width_cm = side_pair_valid ? corridor_width_from_sides(left_cm, right_cm) : 0.0f;
    CorridorWidthClass corridor_class = side_pair_valid
                                      ? corridor_class_from_width(corridor_width_cm)
                                      : CW_UNKNOWN;
    uint8_t front_straight_open = (uint8_t)(in->f_valid && in->f >= FRONT_ARC_CM);
    uint8_t left_track = left_seen;
    uint8_t right_track = right_seen;

    if (!side_pair_valid && left_seen && right_seen)
    {
        /* 좌우 합이 코스 물리폭을 벗어나면 한쪽은 개구부/경사 반사로 본다.
           조향은 가까운 벽만 추종하고, 방향 안정성은 BNO055 heading-hold가 맡는다. */
        if (left_cm <= right_cm) right_track = 0U;
        else                     left_track  = 0U;
    }
    if (left_track  && left_cm  > CENTER_SINGLE_MAX_CM) left_track  = 0U;
    if (right_track && right_cm > CENTER_SINGLE_MAX_CM) right_track = 0U;

    /* CRUISE는 복도 센터링만 담당한다. 코너는 cruise_run의 DS_CORNER 아크가 별도로 처리한다. */
    float steer_max = CENTER_STEER_MAX_PCT;

    /* ---- 초음파 측정 미분: 데드존 무관 매 루프 연속 미분 (derivative-kick 차단; prev_* 연속 갱신) ---- */
    float dt = 0.02f;
    if (ctx_center.prev_ms != 0U && in->now > ctx_center.prev_ms)
        dt = clampf((float)(in->now - ctx_center.prev_ms) / 1000.0f, 0.005f, 0.100f);

    float d_error = (side_pair_valid && ctx_center.error_valid && ctx_center.prev_ms != 0U)
                  ? (error - ctx_center.prev_error) / dt
                  : 0.0f;

    /* ---- 차체 각속도(IMU 자이로 적분기반): wrap180(heading-h_prev)/dt, CW(우)=+ (dbg.yaw_rate와 동일 부호) ---- */
    float yaw_rate = (in->imu_live && ctx_center.imu_prev_live && ctx_center.prev_ms != 0U)
                   ? wrap180(in->heading - h_prev) / dt
                   : 0.0f;

    /* 상태 연속 갱신. 양벽이 끊긴 구간의 MAX 센티넬은 다음 양벽 복귀 때 D항 킥으로 쓰지 않는다. */
    ctx_center.prev_error  = error;
    ctx_center.error_valid = side_pair_valid;
    ctx_center.prev_ms    = in->now;
    if (in->imu_live) h_prev = in->heading;
    ctx_center.imu_prev_live = in->imu_live;

    /* ---- 모드 선택: 양벽 보이면 센터링, 한쪽 벽만 보이면 단일벽 추종, 무벽이면 heading-hold ----
       center_sanitize_cm가 무에코/스파이크(0 또는 >MAX)를 MAX로 클램프 → 'MAX 미만'이 곧 '벽 보임' */
    float   steer;
    uint8_t mode;
    float   hdg_err = in->imu_live ? wrap180(in->heading - h_ref) : 0.0f;
    float   hdg_ctrl_err = in->imu_live ? deadbandf(hdg_err, CENTER_HDG_DEADBAND_DEG) : 0.0f;
    float   left_guard_cm = CENTER_NEAR_GUARD_CM;
    float   right_guard_cm = CENTER_NEAR_GUARD_CM;
    float   guard_risk_cm = 0.0f;
    float   center_kp = CENTER_KP;
    float   center_kd = CENTER_KD;
    float   both_hdg_blend = CENTER_BOTH_HDG_BLEND;
    float   straight_hdg_blend = CENTER_STRAIGHT_HDG_BLEND;
    float   fast_speed = CENTER_STRAIGHT_FAST_SPEED_PCT;
    float   fast_err_cm = CENTER_STRAIGHT_FAST_ERR_CM;
    float   fast_hdg_deg = CENTER_STRAIGHT_FAST_HDG_DEG;
    float   href_align_err_cm = CENTER_HREF_ALIGN_ERR_CM;
    float   settle_speed_cap = CENTER_SETTLE_SPEED_PCT;
    float   hdg_fast_deg = CENTER_HDG_FAST_DEG;
    float   hdg_min_speed = CENTER_HDG_MIN_SPEED_PCT;

    if (corridor_class == CW_NARROW)
    {
        center_kp = CENTER_NARROW_KP;
        both_hdg_blend = CENTER_NARROW_HDG_BLEND;
        straight_hdg_blend = CENTER_BOTH_HDG_BLEND;
        fast_speed = CENTER_NARROW_FAST_SPEED_PCT;
        fast_err_cm = 3.0f;
        fast_hdg_deg = 4.0f;
    }
    else if (corridor_class == CW_WIDE)
    {
        center_kp = CENTER_WIDE_KP;
        both_hdg_blend = CENTER_WIDE_HDG_BLEND;
        straight_hdg_blend = CENTER_WIDE_STRAIGHT_HDG_BLEND;
        fast_speed = CENTER_WIDE_FAST_SPEED_PCT;
        fast_err_cm = 7.0f;
        fast_hdg_deg = 6.0f;
        href_align_err_cm = 6.0f;
        settle_speed_cap = CENTER_WIDE_SETTLE_SPEED_PCT;
        hdg_fast_deg = 6.0f;
        hdg_min_speed = 42.0f;
    }
    else if (corridor_class == CW_MID)
    {
        settle_speed_cap = CENTER_MID_SETTLE_SPEED_PCT;
        hdg_min_speed = 38.0f;
    }
    if (front_straight_open && side_pair_valid)
    {
        center_kp = CENTER_STRAIGHT_SIDE_KP;
        center_kd = CENTER_STRAIGHT_SIDE_KD;
        both_hdg_blend = CENTER_STRAIGHT_HDG_BLEND;
        straight_hdg_blend = CENTER_STRAIGHT_HDG_BLEND;
        fast_err_cm = CENTER_STRAIGHT_FAST_ERR_CM;
        fast_hdg_deg = CENTER_STRAIGHT_FAST_HDG_DEG;
        hdg_fast_deg = CENTER_STRAIGHT_FAST_HDG_DEG;
        if (hdg_min_speed < 44.0f)
            hdg_min_speed = 44.0f;
    }

    if (in->imu_live)
    {
        float yaw_mag = fabsf(hdg_err);
        if (yaw_mag > CENTER_YAW_SWEEP_MAX_DEG)
            yaw_mag = CENTER_YAW_SWEEP_MAX_DEG;

        /* 차체 길이 27cm라 yaw가 있으면 앞/뒤 모서리가 half_len*sin(yaw)만큼 벽으로 더 접근한다.
           heading=CW+ 기준: hdg_err>0이면 우측 벽을 향하고, hdg_err<0이면 좌측 벽을 향한다. */
        float yaw_sweep_cm = CENTER_BODY_HALF_LEN_CM
                            * sinf(yaw_mag * 0.01745329f)
                            * CENTER_YAW_SWEEP_GAIN;
        if (hdg_err > CENTER_HDG_DEADBAND_DEG)
            right_guard_cm += yaw_sweep_cm;
        else if (hdg_err < -CENTER_HDG_DEADBAND_DEG)
            left_guard_cm += yaw_sweep_cm;
    }

    if (side_pair_valid)
    {
        /* 양벽 센터링: 좌우 거리를 계속 같게 맞추되, 중앙 근처는 약한 P로만 보정한다.
           기존 hard-zero 데드존은 중앙에서 조향을 완전히 놓았다가 벽에 가까워진 뒤 반대로 미는
           패턴을 만들 수 있어 IMG_2980 직선 헌팅에 불리했다. */
        float abs_error = fabsf(error);
        float center_error;
        if (abs_error <= CENTER_DEADZONE_CM)
        {
            center_error = error * CENTER_INNER_KP_SCALE;
        }
        else
        {
            float shaped = (abs_error - CENTER_DEADZONE_CM)
                         + (CENTER_DEADZONE_CM * CENTER_INNER_KP_SCALE);
            center_error = (error >= 0.0f) ? shaped : -shaped;
        }

        mode  = SM_BOTH;
        steer = (center_kp * center_error) + (center_kd * d_error);

        if (in->imu_live)
        {
            float hdg_blend = both_hdg_blend;
            if (front_straight_open && abs_error <= CENTER_STRAIGHT_HDG_GATE_CM)
            {
                float boost_gate = 1.0f;
                if (abs_error > CENTER_DEADZONE_CM)
                {
                    boost_gate = 1.0f - ((abs_error - CENTER_DEADZONE_CM) /
                        (CENTER_STRAIGHT_HDG_GATE_CM - CENTER_DEADZONE_CM));
                    boost_gate = clampf(boost_gate, 0.0f, 1.0f);
                }
                hdg_blend += (straight_hdg_blend - both_hdg_blend) * boost_gate;
            }
            if (abs_error > CENTER_DEADZONE_CM)
            {
                float gate = (abs_error - CENTER_DEADZONE_CM) /
                             (CENTER_BOTH_HDG_GATE_CM - CENTER_DEADZONE_CM);
                gate = clampf(gate, 0.0f, 1.0f);
                hdg_blend *= 1.0f - ((1.0f - CENTER_BOTH_HDG_MIN_SCALE) * gate);
            }
            steer += KP_HDG * hdg_blend * hdg_ctrl_err;
        }

        if (!front_straight_open
            && in->imu_live
            && fabsf(error) <= href_align_err_cm
            && fabsf(hdg_err) <= CENTER_HREF_ALIGN_HDG_DEG
            && fabsf(d_error) <= CENTER_HREF_ALIGN_DERR_CMS
            && fabsf(yaw_rate) <= CENTER_HREF_ALIGN_YAW_DPS)
        {
            h_ref += wrap180(in->heading - h_ref) * CENTER_HREF_BLEND;
            h_ref = wrap360(h_ref);
        }

        /* ★레그축 재스냅: 양벽이 '균형+안정'(|l-r| 작고 변화율/요레이트 낮음)이면 차체는 실제 복도축과
           평행 주행 중이다. 그런데 h_ref와 25° 이상 어긋난 상태가 지속되면 기준 쪽이 틀린 것 —
           DS_CORNER 이벤트 없이 통과한 45° 굽이(경사벽 specular로 front 미감지)나 과회전 잔재가 원인.
           이때 현재 heading(≈실제 레그축)의 그리드 스냅으로 h_ref/course_heading을 교체해 회복한다.
           구 코드는 CENTER_HREF_ALIGN_HDG_DEG(8°) 게이트 탓에 45° 오염 기준을 영구 유지했다 */
        if (in->imu_live
            && fabsf(error) <= (CENTER_HREF_ALIGN_ERR_CM + 1.0f)
            && fabsf(d_error) <= CENTER_HREF_ALIGN_DERR_CMS
            && fabsf(yaw_rate) <= CENTER_AXIS_RESNAP_YAW_DPS
            && fabsf(hdg_err) >= CENTER_AXIS_RESNAP_DEG)
        {
            if (ctx_center.axis_resnap_cnt < 255U) ctx_center.axis_resnap_cnt++;
            if (ctx_center.axis_resnap_cnt >= CENTER_AXIS_RESNAP_N)
            {
                h_ref = course_grid_snap(in->heading);
                course_heading = h_ref;
                ctx_center.axis_resnap_cnt = 0U;
            }
        }
        else
        {
            ctx_center.axis_resnap_cnt = 0U;
        }
    }
    else if (left_track || right_track)
    {
        float wall_steer = left_track
                         ? CENTER_SINGLE_KP * (left_cm - CENTER_SINGLE_TARGET_CM)
                         : CENTER_SINGLE_KP * (CENTER_SINGLE_TARGET_CM - right_cm);
        /* ★반발 전용(흡인 금지): 벽이 목표보다 멀 때 벽 '쪽으로' 조향하던 흡인항 제거.
           코스 축은 IMU heading-hold가 잡는다 — 벽 추종 흡인은 37cm 레그서 한쪽 소나 순간 드랍 시
           반대측 여유를 5cm까지 압축할 수 있다(37 − 차폭16 − 목표16). 가까울 때 밀어내기만 남긴다. */
        if (left_track  && wall_steer > 0.0f) wall_steer = 0.0f;   /* 좌벽 흡인(+) 차단 */
        if (!left_track && wall_steer < 0.0f) wall_steer = 0.0f;   /* 우벽 흡인(−) 차단 */
        float hdg_steer = in->imu_live
                        ? (KP_HDG * CENTER_SINGLE_HDG_BLEND * hdg_ctrl_err)
                        : 0.0f;
        mode  = SM_SINGLE;
        steer = wall_steer + hdg_steer;
    }
    else if (in->imu_live)
    {
        /* 무벽 + IMU → heading-hold.
           부호: steer>0=좌(CCW), heading=CW+ → heading>h_ref(우로 밀림)면 steer>0(좌로 복귀) = +KP_HDG*(heading-h_ref) */
        mode  = SM_HDG;
        steer = KP_HDG * hdg_ctrl_err;
    }
    else
    {
        /* 벽 상실 + IMU 사망 → 기준 없음 → open-loop 직진 */
        mode  = SM_OPEN;
        steer = 0.0f;
    }

    /* ---- 측벽 반발 (단일벽/무벽 모드 전용, 깊이 제곱 램프) ----
       ★양벽(SM_BOTH)에서는 반발 미적용. 근거: 37cm 레그는 중앙에서도 양쪽 여유 10.5cm < SOFT(11)라
       양측 반발이 '상시 동시' 작동 — 좌우 반발 합력은 net ≈ 0.9%/cm(중심 오프셋)의 병렬 P 경로가 되어
       데드존을 우회하고 D항 없이 흐른다(감쇠 없는 고게인 P + 센서 위상지연 = 리밋사이클,
       IMG_2975/2976 직선 와리가리·벽 접촉의 구조적 원인). 양벽 모드는 데드존+D+yaw댐핑을 갖춘
       센터링 PD가 단독 소유하고, SIDE_AVOID_CM 미만 위급은 DS_SIDE_AVOID가 별도 계층에서 방어한다.
       단일벽/무벽 모드는 pair 정보가 없어 반발 유지 — depth²/(SOFT−HARD) 램프로 진입부 완만,
       HARD(8cm) 위험선에서 선형과 동일 조향력(1.35×3=4.05%). */
    if (!side_pair_valid)
    {
        float repel_span = CENTER_SIDE_SOFT_CM - CENTER_SIDE_HARD_CM;
        if (left_track && left_cm < CENTER_SIDE_SOFT_CM)
        {
            float depth = CENTER_SIDE_SOFT_CM - left_cm;
            steer -= CENTER_SIDE_REPEL_KP * depth * depth / repel_span;
        }
        if (right_track && right_cm < CENTER_SIDE_SOFT_CM)
        {
            float depth = CENTER_SIDE_SOFT_CM - right_cm;
            steer += CENTER_SIDE_REPEL_KP * depth * depth / repel_span;
        }
    }

    /* ---- 차체 footprint 근접벽 가드: 양벽/단일벽 공통, 위험도가 큰 한쪽 벽만 반대로 밀어낸다.
       정적 거리뿐 아니라 yaw로 생기는 앞/뒤 모서리 스윕을 보호선에 더한다. 37cm 최협 중앙(10.5cm)은
       yaw가 작으면 건드리지 않고, 코너 탈출처럼 차체가 벽을 향할 때만 더 일찍 개입한다. */
    if (left_track || right_track)
    {
        float left_risk = left_track ? (left_guard_cm - left_cm) : -1000.0f;
        float right_risk = right_track ? (right_guard_cm - right_cm) : -1000.0f;

        if (left_risk > 0.0f || right_risk > 0.0f)
        {
            if (left_risk >= right_risk)
            {
                guard_risk_cm = left_risk;
                steer -= CENTER_NEAR_GUARD_KP * left_risk;
            }
            else
            {
                guard_risk_cm = right_risk;
                steer += CENTER_NEAR_GUARD_KP * right_risk;
            }
        }
    }

    /* ---- Yaw-rate 댐핑 (음의 피드백): 모드 무관, imu_live일 때만 ----
       steer>0=좌(CCW)→heading 감소(CW+). 실제 회전 yaw_rate(CW+)를 상쇄하려면 동부호로 더해야 음의 피드백:
       yaw_rate>0(우로 회전 중)→ steer += (좌로 보정). ∴ 이 레포 부호계에선 '+= KD_YAW*yaw_rate'가 댐핑('-'는 발산).
       노이즈 큰 초음파 D(CENTER_KD) 비중↓, 자이로가 즉각 감쇠 담당 */
    if (in->imu_live)
    {
        float yaw_ctrl = deadbandf(yaw_rate, CENTER_YAW_RATE_DEADBAND_DPS);
        steer += clampf(KD_YAW * yaw_ctrl, -CENTER_YAW_DAMP_MAX_PCT, CENTER_YAW_DAMP_MAX_PCT);
    }

    steer = clampf(steer, -steer_max, steer_max);

    /* 조향 변화율 제한(slew): 루프당 ±CENTER_STEER_SLEW_PCT 이내로만 변함 → 좌우 급반전(흔들림) 억제.
       데드밴드로 0이 될 때도 급강하 대신 서서히 복귀 → 직진/좁은길 부드러움 */
    {
        steer = clampf(steer, ctx_center.steer_prev - CENTER_STEER_SLEW_PCT, ctx_center.steer_prev + CENTER_STEER_SLEW_PCT);
        ctx_center.steer_prev = steer;
    }

    /* 속도: 조향량/정면거리/측벽거리/heading 오차 기반 제한기.
       평상시에는 20초 목표용 빠른 base를 쓰고, 위험 신호가 있을 때만 상한을 낮춘다. */
    float speed_top = CENTER_BASE_SPEED_PCT;
    if (side_pair_valid && in->f_valid && in->f >= CENTER_FRONT_FAST_CM && guard_risk_cm <= 0.0f)
    {
        float abs_error = fabsf(error);
        float abs_hdg = in->imu_live ? fabsf(hdg_err) : 0.0f;
        if (abs_error <= fast_err_cm &&
            (!in->imu_live || abs_hdg <= fast_hdg_deg))
        {
            speed_top = fast_speed;
        }
    }

    float steer_ratio = fabsf(steer) / steer_max;   /* steer_max는 CENTER_STEER_MAX_PCT */
    float base = speed_top - ((speed_top - CENTER_MIN_SPEED_PCT) * steer_ratio);
    if (in->f_valid)
    {
        float f_cm = (float)in->f;
        float front_cap = speed_top;

        if (f_cm <= CENTER_FRONT_SLOW_CM)
        {
            front_cap = CENTER_FRONT_MIN_SPEED_PCT;
        }
        else if (f_cm < CENTER_FRONT_FAST_CM)
        {
            float ratio = (f_cm - CENTER_FRONT_SLOW_CM) / (CENTER_FRONT_FAST_CM - CENTER_FRONT_SLOW_CM);
            front_cap = CENTER_FRONT_MIN_SPEED_PCT
                      + ((speed_top - CENTER_FRONT_MIN_SPEED_PCT) * ratio);
        }

        if (base > front_cap)
            base = front_cap;
    }
    if (left_track || right_track)
    {
        float side_min = left_cm < right_cm ? left_cm : right_cm;
        float side_cap = speed_top;

        if (side_min <= CENTER_SIDE_HARD_CM)
        {
            side_cap = CENTER_SIDE_MIN_SPEED_PCT;
        }
        else if (side_min < CENTER_SIDE_SOFT_CM)
        {
            float ratio = (side_min - CENTER_SIDE_HARD_CM) / (CENTER_SIDE_SOFT_CM - CENTER_SIDE_HARD_CM);
            side_cap = CENTER_SIDE_MIN_SPEED_PCT
                     + ((speed_top - CENTER_SIDE_MIN_SPEED_PCT) * ratio);
        }

        if (base > side_cap)
            base = side_cap;

        if (guard_risk_cm > 0.0f)
        {
            float ratio = guard_risk_cm / CENTER_GUARD_FULL_DEPTH_CM;
            ratio = clampf(ratio, 0.0f, 1.0f);
            float guard_cap = speed_top
                            - ((speed_top - CENTER_GUARD_MIN_SPEED_PCT) * ratio);
            if (base > guard_cap)
                base = guard_cap;
        }
    }
    if (in->imu_live)
    {
        float abs_hdg = fabsf(hdg_err);
        float hdg_cap = speed_top;

        if (abs_hdg >= CENTER_HDG_SLOW_DEG)
        {
            hdg_cap = hdg_min_speed;
        }
        else if (abs_hdg > hdg_fast_deg)
        {
            float ratio = (abs_hdg - hdg_fast_deg) / (CENTER_HDG_SLOW_DEG - hdg_fast_deg);
            hdg_cap = speed_top - ((speed_top - hdg_min_speed) * ratio);
        }

        if (base > hdg_cap)
            base = hdg_cap;
    }
    if ((in->now - t_state) < CENTER_SETTLE_MS && base > settle_speed_cap)
        base = settle_speed_cap;
    base = clampf(base, CENTER_SIDE_MIN_SPEED_PCT, speed_top);

    float lt = clampf(base - steer + (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);   /* 전진 주행은 floor 적용. +trim=좌바퀴↑(좌쏠림 보정) */
    float rt = clampf(base + steer - (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
    lt = forward_drive_floor(lt);
    rt = forward_drive_floor(rt);

    Motor_Left((int8_t)(lt + 0.5f));
    Motor_Right((int8_t)(rt + 0.5f));

    dbg.steer      = steer;
    dbg.steer_mode = mode;
    dbg.yaw_rate   = yaw_rate;
    dbg.hdg_err    = hdg_err;
    dbg.duty_l     = (int16_t)(lt + 0.5f);
    dbg.duty_r     = (int16_t)(rt + 0.5f);
}

/* ---- 상태 진입 헬퍼 (진입 액션 + 타임스탬프) ---- */

static void enter(DriveState s, uint32_t now)
{
    st = s;
    t_state = now;
}

static void cruise_enter(const DriveInputs *in)
{
    Centering_Reset(&ctx_center);
    uint8_t from_turn  = (uint8_t)((st == DS_SPIN || st == DS_CORNER) && h_entry_valid && in->imu_live);
    /* ★SIDE_AVOID 탈출은 회피 아크로 틀어진 '현재' heading이 아니라 leg 축(course_heading)으로 복귀.
       현재 heading을 래치하면 복도를 대각으로 달리면서도 hdg_err=0(기준 자체가 대각)이라
       heading 감속캡·복귀 조향 둘 다 침묵 → 반대벽 직행 → 재회피 핑퐁 (IMG_2976 벽 접촉 증가 원인).
       course_heading은 시작 래치 + 코너 탈출마다 leg 정방향으로 갱신되므로 항상 유효한 복도축. */
    uint8_t from_avoid = (uint8_t)(st == DS_SIDE_AVOID && in->imu_live && is_start_latch);
    /* ★그리드 스냅: 구 turn_locked(h_entry±90)는 (a) 코너 진입 대각 δ를 다음 레그 기준에 유전시키고
       (b) 45° 코너를 90°로 오스냅해 기준을 45° 오염시켰다. 탈출 시점 물리 heading은 새 레그축 ±22.5°
       안이므로(90° exit A는 88°±δ, 45° 그리드 exit는 ±10° 정렬 게이트 통과 상태) 스냅이 정확한 축을 복원 */
    if      (from_turn)              h_ref = course_grid_snap(in->heading);
    else if (from_avoid)             h_ref = course_heading;
    else if (in->imu_live)           h_ref = course_grid_snap(in->heading);
    else                             h_ref = in->heading;
    course_heading = h_ref;   /* [req4] 코너 빠져나온 새 leg를 정방향으로 재래치(다중 코너 데드락 방지) */
    if (from_turn || from_avoid)
    {
        /* 다음 CRUISE 첫 루프가 현재 대각 heading으로 h_ref를 덮어쓰지 못하게 한다. */
        ctx_center.fresh = 0U;
        ctx_center.imu_prev_live = 1U;
    }
    rev_cnt    = 0;
    front_near_cnt = 0;
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    h_prev     = in->heading;   /* yaw-rate 리셋: 회피 복귀 직후 stale heading차로 스파이크 D항 방지 */
    enter(DS_CRUISE, in->now);
}

static void brake_enter(const DriveInputs *in)
{
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    if (BRAKE_MS > 0) Car_Brake();   /* BRAKE_MS=0 → 레거시 코스트 폴백 */
    else              Car_Stop();
    enter(DS_BRAKE, in->now);
}

/* tie_flip: 좌우 우열이 SIDE_HYST 미만(동률)일 때 직전 방향 유지(0) vs 반대로(1).
 * 후진을 거쳐 들어오면 반대쪽을 시도하는 게 포켓 탈출에 유리 */
static void spin_enter(const DriveInputs *in, uint8_t tie_flip)
{
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    if      (in->l > in->r + SIDE_HYST) turn_dir = 0;   /* 왼쪽이 더 트임 → 좌회전 */
    else if (in->r > in->l + SIDE_HYST) turn_dir = 1;
    else if (tie_flip)                  turn_dir ^= 1;
    swap_cnt  = 0;
    clear_cnt = 0;
    h_entry = in->heading;
    h_stuck = in->heading;
    h_entry_valid = in->imu_live;
    t_stuck = in->now;
    t_spin0 = in->now;   /* 블라인드 시간컷 기준. 방향 재래치 시 새 회전으로 다시 잡는다 */
    TurnPid_Reset();
    enter(DS_SPIN, in->now);
}

static void reverse_enter(const DriveInputs *in)
{
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    Car_Backward(DRIVE_SPEED);
    if (rev_cnt < 255) rev_cnt++;
    enter(DS_REVERSE, in->now);
}

static void hold_enter(const DriveInputs *in)
{
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    Car_Stop();   /* 정지 상태 유지엔 코스트로 충분 */
    enter(DS_HOLD, in->now);
}

/* 측면 박기직전: 더 가까운(박힌) 쪽 반대로 회전하도록 래치 후 진입.
 * l<r=좌벽 박힘→우회전(turn_dir 1), r<l=우벽 박힘→좌회전(0), 동률=직전 유지 */
static void side_avoid_enter(const DriveInputs *in)
{
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    if      (in->l < in->r) turn_dir = 1;   /* 좌벽 가까움 → 우회전으로 멀어짐 */
    else if (in->r < in->l) turn_dir = 0;   /* 우벽 가까움 → 좌회전 */
    enter(DS_SIDE_AVOID, in->now);
}

/* [req1] 전면 근접값(f<TURN)이 '측벽 빔 그레이징(좁은 복도 빔 간섭)'으로 설명되는지 판정.
 *
 * 기하 모델: 전방 초음파 빔은 반각 θ_b의 원뿔. 측방거리 w(=차체 옆 벽까지)의 벽면이 이 원뿔에
 * 처음 잡히는 지점의 슬랜트(빗변) 거리는 R_min = w / sin(θ_b) (전방축거리 x=w/tanθ_b에서
 * R=√(x²+w²)=w/sinθ_b). 즉 측벽 단독 반사로는 R_min '이상'만 측정 가능 →
 *   · f ≥ w_near/sin(θ_b)  : 근접 측벽 그레이징으로 설명 가능 = 가짜(정면벽 아님) 의심
 *   · f <  w_near/sin(θ_b) : 어떤 측벽으로도 못 만드는 더 가까운 실제 표적 = 진짜 정면벽
 * 부동소수 나눗셈 회피 위해 'w_near ≤ f·sin(θ_b)'로 변형(동치).
 *
 * IMU 교차검증: 차체가 복도축(h_ref)에서 FRONT_OFFAXIS_DEG 넘게 틀어지면 전방빔 중심선이
 * 축을 벗어나 측벽을 직격 → 그 자체로 그레이징 의심(기하 게이트 보완). imu_live일 때만. */
static uint8_t front_graze_suspected(const DriveInputs *in)
{
    uint16_t w_near = (in->l < in->r) ? in->l : in->r;
    uint16_t w_far  = (in->l < in->r) ? in->r : in->l;
    float corridor_width_cm = (float)in->l + (float)in->r + COURSE_CAR_WIDTH_CM;
    CorridorWidthClass corridor_class = corridor_class_from_width(corridor_width_cm);
    uint16_t asym_cm = (corridor_class == CW_WIDE) ? CORNER_WIDE_ASYM_CM : CORNER_ASYM_CM;
    uint16_t asym_open_cm = (corridor_class == CW_WIDE) ? CORNER_WIDE_ASYM_OPEN_CM : CORNER_ASYM_OPEN_CM;

    /* 한쪽이 CORNER_ASYM_CM 이상 더 트여도, 근접측 여유가 최협 중앙여유보다 작으면
       실제 코너가 아니라 벽을 스치며 대각으로 바라보는 상태일 수 있다. 이때 asym 면제로
       CORNER에 들어가면 영상 21~25s처럼 아크가 벽을 계속 민다. */
    if (w_far >= (uint16_t)(w_near + asym_cm)
        && w_far >= asym_open_cm
        && w_near >= CORNER_NEAR_SAFE_CM)
    {
        if (in->imu_live && fabsf(wrap180(in->heading - h_ref)) > CORNER_ENTRY_HDG_MAX_DEG)
            return 1U;
        return 0U;
    }

    /* 기하 게이트: 근접 측벽이 현재 f를 그레이징으로 만들 수 있는 거리에 있는가 */
    if ((float)w_near <= (float)in->f * FRONT_BEAM_HALF_SIN)
        return 1U;

    /* Yaw-보정 슬랜트 게이트: 차체가 복도축 대비 φ만큼 틀어지면 빔 '축' 자체가 yaw쪽 벽으로
       기울어 그 벽의 최단 슬랜트 반환거리가 w/sin(θ_b+φ)로 줄어든다(고정 게이트는 φ=0 가정).
       sin(θ_b+φ) = sinθ_b·cosφ + cosθ_b·sinφ ⇒ w_yaw ≤ f·sin(θ_b+φ)면 yaw쪽 벽 빗면 반사로
       설명 가능 = 가짜. 진짜 정면벽(f < w_yaw/sin(θ_b+φ))은 계속 BRAKE 통과 → 안전 유지.
       와리가리 중 대각 빔이 벽을 잡고 f<TURN을 N회 확정 → 직선 한복판 급제동하던
       오작동 차단 (IMG_2975 '갑자기 멈춤'의 기하 원인). imu_live일 때만(φ 필요). */
    if (in->imu_live)
    {
        float phi_s = wrap180(in->heading - h_ref);   /* CW+ : >0 = 우측 yaw → 우벽을 향함 */
        float phi   = fabsf(phi_s);
        if (phi > 45.0f) phi = 45.0f;                 /* sin 증가 단조 구간으로 클램프 */
        float phi_rad   = phi * 0.01745329f;
        float cos_beam  = sqrtf(1.0f - (FRONT_BEAM_HALF_SIN * FRONT_BEAM_HALF_SIN));
        float sin_eff   = (FRONT_BEAM_HALF_SIN * cosf(phi_rad)) + (cos_beam * sinf(phi_rad));
        uint16_t w_yaw  = (phi_s > 0.0f) ? in->r : in->l;
        if ((float)w_yaw <= (float)in->f * sin_eff)
            return 1U;
    }

    /* IMU 게이트(백스톱): 복도축 대비 과도 yaw → 전방빔 축이탈 → 측벽 직격 의심 */
    if (in->imu_live && fabsf(wrap180(in->heading - h_ref)) > FRONT_OFFAXIS_DEG)
        return 1U;

    return 0U;   /* 두 게이트 모두 불성립 → 진짜 정면 장애물 */
}

/* [req2] 아크 코너 진입: 정지 없이 트인 쪽으로 방향 래치 + 진입 heading 기록 (전진성 유지) */
static void corner_enter(const DriveInputs *in)
{
    corner_near_cnt = 0;
    side_clear_cnt = 0;
    if      (in->l > in->r + SIDE_HYST) turn_dir = 0;   /* 좌측이 더 트임 → 좌아크 */
    else if (in->r > in->l + SIDE_HYST) turn_dir = 1;   /* 우측이 더 트임 → 우아크 */
    /* 동률(양쪽 트임)이면 직전 turn_dir 유지 */
    clear_cnt = 0;
    grid_clear_cnt = 0;
    h_entry   = in->heading;
    h_entry_valid = in->imu_live;
    enter(DS_CORNER, in->now);
}

/* ---- 상태별 실행 ---- */

/* CRUISE: 직진 센터링 주행 + 코너/장애물 전이. 우선순위(위→아래):
   1) 전 센서 무에코/무벽 → HOLD
   2) [req2] 코너(정면 좁아짐 + 한쪽 트임) → 전진성 아크(DS_CORNER) — '굴러서' 통과(대각 끼임 회피)
   3) [req1] 정면 장애물 → 그레이징 필터 통과 + N회 확정 시에만 BRAKE(→막다른곳 REVERSE / 아니면 SPIN 피벗 폴백)
   4) 측벽 박기직전 → SIDE_AVOID
   5) 센터링 PD */
static void cruise_run(const DriveInputs *in)
{
    /* 1) fail-safe: 정면 무에코만으로는 긴 직선의 '멀리 트임'일 수 있으므로 멈추지 않는다.
       측면까지 모두 MAX면 센서/트랙 상실로 보고 HOLD. */
    if (in->front_miss >= FRONT_FAIL_LIMIT
        && in->side_valid
        && in->l >= ULTRA_MAX_CM
        && in->r >= ULTRA_MAX_CM)
    {
        hold_enter(in);
        return;
    }

    /* 1.5) ★[fix] 런치 직진 커밋: 자율 시작 직후 LAUNCH_MS 동안 코너/스핀 판단을 억제하고 직진만 —
       (a) 모터 인러시 전류의 5V brownout/I2C 글리치로 imu_live가 죽는 창을 통과시키고(이때 스핀 각도컷이
           무력→180° 역주행 원인), (b) gyro heading 정착 + h_ref 기준을 그 사이 굳히며, (c) 시작점에서
       '돌 일' 자체를 없애 코스로 전진 진입시킨다. 단 정면 경고(f<TURN)나 측벽 위급이면 즉시 해제. */
    if (launching)
    {
        if (t_launch == 0U) t_launch = in->now;   /* 첫 cruise 프레임에 래치 */
        uint8_t window      = ((in->now - t_launch) < (uint32_t)LAUNCH_MS);
        uint8_t front_warn  = (in->f < FRONT_TURN_CM && in->f_valid);
        uint8_t side_emerg  = 0U;
        if (in->side_valid)
        {
            uint16_t smin_launch = (in->l < in->r) ? in->l : in->r;
            side_emerg = (smin_launch < CENTER_SIDE_HARD_CM);
        }
        if (window && !front_warn && !side_emerg)
        {
            if (in->imu_live)
            {
                h_ref  = course_grid_snap(in->heading);   /* 런치 datum = 시작 그리드축. gyro 수렴 흔들림이 기준에 안 들어간다 */
                h_prev = in->heading;
            }
            dbg.launch = 1;
            /* ★[fix①] 런치엔 벽센터링 호출 금지 — 시작점 brownout으로 imu_live=0이면 yaw댐핑 0 →
               P-only 센터링이 근접 측벽서 오버스티어→벽 핑퐁. 좌우 동일 base 듀티로 '순수 직진'만
               강제(측벽 무시). lt/rt는 ephemeral local(상태 아님). emerg(f<STOP)면 위서 즉시 해제 */
            float base = CENTER_SETTLE_SPEED_PCT;
            float lt = clampf(base + (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
            float rt = clampf(base - (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);
            Motor_Left ((int8_t)(lt + 0.5f));
            Motor_Right((int8_t)(rt + 0.5f));
            dbg.steer  = 0.0f;
            dbg.duty_l = (int16_t)(lt + 0.5f);
            dbg.duty_r = (int16_t)(rt + 0.5f);
            return;
        }
        launching  = 0U;            /* 윈도 종료 또는 안전 경고 → 런치 해제, 정상 우선순위 진행 */
        dbg.launch = 0;
    }

    /* 전면 근접 시 그레이징(측벽 빔 간섭) 여부를 1회 산정 — 코너 진입 게이트와 제동 게이트가 공용
       (중복 평가 방지 + dbg.graze 일관성). f<ARC 범위에서만 평가, 그 밖은 0 */
    uint8_t front_near = (in->f < FRONT_ARC_CM && in->f_valid);
    uint8_t graze      = (front_near && in->side_valid) ? front_graze_suspected(in) : 0U;
    dbg.graze = graze;

    if (in->f_valid && in->f < FRONT_STOP_CM && !graze)
    {
        brake_enter(in);
        return;
    }

    /* 2) ★코너 진입(아크 우선): 정면 ARC로 좁혀짐 + ★그레이징 아님(진짜 전방벽) + 한쪽 트임(회전로)이면
          전진성 아크로 통과. ★고속 횡쏠림 시 측벽 스침(가짜 f<ARC)을 '반대측 코너'로 오판해 벽으로 아크
          돌진하던 치명 오작동 차단 — 가짜면 코너 억제, 아래로 위임해 센터링 PD가 자세 교정 */
    if (front_near && in->side_valid && !graze)
    {
        float corridor_width_cm = (float)in->l + (float)in->r + COURSE_CAR_WIDTH_CM;
        CorridorWidthClass corridor_class = corridor_class_from_width(corridor_width_cm);
        uint16_t side_open_cm = (corridor_class == CW_WIDE) ? CORNER_WIDE_OPEN_CM : SIDE_OPEN_CM;
        uint16_t asym_open_cm = (corridor_class == CW_WIDE) ? CORNER_WIDE_ASYM_OPEN_CM : CORNER_ASYM_OPEN_CM;
        uint16_t asym_cm = (corridor_class == CW_WIDE) ? CORNER_WIDE_ASYM_CM : CORNER_ASYM_CM;

        uint8_t left_corner  = (in->l >= side_open_cm && in->r >= CORNER_NEAR_SAFE_CM);
        uint8_t right_corner = (in->r >= side_open_cm && in->l >= CORNER_NEAR_SAFE_CM);
        /* 절대 트임을 못 넘더라도, 더 먼 쪽이 충분히 크고 좌우 차가 뚜렷할 때만 완만 코너로 인정한다. */
        uint8_t left_asym = (in->l >= asym_open_cm &&
                             in->l >= (uint16_t)(in->r + asym_cm) &&
                             in->r >= CORNER_NEAR_SAFE_CM);
        uint8_t right_asym = (in->r >= asym_open_cm &&
                              in->r >= (uint16_t)(in->l + asym_cm) &&
                              in->l >= CORNER_NEAR_SAFE_CM);
        uint8_t corner_candidate = (uint8_t)((left_corner != right_corner) || left_asym || right_asym);
        if (corner_candidate)
        {
            if (corner_near_cnt < 255) corner_near_cnt++;
            if (corner_near_cnt >= CORNER_CONFIRM_N)
            {
                corner_near_cnt = 0U;
                corner_enter(in);
                return;
            }
        }
        else
        {
            corner_near_cnt = 0U;
        }
        /* 양측 다 벽 + 대칭 = 코너 아님(막힘/실표적) → 아래 정면 확정 게이트로 위임 */
    }
    else
    {
        corner_near_cnt = 0U;
    }

    /* 3) ★[req1] 정면 장애물 확정 게이트: f<TURN이라도 그레이징/축이탈로 설명되면 제동 보류(센터링이 교정) —
          그레이징 아닌 진짜 근접이 FRONT_CONFIRM_N회 연속일 때만 BRAKE. graze는 위에서 산정(f<TURN ⊂ f<ARC) */
    if (in->f < FRONT_TURN_CM && in->f_valid)
    {
        if (graze)
        {
            front_near_cnt = 0;   /* 가짜 의심 → 카운트 리셋, 센터링/측면회피에 위임 */
        }
        else if (front_near_cnt < 255 && ++front_near_cnt >= FRONT_CONFIRM_N)
        {
            dbg.front_near = front_near_cnt;
            brake_enter(in);      /* 진짜 정면벽 확정 → 능동 제동 → brake_run이 REVERSE/SPIN 분기 */
            return;
        }
    }
    else
    {
        front_near_cnt = 0;       /* 정면 트임 또는 무효 측정 → 리셋 */
    }
    dbg.front_near = front_near_cnt;

    if (!in->side_valid)
    {
        /* early front-only 프레임은 stale l/r로 코너/측면회피/센터링 판단을 하지 않는다.
         * 정면 비상만 즉시 처리하고, 그 외에는 직전 PWM을 유지해 멈칫거림을 줄인다. */
        if (in->f_valid && in->f < FRONT_STOP_CM)
            brake_enter(in);
        return;
    }

    /* 4) 측면 박기직전 → 비상 회피. 조향으론 못 막는 측벽 긁힘 차단 */
    uint16_t smin = (in->l < in->r) ? in->l : in->r;
    if (smin < SIDE_AVOID_CM) { side_avoid_enter(in); return; }

    /* 5) 센터링 PD 직진 주행 */
    Drive_CenteringPD_Run(in);
    return;
}

/* BRAKE: BRAKE_MS 동안 단락 제동 유지 → 해제 후 막다른곳이면 REVERSE, 아니면 SPIN */
static void brake_run(const DriveInputs *in)
{
    if ((in->now - t_state) < (uint32_t)BRAKE_MS) return;   /* 제동 유지 중 */

    Car_Stop();   /* 단락 제동 해제 */

    if (!in->side_valid)
        return;   /* 막다른길/회전방향 판단은 신선한 측면값이 올 때까지 보류 */

    uint8_t dead_end = (in->f < FRONT_DANGER_CM &&
                        in->l < SIDE_BLOCK_CM   &&
                        in->r < SIDE_BLOCK_CM);
    if (dead_end && rev_cnt < REV_MAX_CHUNKS)
        reverse_enter(in);
    else
        spin_enter(in, (rev_cnt > 0));   /* 후진을 거쳤으면 동률 시 반대쪽 시도 */
}

/* SPIN: 제자리 회전(전진 이동 0). 매 루프 센서 갱신하며 탈출각/스턱/타임아웃 감시 */
static void spin_run(const DriveInputs *in)
{
    float turn_prog = 0.0f;
    uint8_t turn_prog_valid = 0U;

    /* 막다른 곳 → 후진 (chunk 예산 내에서만) */
    if (in->side_valid
        && in->f < FRONT_DANGER_CM && in->l < SIDE_BLOCK_CM && in->r < SIDE_BLOCK_CM
        && rev_cnt < REV_MAX_CHUNKS)
    {
        reverse_enter(in);
        return;
    }

    if (in->imu_live)
    {
        if (!h_entry_valid)
        {
            h_entry = in->heading;
            h_stuck = in->heading;
            t_stuck = in->now;
            h_entry_valid = 1U;
            TurnPid_Reset();
        }
        turn_prog = turn_progress_deg(in->heading);
        turn_prog_valid = 1U;
        dbg.spin_deg = turn_prog;

        if ((in->now - t_state) >= (uint32_t)SPIN_COMMIT_MS && turn_prog < -TURN_WRONG_DEG)
        {
            turn_dir ^= 1U;
            swap_cnt++;
            clear_cnt = 0U;
            h_entry = in->heading;
            h_stuck = in->heading;
            t_stuck = in->now;
            t_state = in->now;
            t_spin0 = in->now;
            TurnPid_Reset();
            return;
        }

        /* 스턱: ROT_STUCK_MS 윈도 동안 회전 명령에도 heading 정체 → 바퀴 헛돎/걸림 → 후진 */
        if ((in->now - t_stuck) >= (uint32_t)ROT_STUCK_MS)
        {
            if (fabsf(wrap180(in->heading - h_stuck)) < ROT_STUCK_DEG
                && rev_cnt < REV_MAX_CHUNKS)
            {
                reverse_enter(in);
                return;
            }
            h_stuck = in->heading;
            t_stuck = in->now;
        }
    }

    /* 타임아웃 백스톱 (IMU 사망 시 스턱 커버): 후진 예산 있으면 후진, 없으면 방향 flip 후 새 회전으로 재시작 */
    if ((in->now - t_state) > (uint32_t)SPIN_MAX_MS)
    {
        if (rev_cnt < REV_MAX_CHUNKS) { reverse_enter(in); return; }
        turn_dir ^= 1;
        t_state = in->now;
        t_spin0 = in->now;
        clear_cnt = 0U;
        h_entry = in->heading;
        h_entry_valid = in->imu_live;
        TurnPid_Reset();
    }

    /* 방향 재평가: 최소유지(SPIN_COMMIT_MS) 후, 래치쪽 막힘 && 반대쪽 SIDE_HYST 이상 우세 → 1회 flip */
    if (in->side_valid && (in->now - t_state) >= (uint32_t)SPIN_COMMIT_MS && swap_cnt < SWAP_LIMIT)
    {
        uint16_t latched = (turn_dir == 0) ? in->l : in->r;
        uint16_t other   = (turn_dir == 0) ? in->r : in->l;
        if (latched < SIDE_BLOCK_CM && other > (uint16_t)(latched + SIDE_HYST))
        {
            turn_dir ^= 1;
            swap_cnt++;
            t_state = in->now;
            h_entry = in->heading;
            h_entry_valid = in->imu_live;
            TurnPid_Reset();
        }
    }

    /* [req4] 현재 leg 정방향(course_heading) 대비 편차 — 역방향(≥COURSE_REV_DEG)이면 cruise 탈출 금지.
       imu 사망 시 heading 무의미 → 게이트 비활성(fwd_ok=1)로 데드락 방지(블라인드 시간컷은 그대로 동작) */
    float course_dev = in->imu_live ? fabsf(wrap180(in->heading - course_heading)) : 0.0f;
    dbg.course_dev   = course_dev;
    dbg.reverse      = (in->imu_live && course_dev >= COURSE_REV_DEG) ? 1U : 0U;
    uint8_t fwd_ok   = (!in->imu_live || course_dev < COURSE_REV_DEG);   /* 정방향(또는 IMU사망)만 탈출 허용 */

    /* 탈출 → CRUISE: 정면 트임이 '유효 측정으로' CLEAR_CONFIRM회 연속
       + (IMU 시) 진입 대비 TURN_MIN_DEG 이상 회전 — 경사벽 specular 가짜-트임 조기탈출 차단
       + [req4] 정방향(fwd_ok) — 뒤가 뚫려도 역방향이면 탈출 금지 */
    if (!turn_prog_valid && in->imu_live && h_entry_valid)
    {
        turn_prog = turn_progress_deg(in->heading);
        turn_prog_valid = 1U;
    }

    if (in->f >= FRONT_CLEAR_CM && in->f_valid)
    {
        if (clear_cnt < 255) clear_cnt++;
        if (clear_cnt >= CLEAR_CONFIRM && fwd_ok &&
            (!in->imu_live || turn_prog >= TURN_MIN_DEG))
        {
            cruise_enter(in);
            return;
        }
    }
    else if (in->f < FRONT_CLEAR_CM)
    {
        clear_cnt = 0;   /* 무효 측정(f_valid=0)은 리셋도 가산도 안 함 — 직전 카운트 유지 */
    }

    /* ★피벗 회전각 완료 탈출(짧은 미로 복도 대응): 정면이 FRONT_CLEAR까지 안 트여도 IMU상 TURN_TARGET_DEG
       (~90°) 돌았고 현재 정면이 벽이 아님(≥FRONT_TURN_CM)이면 진행. 좁은 박스 미로서 정면이 멀리까지
       안 트이는 코너에서도 한 번만 깔끔히 꺾고 빠져나가게 함(과회전/제자리 헛돎 방지) */
    if (in->imu_live && h_entry_valid && in->f_valid && fwd_ok
        && turn_prog >= TURN_TARGET_DEG
        && in->f >= FRONT_TURN_CM)
    {
        cruise_enter(in);
        return;
    }

    /* ★[req1] 과회전/역주행 컷오프: TURN_TARGET 후에도 정면이 안 트여(f<FRONT_TURN) 스핀이 종료
       못 하면, 누적 회전각이 TURN_CUTOFF_DEG(>target) 초과 시 '정면 거리 조건을 무시'하고 1차 강제
       종료 → CRUISE. 좁은 미로서 90° 꺾어도 전면벽<FRONT_TURN이라 무한 스핀→180° 역주행하던 것 차단.
       복귀 후 cruise_run이 (측면 트임 있으면)아크 / (막힘이면)재제동으로 재평가 = 단일 연속회전의
       과회전만 끊고 판단은 메인 우선순위에 위임. imu_live일 때만(각도 필요) */
    if (in->imu_live && h_entry_valid && fwd_ok
        && turn_prog >= TURN_CUTOFF_DEG)
    {
        cruise_enter(in);
        return;
    }

    /* ★[fix] 블라인드 스핀 시간컷 (IMU 무관): 모터 인러시 I2C 글리치로 imu_live=0이 되면 위 각도
       컷오프가 죽어 무한 스핀→180° 역주행. 스핀 진입(t_spin0) 후 총 경과가 SPIN_BLIND_MS 초과면
       정면/IMU 무관 강제 종료 → CRUISE 재평가. 방향 재래치 시 t_spin0를 새로 잡는다 */
    if ((in->now - t_spin0) >= (uint32_t)SPIN_BLIND_MS && fwd_ok)
    {
        cruise_enter(in);
        return;
    }

    /* 회전 명령 (매 루프 재가): IMU가 신선하면 heading PID, 아니면 시간컷 기반 블라인드 피벗 */
    if (turn_prog_valid) TurnPid_Run(in, turn_prog);
    else if (turn_dir == 0) Car_PivotLeft(TURN_SPEED, TURN_INNER);
    else                    Car_PivotRight(TURN_SPEED, TURN_INNER);
}

/* [req2] CORNER: 전진성 아크로 코너 통과 (제자리 피벗 대신). 매 루프 센서 갱신하며 안전/탈출 감시.
   - 안전 1: 아크가 반경 안에서 못 돌고 정면 위급(<STOP) → BRAKE → (brake_run이) SPIN 피벗 폴백
   - 안전 2: 막다른 포켓(정면 danger + 양측 막힘) → BRAKE → REVERSE
   - 탈출 A(IMU): 진입 대비 TURN_TARGET_DEG 회전 + 정면이 벽 아님(≥TURN) → CRUISE
   - 탈출 B(폴백/IMU사망): 정면 CLEAR가 CLEAR_CONFIRM회 연속 → CRUISE
   - 타임아웃: ARC_MAX_MS 초과(개활지서 무한아크 방지) → BRAKE 폴백 */
static void corner_run(const DriveInputs *in)
{
    if (in->imu_live && !h_entry_valid)
    {
        h_entry = in->heading;
        h_entry_valid = 1U;
    }

    /* 안전 1: 아크 실패(정면 위급) → 정지 후 제자리 피벗으로 마무리.
       ★직선용 FRONT_STOP(28) 대신 아크 전용 CORNER_ABORT(16): 37×37 정션서 정상 아크도
       회전 중 전방빔이 외벽 ~16cm까지 스침 → 28 기준이 멀쩡한 아크를 SPIN으로 강등하던 것 차단 */
    if (in->f < CORNER_ABORT_CM && in->f_valid) { brake_enter(in); return; }

    /* 안전 2: 아크 중 측벽이 위험선 안으로 들어오면, 코너 판단을 유지하지 말고 벽 반대쪽 전진 아크로 이탈한다. */
    if (in->side_valid)
    {
        uint16_t smin = (in->l < in->r) ? in->l : in->r;
        if (smin < SIDE_AVOID_CM) { side_avoid_enter(in); return; }
    }

    /* 안전 3: 막다른 포켓 → 정지→후진(brake_run에서 dead_end 분기) */
    if (in->side_valid && in->f < FRONT_DANGER_CM && in->l < SIDE_BLOCK_CM && in->r < SIDE_BLOCK_CM)
    {
        brake_enter(in);
        return;
    }

    /* 타임아웃 백스톱 (IMU 사망 + 정면이 CLEAR까지 안 트이는 개활 곡선): 피벗으로 폴백 */
    if ((in->now - t_state) > (uint32_t)ARC_MAX_MS) { brake_enter(in); return; }

    float turn_prog = (in->imu_live && h_entry_valid) ? turn_progress_deg(in->heading) : 0.0f;

    if (in->imu_live && h_entry_valid
        && (in->now - t_state) >= (uint32_t)SPIN_COMMIT_MS
        && turn_prog < -TURN_WRONG_DEG)
    {
        brake_enter(in);
        return;
    }

    /* 탈출 A (IMU): 목표각 회전 완료 + 정면 벽 아님 → 직진 복귀 */
    if (in->imu_live && h_entry_valid && in->f_valid
        && turn_prog >= TURN_TARGET_DEG
        && in->f >= FRONT_TURN_CM)
    {
        cruise_enter(in);
        return;
    }

    /* ★탈출 A′ (45° 코너): testtrack에는 45R 코너가 2개 있는데 구 탈출 조건(A: 88°, B: 80°+CLEAR)은
       전부 90° 전제라 45° 코너에서 40°+ 과회전 → 다음 레그 내내 h_ref 45° 오염 + 벽 직행이 구조 강제였다.
       판별 기하: 90° 정션 중간(대각 45°)의 전방빔은 외벽 빗면 ~26..47cm(37..67cm 정션)를 벗어날 수 없고,
       진짜 45° 출구 정렬 시엔 새 레그 축방향 정반사로 ≥52cm가 나온다 → 52cm를 CLEAR_CONFIRM회 연속 +
       45° 그리드축 ±10° 정렬 + 회전각 35..65° 창일 때만 조기 탈출. 65° 초과는 90° 코너로 보고 A에 위임 */
    if (in->f_valid)
    {
        if (in->f >= CORNER_GRID_EXIT_CM) { if (grid_clear_cnt < 255U) grid_clear_cnt++; }
        else                              grid_clear_cnt = 0U;
    }
    if (in->imu_live && h_entry_valid
        && grid_clear_cnt >= CLEAR_CONFIRM
        && turn_prog >= CORNER_GRID_EXIT_MIN_DEG
        && turn_prog <= CORNER_GRID_EXIT_MAX_DEG
        && fabsf(wrap180(in->heading - course_grid_snap(in->heading))) <= CORNER_GRID_ALIGN_DEG)
    {
        cruise_enter(in);
        return;
    }

    /* 탈출 B (폴백/IMU 사망): 정면 트임이 유효 측정으로 CLEAR_CONFIRM회 연속 */
    if (in->f >= FRONT_CLEAR_CM && in->f_valid)
    {
        if (clear_cnt < 255) clear_cnt++;
        if (clear_cnt >= CLEAR_CONFIRM &&
            (!in->imu_live || !h_entry_valid || turn_prog >= CORNER_EXIT_MIN_DEG))
        {
            cruise_enter(in);
            return;
        }
    }
    else if (in->f < FRONT_CLEAR_CM)
    {
        clear_cnt = 0;   /* 무효 측정은 유지, 벽 재근접만 리셋 */
    }

    /* 아크 명령 (매 루프 재가): 양바퀴 전진, 트인 쪽으로 곡선 (전진성 = 대각 끼임 회피).
       목표각 근처에서는 계단식 감속 대신 선형 램프로 줄여 코너를 끊지 않고 부드럽게 빠져나간다. */
    uint8_t outer = ARC_OUTER;
    uint8_t inner = ARC_INNER;
    if (in->imu_live && h_entry_valid)
    {
        float remain = TURN_TARGET_DEG - turn_prog;
        if (remain < 0.0f) remain = 0.0f;
        if (remain <= ARC_APPROACH_DEG)
        {
            float k = 1.0f - (remain / ARC_APPROACH_DEG);
            outer = (uint8_t)((float)ARC_OUTER -
                (((float)ARC_OUTER - (float)ARC_APPROACH_OUTER) * k) + 0.5f);
            inner = (uint8_t)((float)ARC_INNER -
                (((float)ARC_INNER - (float)ARC_APPROACH_INNER) * k) + 0.5f);
        }
    }
    if (turn_dir == 0) Car_ArcLeft(outer, inner);
    else               Car_ArcRight(outer, inner);
}

/* REVERSE: 후진 chunk 1회 → 제동 → BRAKE가 재평가 (막다른곳 지속=추가 chunk, 아니면 SPIN) */
static void reverse_run(const DriveInputs *in)
{
    if ((in->now - t_state) >= (uint32_t)BACK_CHUNK_MS)
    {
        brake_enter(in);
        return;
    }
    Car_Backward(DRIVE_SPEED);
}

/* HOLD: 센서/트랙 상실 안전정지. 에코 회복 → 즉시 복귀, 장기화 → SPIN으로 specular 기하 탈출 */
static void hold_run(const DriveInputs *in)
{
    if (in->f_valid)
    {
        if (in->f < FRONT_STOP_CM) brake_enter(in);
        else                       cruise_enter(in);
        return;
    }
    if ((in->now - t_state) > (uint32_t)HOLD_MAX_MS)
    {
        spin_enter(in, 0);   /* 회전은 무이동이라 정면 블라인드 상태로도 안전 */
        return;
    }
    Car_Stop();
}

/* SIDE_AVOID: 측벽 박기직전 비상 회피. 박힌 쪽 반대로 전진 아크를 걸며 측면 트임/정면위급/타임아웃 감시 */
static void side_avoid_run(const DriveInputs *in)
{
    /* 정면 위급 우선 — 측면 피하다 정면 박는 것 방지 */
    if (in->f_valid && in->f < FRONT_STOP_CM) { brake_enter(in); return; }

    if (!in->side_valid)
    {
        Car_Stop();
        return;
    }

    /* 측면 트임(히스테리시스) → CRUISE 복귀. cruise_enter가 h_ref 갱신 = 새 진행방향 직진 */
    uint16_t smin = (in->l < in->r) ? in->l : in->r;
    if (smin >= SIDE_AVOID_CLEAR_CM)
    {
        if (side_clear_cnt < 255) side_clear_cnt++;
        if (side_clear_cnt >= SIDE_AVOID_CLEAR_CONFIRM)
        {
            cruise_enter(in);
            return;
        }
    }
    else
    {
        side_clear_cnt = 0U;
    }

    /* 전진 아크로도 못 빠지면 정지 후 재평가한다. 제자리 피벗 남발은 과회전과 벽 스침을 키운다. */
    if ((in->now - t_state) > (uint32_t)SIDE_AVOID_MAX_MS)
    {
        brake_enter(in);
        return;
    }

    /* 박힌 쪽 반대로 전진 아크. 후진/제자리 회전보다 차체 27cm 길이에서 벽을 덜 긁는다. */
    if (turn_dir == 0) Car_ArcLeft(SIDE_ESCAPE_OUTER, SIDE_ESCAPE_INNER);
    else               Car_ArcRight(SIDE_ESCAPE_OUTER, SIDE_ESCAPE_INNER);
}

/* ---- 공개 API ---- */

void Drive_Init(void)
{
    st         = DS_CRUISE;
    t_state    = 0;
    turn_dir   = 0;
    swap_cnt   = 0;
    rev_cnt    = 0;
    clear_cnt  = 0;
    front_near_cnt = 0;
    corner_near_cnt = 0;
    grid_clear_cnt = 0;
    side_clear_cnt = 0;
    Centering_Reset(&ctx_center);
    TurnPid_Reset();
    launching   = 1U;   /* [fix] (재)자율 진입마다 런치 직진 커밋 재무장 */
    t_launch    = 0;
    t_spin0     = 0;
    h_entry_valid = 0U;
    h_prev      = 0.0f;
    is_start_latch = 0;   /* [req4] 다음 imu_live 프레임에 course_heading 절대 래치 */
}

void Drive_Update(const DriveInputs *in)
{
    /* [req4] 시작 절대방위 1회 래치: IMU 신뢰 가능해진 최초 프레임 heading을 코스 정방향 기준으로.
       course_zero = 45° 그리드 원점 — 이후 모든 레그축은 course_grid_snap이 여기서 파생한다 */
    if (!is_start_latch && in->imu_live)
    {
        course_zero    = in->heading;
        course_heading = in->heading;
        is_start_latch = 1U;
    }

    switch (st)
    {
    case DS_CRUISE:  cruise_run(in);  break;
    case DS_BRAKE:   brake_run(in);   break;
    case DS_SPIN:    spin_run(in);    break;
    case DS_REVERSE: reverse_run(in); break;
    case DS_HOLD:    hold_run(in);    break;
    case DS_SIDE_AVOID: side_avoid_run(in); break;
    case DS_CORNER:  corner_run(in);  break;   /* [req2] 전진성 아크 코너 */
    default:         Car_Stop(); st = DS_CRUISE; break;
    }
    dbg.state    = (uint8_t)st;
    dbg.turn_dir = turn_dir;
    dbg.rev_cnt  = rev_cnt;
}
