/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drive.c
  * @brief   주행 제어기 구현 — 비블로킹 5-상태 머신 + 복도 센터링 조향
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

/* ---- 조향 모드 (dbg.steer_mode 인코딩; 1=좌벽/2=우벽은 한벽모드 폐지로 미발생) ---- */
#define SM_BOTH   0   /* 양벽 센터링: u = CENTER_KP × (l − r) */
#define SM_HDG    3   /* 무벽: IMU heading-hold */
#define SM_OPEN   4   /* 무벽 + IMU 사망: open-loop 직진 (레거시 거동) */

/* ---- Narrow-course centering control ---- */
#define CENTER_MA_WIN              3U      /* Moving average window. 1→3: 고주파 노이즈(초음파 raw 스파이크) 억제용 최소 MA 복원. side가 median(3) 거쳐도
                                              직진 시 D항에 raw 노이즈 유입→미분증폭 톱니진동 발생. MA(3)=차분노이즈 3×↓(분석 참조), 위상지연은 1샘플(~20ms)뿐 */
#define CENTER_DEADZONE_CM         3.0f    /* 중앙 불감대 ±3.0cm(2.5→3.0 영상 직진 weave→근중앙 limit cycle 추가 소거: 초음파 노이즈 플로어(~1.5cm) 위 게이트 — 율제한을 LPF로 악용 않고 노이즈는 데드존이 전담. 복원력 부족(중앙 못잡음) 시 ↓(2.5) */
#define CENTER_KP                  0.15f   /* Steering P gain [%duty/cm] = 폐루프 강성(ω_n²∝KP). 0.18→0.15: 헌팅 루프게인↓(오버슈트↓). 중앙추종 둔하면 ↑(0.18) */
#define CENTER_KD                  0.0f    /* Steering D gain [%duty/(cm/s)] (0.08→0: 초음파 미분 완전 제거 — 댐핑은 자이로 KD_YAW가 전담(클린). 노이즈 원천 차단.
                                              ⚠IMU 사망 폴백(imu_live=0)땐 댐핑 0 = P-only 더블적분기 = 헌팅 위험 → IMU-dead 보험 원하면 0.03 권장 */
#define CENTER_STEER_MAX_PCT       13.0f   /* Max differential steering command. 10→13: 완만곡선 센터링 추종 권한↑(직진은 오차 작아 영향 미미) */
#define CENTER_STEER_SLEW_PCT      4.0f    /* ★조향 변화율 상한 [%duty/loop] (3.0→4.0 헌팅 위상지연↓: 자이로 댐핑이 지연 없이 전달될 최소 액추에이터 대역폭 유지 — 율제한을 노이즈 LPF로 쓰면 비선형 위상지연으로 내부루프 무력화. 1.5로 조이지 말 것. */
#define CENTER_BASE_SPEED_PCT      44.0f   /* ★실제 직진 속도 [%duty] (46→44: 영상 벽충돌 → 트랙 ~10cm 측여유서 관성↓·센터링 추종여유↑. 트랙룰=벽충돌은 감속으로(임계확대 금지). MIN(43)보다 커야 함(불변식). 안정 후 단계 상향. 흔들리면 ↓(43) */
#define CENTER_MIN_SPEED_PCT       43.0f   /* 최대조향 시 속도 하한 (35→43: BASE 상향에 비례(×1.24). MOTOR_MIN_PCT(30) 위 유지(최대조향 lt=43−10=33). 정지 아님 */
#define CENTER_SENSOR_MAX_CM       80U     /* Clamp side sensor spikes */

/* ---- Corner anticipation 폐지 ----
 * 좁은 미로용 재설계: 아크(굴러서 통과) 코너 → 정지+제자리 피벗(brake→spin)으로 전환.
 * CORNER_NEAR_CM / CORNER_STEER_MAX_PCT(아크 블렌드 노브) 제거 — 센터링은 순수 직진 유지만 담당. */

/* ---- 내부 상태 (MotorTask 단독 소유 — 단일 태스크 문맥이라 비volatile/뮤텍스 불요) ----
 * dbg.state 미러는 Drive_Update 말미에서 단일 태스크가 복사 → volatile 불필요 */
static DriveState st = DS_CRUISE;
static uint32_t   t_state  = 0;    /* 현재 상태 진입 시각 */
static uint8_t    turn_dir = 0;    /* 0=좌 1=우 */
static uint8_t    swap_cnt = 0;    /* 이번 SPIN에서 방향 flip 횟수 */
static uint8_t    rev_cnt  = 0;    /* CRUISE 복귀 없이 누적된 후진 chunk (CRUISE 진입 시 리셋) */
static uint8_t    clear_cnt = 0;   /* 정면 트임 연속(유효 측정) 카운트 */
static uint8_t    front_near_cnt = 0; /* [req1] 그레이징 아닌 정면근접 연속 카운트 — BRAKE 확정 게이트 */
static uint8_t    launching = 1U;  /* [fix] 자율 시작 직후 런치 직진 커밋 활성 (시작점 역주행 봉쇄) */
static uint32_t   t_launch  = 0;   /* [fix] 런치 시작 시각(첫 cruise 프레임에 래치, 0=미래치) */
static uint32_t   t_spin0   = 0;   /* [fix] SPIN 진입 절대 시각 — 블라인드 시간컷용(swap에 리셋 안 됨) */
static float      h_entry  = 0.0f; /* SPIN 진입 heading (회전각 게이트/미러 기준) */
static uint8_t    h_entry_valid = 0U; /* h_entry가 신선한 IMU 프레임에서 래치됐는지 */
static float      h_ref    = 0.0f; /* heading-hold 기준 */
static float      h_stuck  = 0.0f; /* 스턱 감지 윈도 시작 heading */
static uint32_t   t_stuck  = 0;
/* 센터링 제어 컨텍스트 (req2: 구 Drive_CenteringPD_Run 함수-내 static들을 캡슐화) */
typedef struct {
    uint16_t l_buf[CENTER_MA_WIN];
    uint16_t r_buf[CENTER_MA_WIN];
    uint8_t  idx;
    uint8_t  count;
    uint8_t  fresh;       /* 구 center_reset: 첫 루프 heading 래치 트리거 */
    uint8_t  imu_prev_live; /* 직전 제어 프레임에서 heading이 신선했는지 */
    float    prev_error;  /* 초음파 D항 직전 오차 */
    uint32_t prev_ms;     /* 미분 dt 기준 시각 */
    float    steer_prev;  /* slew 연속성용 직전 조향량 */
} CenteringContext;
static CenteringContext ctx_center = { .fresh = 1U };
static float      h_prev   = 0.0f; /* yaw-rate D항: 직전 heading */
/* [req4] 역주행 절대방위 앵커: 시작 시 최초 1회 절대 래치 + 코너(cruise_enter)마다 현재 leg로 재래치 */
static float      course_heading = 0.0f;
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

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint16_t center_sanitize_cm(uint16_t cm)
{
    if (cm == 0U) return CENTER_SENSOR_MAX_CM;
    if (cm > CENTER_SENSOR_MAX_CM) return CENTER_SENSOR_MAX_CM;
    return cm;
}

static float center_avg_u16(const uint16_t *buf, uint8_t n)
{
    uint32_t sum = 0U;

    for (uint8_t i = 0U; i < n; i++)
        sum += buf[i];

    return (float)sum / (float)n;
}

/* 센터링 컨텍스트 리셋 (req2): 버퍼/이력/slew 0, fresh=1로 다음 루프 heading 재래치 예약.
 * heading은 in이 없는 Drive_Init에서도 부르므로 여기서 안 만짐 — fresh 블록이 latch */
static void Centering_Reset(CenteringContext *c)
{
    c->idx        = 0U;
    c->count      = 0U;
    c->prev_error = 0.0f;
    c->prev_ms    = 0U;
    c->steer_prev = 0.0f;
    c->imu_prev_live = 0U;
    c->fresh      = 1U;
}

static void Drive_CenteringPD_Run(const DriveInputs *in)
{
    if (ctx_center.fresh)
    {
        /* Centering_Reset가 버퍼/이력을 0으로 둠 → IMU가 신선할 때만 heading 기준 래치 */
        if (in->imu_live)
        {
            h_ref  = in->heading;
            h_prev = in->heading;
            ctx_center.imu_prev_live = 1U;
        }
        ctx_center.fresh = 0U;
    }

    if (in->imu_live && !ctx_center.imu_prev_live)
    {
        /* IMU 사망/실패 프레임 뒤 첫 신선한 heading은 기준과 yaw 미분 이력을 다시 잡는다.
         * 오래된 heading과 새 heading을 한 루프 dt로 나눠 생기는 yaw spike를 차단. */
        h_ref  = in->heading;
        h_prev = in->heading;
    }

    ctx_center.l_buf[ctx_center.idx] = center_sanitize_cm(in->l);
    ctx_center.r_buf[ctx_center.idx] = center_sanitize_cm(in->r);

    ctx_center.idx = (uint8_t)((ctx_center.idx + 1U) % CENTER_MA_WIN);
    if (ctx_center.count < CENTER_MA_WIN)
        ctx_center.count++;

    float left_cm  = center_avg_u16(ctx_center.l_buf, ctx_center.count);
    float right_cm = center_avg_u16(ctx_center.r_buf, ctx_center.count);
    float error = left_cm - right_cm;

    /* 아크 블렌드 폐지(피벗 주행): 조향 상한 고정 = 순수 센터링. 코너는 cruise_run이 brake→spin(피벗)으로 처리 */
    float steer_max = CENTER_STEER_MAX_PCT;

    /* ---- 초음파 측정 미분: 데드존 무관 매 루프 연속 미분 (derivative-kick 차단; prev_* 연속 갱신) ---- */
    float dt = 0.02f;
    if (ctx_center.prev_ms != 0U && in->now > ctx_center.prev_ms)
        dt = clampf((float)(in->now - ctx_center.prev_ms) / 1000.0f, 0.005f, 0.100f);

    float d_error = (ctx_center.prev_ms == 0U) ? 0.0f : (error - ctx_center.prev_error) / dt;  /* 첫 루프 → 0 */

    /* ---- 차체 각속도(IMU 자이로 적분기반): wrap180(heading-h_prev)/dt, CW(우)=+ (dbg.yaw_rate와 동일 부호) ---- */
    float yaw_rate = (in->imu_live && ctx_center.imu_prev_live && ctx_center.prev_ms != 0U)
                   ? wrap180(in->heading - h_prev) / dt
                   : 0.0f;

    /* 상태 연속 갱신 (미분 연속성) — 데드존/모드 무관하게 항상 */
    ctx_center.prev_error = error;
    ctx_center.prev_ms    = in->now;
    if (in->imu_live) h_prev = in->heading;
    ctx_center.imu_prev_live = in->imu_live;

    /* ---- 모드 선택: 양벽 보이면 센터링, 한쪽이라도 상실(무에코=센티넬)이면 heading-hold ----
       center_sanitize_cm가 무에코/스파이크(0 또는 >MAX)를 MAX로 클램프 → 'MAX 미만'이 곧 '벽 보임' */
    uint8_t left_seen  = (left_cm  < (float)CENTER_SENSOR_MAX_CM);
    uint8_t right_seen = (right_cm < (float)CENTER_SENSOR_MAX_CM);

    float   steer;
    uint8_t mode;

    if (left_seen && right_seen)
    {
        /* 양벽 센터링: 데드존은 출력단에서만 게이트(P+D=0), 미분 이력은 위에서 이미 갱신됨 */
        mode  = SM_BOTH;
        steer = (fabsf(error) <= CENTER_DEADZONE_CM)
              ? 0.0f
              : (CENTER_KP * error) + (CENTER_KD * d_error);
    }
    else if (in->imu_live)
    {
        /* 한쪽 벽 상실 → 초음파 오차 버리고 IMU heading-hold (벽 튐에 의한 급조향 차단).
           부호: steer>0=좌(CCW), heading=CW+ → heading>h_ref(우로 밀림)면 steer>0(좌로 복귀) = +KP_HDG*(heading-h_ref) */
        mode  = SM_HDG;
        steer = KP_HDG * wrap180(in->heading - h_ref);
    }
    else
    {
        /* 벽 상실 + IMU 사망 → 기준 없음 → open-loop 직진 */
        mode  = SM_OPEN;
        steer = 0.0f;
    }

    /* ---- Yaw-rate 댐핑 (음의 피드백): 모드 무관, imu_live일 때만 ----
       steer>0=좌(CCW)→heading 감소(CW+). 실제 회전 yaw_rate(CW+)를 상쇄하려면 동부호로 더해야 음의 피드백:
       yaw_rate>0(우로 회전 중)→ steer += (좌로 보정). ∴ 이 레포 부호계에선 '+= KD_YAW*yaw_rate'가 댐핑('-'는 발산).
       노이즈 큰 초음파 D(CENTER_KD) 비중↓, 자이로가 즉각 감쇠 담당 */
    if (in->imu_live)
        steer += KD_YAW * yaw_rate;

    steer = clampf(steer, -steer_max, steer_max);

    /* 조향 변화율 제한(slew): 루프당 ±CENTER_STEER_SLEW_PCT 이내로만 변함 → 좌우 급반전(흔들림) 억제.
       데드밴드로 0이 될 때도 급강하 대신 서서히 복귀 → 직진/좁은길 부드러움 */
    {
        steer = clampf(steer, ctx_center.steer_prev - CENTER_STEER_SLEW_PCT, ctx_center.steer_prev + CENTER_STEER_SLEW_PCT);
        ctx_center.steer_prev = steer;
    }

    /* 속도: 조향량 비례 감속만 (전방근접 fc 감속 폐지 — 코너 정지는 cruise_run의 brake가 담당) */
    float steer_ratio = fabsf(steer) / steer_max;   /* steer_max=10>0 */
    float base = CENTER_BASE_SPEED_PCT
               - ((CENTER_BASE_SPEED_PCT - CENTER_MIN_SPEED_PCT) * steer_ratio);
    base = clampf(base, CENTER_MIN_SPEED_PCT, CENTER_BASE_SPEED_PCT);

    float lt = clampf(base - steer + (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);   /* 안쪽 바닥 0 = 아크(역회전 없음). +trim=좌바퀴↑(좌쏠림 보정) */
    float rt = clampf(base + steer - (float)MOTOR_TRIM_PCT, 0.0f, 100.0f);

    Motor_Left((int8_t)(lt + 0.5f));
    Motor_Right((int8_t)(rt + 0.5f));

    dbg.steer      = steer;
    dbg.steer_mode = mode;
    dbg.yaw_rate   = yaw_rate;
    dbg.hdg_err    = wrap180(in->heading - h_ref);
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
    h_ref      = in->heading;   /* 무벽 구간 직진 기준 — 회피 직후의 새 진행방향 */
    course_heading = in->heading;   /* [req4] 코너 빠져나온 새 leg를 정방향으로 재래치(다중 코너 데드락 방지) */
    Centering_Reset(&ctx_center);
    rev_cnt    = 0;
    front_near_cnt = 0;
    h_prev     = in->heading;   /* yaw-rate 리셋: 회피 복귀 직후 stale heading차로 스파이크 D항 방지 */
    enter(DS_CRUISE, in->now);
}

static void brake_enter(const DriveInputs *in)
{
    if (BRAKE_MS > 0) Car_Brake();   /* BRAKE_MS=0 → 레거시 코스트 폴백 */
    else              Car_Stop();
    enter(DS_BRAKE, in->now);
}

/* tie_flip: 좌우 우열이 SIDE_HYST 미만(동률)일 때 직전 방향 유지(0) vs 반대로(1).
 * 후진을 거쳐 들어오면 반대쪽을 시도하는 게 포켓 탈출에 유리 */
static void spin_enter(const DriveInputs *in, uint8_t tie_flip)
{
    if      (in->l > in->r + SIDE_HYST) turn_dir = 0;   /* 왼쪽이 더 트임 → 좌회전 */
    else if (in->r > in->l + SIDE_HYST) turn_dir = 1;
    else if (tie_flip)                  turn_dir ^= 1;
    swap_cnt  = 0;
    clear_cnt = 0;
    h_entry = in->heading;
    h_stuck = in->heading;
    h_entry_valid = in->imu_live;
    t_stuck = in->now;
    t_spin0 = in->now;   /* [fix] 블라인드 시간컷 기준(총 스핀 시간 — swap에도 유지) */
    enter(DS_SPIN, in->now);
}

/* (corner_enter 삭제 — 아크 코너 폐지. 코너 회전 방향 래치는 spin_enter가 담당) */

static void reverse_enter(const DriveInputs *in)
{
    Car_Backward(DRIVE_SPEED);
    if (rev_cnt < 255) rev_cnt++;
    enter(DS_REVERSE, in->now);
}

static void hold_enter(const DriveInputs *in)
{
    Car_Stop();   /* 정지 상태 유지엔 코스트로 충분 */
    enter(DS_HOLD, in->now);
}

/* 측면 박기직전: 더 가까운(박힌) 쪽 반대로 회전하도록 래치 후 진입.
 * l<r=좌벽 박힘→우회전(turn_dir 1), r<l=우벽 박힘→좌회전(0), 동률=직전 유지 */
static void side_avoid_enter(const DriveInputs *in)
{
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

    /* ★[fix③] 한쪽이 CORNER_ASYM_CM 이상 더 트임 = 대칭 좁은복도가 아니라 곡선/코너 기하 →
       정면은 실제 코너벽(그레이징 아님). 정수 비교로 기하/IMU 게이트보다 먼저 단락(경량).
       대칭 근접(양벽 비슷)일 때만 아래 그레이징 판정 — 진짜 빔간섭은 그대로 검출 */
    if (w_far >= (uint16_t)(w_near + CORNER_ASYM_CM))
        return 0U;

    /* 기하 게이트: 근접 측벽이 현재 f를 그레이징으로 만들 수 있는 거리에 있는가 */
    if ((float)w_near <= (float)in->f * FRONT_BEAM_HALF_SIN)
        return 1U;

    /* IMU 게이트: 복도축 대비 과도 yaw → 전방빔 축이탈 → 측벽 직격 의심 */
    if (in->imu_live && fabsf(wrap180(in->heading - h_ref)) > FRONT_OFFAXIS_DEG)
        return 1U;

    return 0U;   /* 두 게이트 모두 불성립 → 진짜 정면 장애물 */
}

/* [req2] 아크 코너 진입: 정지 없이 트인 쪽으로 방향 래치 + 진입 heading 기록 (전진성 유지) */
static void corner_enter(const DriveInputs *in)
{
    if      (in->l > in->r + SIDE_HYST) turn_dir = 0;   /* 좌측이 더 트임 → 좌아크 */
    else if (in->r > in->l + SIDE_HYST) turn_dir = 1;   /* 우측이 더 트임 → 우아크 */
    /* 동률(양쪽 트임)이면 직전 turn_dir 유지 */
    clear_cnt = 0;
    h_entry   = in->heading;
    h_entry_valid = in->imu_live;
    enter(DS_CORNER, in->now);
}

/* ---- 상태별 실행 ---- */

/* CRUISE: 직진 센터링 주행 + 코너/장애물 전이. 우선순위(위→아래):
   1) 정면 무에코 → HOLD
   2) [req2] 코너(정면 좁아짐 + 한쪽 트임) → 전진성 아크(DS_CORNER) — '굴러서' 통과(대각 끼임 회피)
   3) [req1] 정면 장애물 → 그레이징 필터 통과 + N회 확정 시에만 BRAKE(→막다른곳 REVERSE / 아니면 SPIN 피벗 폴백)
   4) 측벽 박기직전 → SIDE_AVOID
   5) 센터링 PD */
static void cruise_run(const DriveInputs *in)
{
    /* 1) fail-safe: 정면 연속 무에코 → HOLD. CRUISE에서만 — SPIN/REVERSE는 전방 노출이 없어 면제 */
    if (in->front_miss >= FRONT_FAIL_LIMIT) { hold_enter(in); return; }

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
            side_emerg = (smin_launch < SIDE_AVOID_CM);
        }
        if (window && !front_warn && !side_emerg)
        {
            if (in->imu_live)
            {
                h_ref  = in->heading;   /* gyro 수렴 중 datum 지속 갱신 → 런치 종료 시 깨끗한 직진 기준 확보 */
                h_prev = in->heading;
            }
            dbg.launch = 1;
            /* ★[fix①] 런치엔 벽센터링 호출 금지 — 시작점 brownout으로 imu_live=0이면 yaw댐핑 0 →
               P-only 센터링이 근접 측벽서 오버스티어→벽 핑퐁. 좌우 동일 base 듀티로 '순수 직진'만
               강제(측벽 무시). lt/rt는 ephemeral local(상태 아님). emerg(f<STOP)면 위서 즉시 해제 */
            float base = CENTER_BASE_SPEED_PCT;
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

    /* 2) ★코너 진입(아크 우선): 정면 ARC로 좁혀짐 + ★그레이징 아님(진짜 전방벽) + 한쪽 트임(회전로)이면
          전진성 아크로 통과. ★고속 횡쏠림 시 측벽 스침(가짜 f<ARC)을 '반대측 코너'로 오판해 벽으로 아크
          돌진하던 치명 오작동 차단 — 가짜면 코너 억제, 아래로 위임해 센터링 PD가 자세 교정 */
    if (front_near && in->side_valid && !graze)
    {
        uint8_t left_open  = (in->l >= SIDE_OPEN_CM);
        uint8_t right_open = (in->r >= SIDE_OPEN_CM);
        /* ★[fix③] 절대 트임 미달이라도 좌우 비대칭(완만곡선)이면 더 트인 쪽으로 아크 진입
           (corner_enter가 l>r+SIDE_HYST로 방향 래치 → 트인 쪽 곡선) */
        uint8_t asym = (in->l >= (uint16_t)(in->r + CORNER_ASYM_CM)) ||
                       (in->r >= (uint16_t)(in->l + CORNER_ASYM_CM));
        if (left_open || right_open || asym) { corner_enter(in); return; }
        /* 양측 다 벽 + 대칭 = 코너 아님(막힘/실표적) → 아래 정면 확정 게이트로 위임 */
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
         * full 프레임이 곧 도착하므로 그 전까지는 안전하게 코스트 정지. */
        Car_Stop();
        return;
    }

    /* 4) 측면 박기직전 → 비상 회피(멈춤+피벗). 조향으론 못 막는 측벽 긁힘 차단 */
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
        }
        dbg.spin_deg = wrap180(in->heading - h_entry);

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

    /* 타임아웃 백스톱 (IMU 사망 시 스턱 커버): 후진 예산 있으면 후진, 없으면 방향 flip 후 계속 */
    if ((in->now - t_state) > (uint32_t)SPIN_MAX_MS)
    {
        if (rev_cnt < REV_MAX_CHUNKS) { reverse_enter(in); return; }
        turn_dir ^= 1;
        t_state = in->now;
        h_entry = in->heading;
        h_entry_valid = in->imu_live;
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
    if (in->f >= FRONT_CLEAR_CM && in->f_valid)
    {
        if (clear_cnt < 255) clear_cnt++;
        if (clear_cnt >= CLEAR_CONFIRM && fwd_ok &&
            (!in->imu_live || (h_entry_valid && fabsf(wrap180(in->heading - h_entry)) >= TURN_MIN_DEG)))
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
        && fabsf(wrap180(in->heading - h_entry)) >= TURN_TARGET_DEG
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
        && fabsf(wrap180(in->heading - h_entry)) >= TURN_CUTOFF_DEG)
    {
        cruise_enter(in);
        return;
    }

    /* ★[fix] 블라인드/총 스핀 시간컷 (IMU 무관): 모터 인러시 I2C 글리치로 imu_live=0이 되면 위 각도
       컷오프가 죽어 무한 스핀→180° 역주행. 스핀 진입(t_spin0) 후 총 경과가 SPIN_BLIND_MS 초과면
       정면/IMU 무관 강제 종료 → CRUISE 재평가. 각도컷의 시간 프록시(≈110°), swap 무관 총량 상한 */
    if ((in->now - t_spin0) >= (uint32_t)SPIN_BLIND_MS && fwd_ok)
    {
        cruise_enter(in);
        return;
    }

    /* 회전 명령 (매 루프 재가) */
    if (turn_dir == 0) Car_PivotLeft(TURN_SPEED, TURN_INNER);
    else               Car_PivotRight(TURN_SPEED, TURN_INNER);
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

    /* 안전 1: 아크 실패(정면 위급) → 정지 후 제자리 피벗으로 마무리 */
    if (in->f < FRONT_STOP_CM && in->f_valid) { brake_enter(in); return; }

    /* 안전 2: 막다른 포켓 → 정지→후진(brake_run에서 dead_end 분기) */
    if (in->side_valid && in->f < FRONT_DANGER_CM && in->l < SIDE_BLOCK_CM && in->r < SIDE_BLOCK_CM)
    {
        brake_enter(in);
        return;
    }

    /* 타임아웃 백스톱 (IMU 사망 + 정면이 CLEAR까지 안 트이는 개활 곡선): 피벗으로 폴백 */
    if ((in->now - t_state) > (uint32_t)ARC_MAX_MS) { brake_enter(in); return; }

    /* 탈출 A (IMU): 목표각 회전 완료 + 정면 벽 아님 → 직진 복귀 */
    if (in->imu_live && h_entry_valid && in->f_valid
        && fabsf(wrap180(in->heading - h_entry)) >= TURN_TARGET_DEG
        && in->f >= FRONT_TURN_CM)
    {
        cruise_enter(in);
        return;
    }

    /* 탈출 B (폴백/IMU 사망): 정면 트임이 유효 측정으로 CLEAR_CONFIRM회 연속 */
    if (in->f >= FRONT_CLEAR_CM && in->f_valid)
    {
        if (clear_cnt < 255) clear_cnt++;
        if (clear_cnt >= CLEAR_CONFIRM) { cruise_enter(in); return; }
    }
    else if (in->f < FRONT_CLEAR_CM)
    {
        clear_cnt = 0;   /* 무효 측정은 유지, 벽 재근접만 리셋 */
    }

    /* 아크 명령 (매 루프 재가): 양바퀴 전진, 트인 쪽으로 곡선 (전진성 = 대각 끼임 회피) */
    if (turn_dir == 0) Car_ArcLeft(ARC_OUTER, ARC_INNER);
    else               Car_ArcRight(ARC_OUTER, ARC_INNER);
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

/* HOLD: 정면 센서 상실 안전정지. 에코 회복 → 즉시 복귀, 장기화 → SPIN으로 specular 기하 탈출 */
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

/* SIDE_AVOID: 측벽 박기직전 비상 회피. 박힌 쪽 반대로 제자리 피벗하며 측면 트임/정면위급/타임아웃 감시 */
static void side_avoid_run(const DriveInputs *in)
{
    /* 정면 위급 우선 — 측면 피하다 정면 박는 것 방지 */
    if (in->f < FRONT_STOP_CM) { brake_enter(in); return; }

    /* 측면 트임(히스테리시스) → CRUISE 복귀. cruise_enter가 h_ref 갱신 = 새 진행방향 직진 */
    if (in->side_valid)
    {
        uint16_t smin = (in->l < in->r) ? in->l : in->r;
        if (smin >= SIDE_AVOID_CLEAR_CM) { cruise_enter(in); return; }
    }

    /* 한 방향으로 못 빠지면 방향 flip 후 재시도 (양쪽 좁은 끼임 백스톱) */
    if ((in->now - t_state) > (uint32_t)SIDE_AVOID_MAX_MS)
    {
        turn_dir ^= 1;
        t_state = in->now;
    }

    /* 박힌 쪽 반대로 제자리 피벗 (전진 0 — '멈추고 방향 잡기') */
    if (turn_dir == 0) Car_PivotLeft(TURN_SPEED, TURN_INNER);
    else               Car_PivotRight(TURN_SPEED, TURN_INNER);
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
    Centering_Reset(&ctx_center);
    launching   = 1U;   /* [fix] (재)자율 진입마다 런치 직진 커밋 재무장 */
    t_launch    = 0;
    t_spin0     = 0;
    h_entry_valid = 0U;
    h_prev      = 0.0f;
    is_start_latch = 0;   /* [req4] 다음 imu_live 프레임에 course_heading 절대 래치 */
}

void Drive_Update(const DriveInputs *in)
{
    /* [req4] 시작 절대방위 1회 래치: IMU 신뢰 가능해진 최초 프레임 heading을 코스 정방향 기준으로 */
    if (!is_start_latch && in->imu_live)
    {
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
