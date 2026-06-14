/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drive.c
  * @brief   주행 제어기 구현 — 비블로킹 5-상태 머신 + 복도 센터링 조향
  *
  *          설계 원칙 (벽 박힘 근절):
  *          - HAL_Delay 없음: 모든 유지시간은 HAL_GetTick 타임스탬프 → 매 루프 센서 갱신
  *          - 회피 중 명령된 전진 이동 0: arc 폐지, SPIN(제자리 회전) 전용
  *            (곡선 회피는 CRUISE의 비례 조향이 담당)
  *          - 모든 회전/정지에 피드백: 능동 제동(BRAKE), IMU 회전각 탈출 게이트,
  *            스턱 감지 — 시간×듀티 개루프 없음 → 배터리 sag 면역
  *          - IMU 사망 시 전 기능이 거리-only로 자동 강등 (주행 중단 없음)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "drive.h"
#include "motor.h"
#include <math.h>

/* ---- 디버그 미러 (SWD Live Expressions 등록 대상) ---- */
volatile uint8_t dbg_state      = 0;     /* DriveState (0 CRUISE / 1 BRAKE / 2 SPIN / 3 REVERSE / 4 HOLD).
                                            구 dbg_avoiding 대체: 0이 아니면 회피 계열 */
volatile uint8_t dbg_steer_mode = 0;     /* 0 양벽센터링 / 1 한벽(좌) / 2 한벽(우) / 3 heading-hold / 4 open-loop */
volatile float   dbg_steer      = 0.0f;  /* 조향량 u [%duty], + = 좌조향 */
volatile int16_t dbg_duty_l     = 0;     /* CRUISE 좌/우 바퀴 듀티 (데드밴드 (0,30) 침범 감시용) */
volatile int16_t dbg_duty_r     = 0;
volatile float   dbg_hdg_err    = 0.0f;  /* heading-hold 오차 [deg] */
volatile float   dbg_yaw_rate   = 0.0f;  /* CRUISE yaw-rate [deg/s], CW(우)=+. 직진 시 0 근처, 흔들리면 ±수십 */
volatile float   dbg_spin_deg   = 0.0f;  /* SPIN 진입 후 누적 회전각 [deg] (TURN_MIN_DEG 게이트 관찰) */
volatile uint8_t dbg_turn_dir   = 0;     /* 래치 회전 방향 0=좌 1=우 (구 main.c 미러 승계) */
volatile uint8_t dbg_rev_cnt    = 0;     /* 직진 복귀 없이 누적된 후진 chunk 수 */

/* ---- 조향 모드 ---- */
#define SM_BOTH   0   /* 양벽 센터링: u = KP_WALL × (l − r) */
#define SM_LEFT   1   /* 좌벽만: 진입 시 거리 래치 후 등거리 유지 */
#define SM_RIGHT  2   /* 우벽만 */
#define SM_HDG    3   /* 무벽: IMU heading-hold */
#define SM_OPEN   4   /* 무벽 + IMU 사망: open-loop 직진 (레거시 거동) */
#define SM_INIT   255 /* CRUISE 진입 직후 — 첫 루프에 모드 래치 강제 */

/* ---- Narrow-course centering control ---- */
#define CENTER_MA_WIN              5U      /* Moving average window: 3~5 recommended */
#define CENTER_DEADZONE_CM         2.0f    /* Force neutral steering inside +/-2 cm */
#define CENTER_KP                  0.80f   /* Steering P gain [%duty / cm] */
#define CENTER_KD                  0.035f  /* Steering D gain [%duty / (cm/s)] */
#define CENTER_STEER_MAX_PCT       10.0f   /* Max differential steering command */
#define CENTER_BASE_SPEED_PCT      45.0f   /* Straight driving speed (was 32, +40%) */
#define CENTER_MIN_SPEED_PCT       32.0f   /* Speed at max steering / corner apex (was 24; 정지 아님) */
#define CENTER_SENSOR_MAX_CM       80U     /* Clamp side sensor spikes */

#if (CENTER_MA_WIN < 3U) || (CENTER_MA_WIN > 5U)
#error "CENTER_MA_WIN must be 3~5"
#endif

/* ---- Corner anticipation (전방 근접 아크 블렌드) ----
 * 정면이 가까워질수록 조향 권한↑(약한 센터링→강한 아크) + 속도 테이퍼(min쪽) →
 * 90° 코너를 정지 없이 굴러서 통과. 못 돌 만큼 급하면 cruise_run의 brake→spin이 백업. */
#define CORNER_NEAR_CM         55.0f   /* 정면 이 거리부터 블렌드 시작. FRONT_STOP_CM(20)<이값, FRONT_CLEAR_CM(38)<이값 유지 */
#define CORNER_STEER_MAX_PCT   38.0f   /* apex(f≈FRONT_STOP)에서 조향 상한. 직진은 CENTER_STEER_MAX_PCT(10). apex 속도는 CENTER_MIN_SPEED_PCT 재사용 */

/* ---- 내부 상태 ---- */
/* st만 volatile: MotorTask가 쓰고 SensorTask가 Drive_GetState()로 크로스-태스크 읽기
 * (선제 제동 게이트). 단일 워드라 원자적 — 가시성만 보장하면 됨 */
static volatile DriveState st = DS_CRUISE;
static uint32_t   t_state  = 0;    /* 현재 상태 진입 시각 */
static uint8_t    turn_dir = 0;    /* 0=좌 1=우 */
static uint8_t    swap_cnt = 0;    /* 이번 SPIN에서 방향 flip 횟수 */
static uint8_t    rev_cnt  = 0;    /* CRUISE 복귀 없이 누적된 후진 chunk (CRUISE 진입 시 리셋) */
static uint8_t    clear_cnt = 0;   /* 정면 트임 연속(유효 측정) 카운트 */
static float      h_entry  = 0.0f; /* SPIN 진입 heading (회전각 게이트/미러 기준) */
static float      h_ref    = 0.0f; /* heading-hold 기준 */
static float      h_stuck  = 0.0f; /* 스턱 감지 윈도 시작 heading */
static uint32_t   t_stuck  = 0;
static float      steer_u  = 0.0f; /* slew 연속성용 직전 조향량 */
static float      side_set = 0.0f; /* 한벽 모드 목표거리 */
static uint8_t    steer_mode = SM_INIT;
static float      h_prev   = 0.0f; /* yaw-rate D항: 직전 heading */
static uint32_t   t_prev   = 0;    /* yaw-rate D항: 직전 시각 */

/* 임의 각도차 → (-180, 180]. 입력은 0~360 값들의 차라 루프는 1~2회로 종료 */
static float wrap180(float a)
{
    while (a >   180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
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

static void Drive_CenteringPD_Run(const DriveInputs *in)
{
    static uint16_t l_buf[CENTER_MA_WIN];
    static uint16_t r_buf[CENTER_MA_WIN];
    static uint8_t  idx = 0U;
    static uint8_t  count = 0U;
    static float    prev_error = 0.0f;
    static uint32_t prev_ms = 0U;

    l_buf[idx] = center_sanitize_cm(in->l);
    r_buf[idx] = center_sanitize_cm(in->r);

    idx = (uint8_t)((idx + 1U) % CENTER_MA_WIN);
    if (count < CENTER_MA_WIN)
        count++;

    float left_cm  = center_avg_u16(l_buf, count);
    float right_cm = center_avg_u16(r_buf, count);
    float error = left_cm - right_cm;
    float steer = 0.0f;

    /* 전방 근접 블렌드 계수: f가 CORNER_NEAR_CM→FRONT_STOP_CM로 줄수록 fc 0→1 */
    float fc = 0.0f;
    if ((float)in->f < CORNER_NEAR_CM)
        fc = clampf((CORNER_NEAR_CM - (float)in->f)
                  / (CORNER_NEAR_CM - (float)FRONT_STOP_CM), 0.0f, 1.0f);

    /* 조향 권한 램프: 직진 CENTER_STEER_MAX_PCT(10) → apex CORNER_STEER_MAX_PCT(38) */
    float steer_max = CENTER_STEER_MAX_PCT
                    + fc * (CORNER_STEER_MAX_PCT - CENTER_STEER_MAX_PCT);

    if (fabsf(error) <= CENTER_DEADZONE_CM)
    {
        prev_error = 0.0f;
    }
    else
    {
        float dt = 0.02f;

        if (prev_ms != 0U && in->now > prev_ms)
        {
            dt = (float)(in->now - prev_ms) / 1000.0f;
            dt = clampf(dt, 0.005f, 0.100f);
        }

        float d_error = (error - prev_error) / dt;

        steer = (CENTER_KP * error) + (CENTER_KD * d_error);
        steer = clampf(steer, -steer_max, steer_max);   /* 고정 10 대신 램프된 상한 */
        prev_error = error;
    }

    prev_ms = in->now;

    /* 속도: 조향량 비례 감속(기존) + 전방근접 비례 감속(신규, min쪽) → 벽 앞 부드러운 접근 */
    float steer_ratio = fabsf(steer) / steer_max;   /* 램프된 max로 정규화 (steer_max≥10>0) */
    float base = CENTER_BASE_SPEED_PCT
               - ((CENTER_BASE_SPEED_PCT - CENTER_MIN_SPEED_PCT) * steer_ratio);
    base = base - fc * (base - CENTER_MIN_SPEED_PCT);
    base = clampf(base, CENTER_MIN_SPEED_PCT, CENTER_BASE_SPEED_PCT);

    float lt = clampf(base - steer, 0.0f, 100.0f);   /* 안쪽 바닥 0 = 아크(역회전 없음) */
    float rt = clampf(base + steer, 0.0f, 100.0f);

    Motor_Left((int8_t)(lt + 0.5f));
    Motor_Right((int8_t)(rt + 0.5f));

    dbg_steer      = steer;
    dbg_steer_mode = SM_BOTH;
    dbg_duty_l     = (int16_t)(lt + 0.5f);
    dbg_duty_r     = (int16_t)(rt + 0.5f);
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
    steer_u    = 0.0f;
    steer_mode = SM_INIT;
    rev_cnt    = 0;
    h_prev     = in->heading;   /* yaw-rate 리셋: 회피 복귀 직후 stale heading차로 스파이크 D항 방지 */
    t_prev     = in->now;
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
    t_stuck = in->now;
    enter(DS_SPIN, in->now);
}

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

/* ---- 상태별 실행 ---- */

/* CRUISE: 조향 주행. 3모드 자동 전환(양벽/한벽/무벽) + slew + 데드밴드 spillover + 접근 감속 */
static void cruise_run(const DriveInputs *in)
{
    /* fail-safe: 정면 연속 무에코 → HOLD. CRUISE에서만 — SPIN/REVERSE는 전방 노출이 없어 면제 */
    if (in->front_miss >= FRONT_FAIL_LIMIT) { hold_enter(in); return; }

    /* 정면 근접 → 즉시 능동 제동 + 회피 (제동은 안전하므로 디바운스 없음) */
    if (in->f < FRONT_STOP_CM) { brake_enter(in); return; }

    /* 측면 박기직전 → 비상 회피(멈춤+피벗). 정면 다음 우선 — 조향으론 못 막는 측벽 긁힘 차단 */
    {
        uint16_t smin = (in->l < in->r) ? in->l : in->r;
        if (smin < SIDE_AVOID_CM) { side_avoid_enter(in); return; }
    }
    Drive_CenteringPD_Run(in);
    return;
}

/* BRAKE: BRAKE_MS 동안 단락 제동 유지 → 해제 후 막다른곳이면 REVERSE, 아니면 SPIN */
static void brake_run(const DriveInputs *in)
{
    if ((in->now - t_state) < (uint32_t)BRAKE_MS) return;   /* 제동 유지 중 */

    Car_Stop();   /* 단락 제동 해제 */

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
    if (in->f < FRONT_DANGER_CM && in->l < SIDE_BLOCK_CM && in->r < SIDE_BLOCK_CM
        && rev_cnt < REV_MAX_CHUNKS)
    {
        reverse_enter(in);
        return;
    }

    if (in->imu_live)
    {
        dbg_spin_deg = wrap180(in->heading - h_entry);

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
    }

    /* 방향 재평가: 최소유지(SPIN_COMMIT_MS) 후, 래치쪽 막힘 && 반대쪽 SIDE_HYST 이상 우세 → 1회 flip */
    if ((in->now - t_state) >= (uint32_t)SPIN_COMMIT_MS && swap_cnt < SWAP_LIMIT)
    {
        uint16_t latched = (turn_dir == 0) ? in->l : in->r;
        uint16_t other   = (turn_dir == 0) ? in->r : in->l;
        if (latched < SIDE_BLOCK_CM && other > (uint16_t)(latched + SIDE_HYST))
        {
            turn_dir ^= 1;
            swap_cnt++;
            t_state = in->now;
            h_entry = in->heading;
        }
    }

    /* 탈출 → CRUISE: 정면 트임이 '유효 측정으로' CLEAR_CONFIRM회 연속
       + (IMU 시) 진입 대비 TURN_MIN_DEG 이상 회전 — 경사벽 specular 가짜-트임 조기탈출 차단 */
    if (in->f >= FRONT_CLEAR_CM && in->f_valid)
    {
        if (clear_cnt < 255) clear_cnt++;
        if (clear_cnt >= CLEAR_CONFIRM &&
            (!in->imu_live || fabsf(wrap180(in->heading - h_entry)) >= TURN_MIN_DEG))
        {
            cruise_enter(in);
            return;
        }
    }
    else if (in->f < FRONT_CLEAR_CM)
    {
        clear_cnt = 0;   /* 무효 측정(f_valid=0)은 리셋도 가산도 안 함 — 직전 카운트 유지 */
    }

    /* 회전 명령 (매 루프 재가) */
    if (turn_dir == 0) Car_PivotLeft(TURN_SPEED, TURN_INNER);
    else               Car_PivotRight(TURN_SPEED, TURN_INNER);
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
    uint16_t smin = (in->l < in->r) ? in->l : in->r;
    if (smin >= SIDE_AVOID_CLEAR_CM) { cruise_enter(in); return; }

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
    steer_u    = 0.0f;
    steer_mode = SM_INIT;
}

void Drive_Update(const DriveInputs *in)
{
    switch (st)
    {
    case DS_CRUISE:  cruise_run(in);  break;
    case DS_BRAKE:   brake_run(in);   break;
    case DS_SPIN:    spin_run(in);    break;
    case DS_REVERSE: reverse_run(in); break;
    case DS_HOLD:    hold_run(in);    break;
    case DS_SIDE_AVOID: side_avoid_run(in); break;
    default:         Car_Stop(); st = DS_CRUISE; break;
    }
    dbg_state    = (uint8_t)st;
    dbg_turn_dir = turn_dir;
    dbg_rev_cnt  = rev_cnt;
}

DriveState Drive_GetState(void)
{
    return st;
}
