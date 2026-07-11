/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"    /* hiwdg — MotorTask가 큐 수신 성공 시에만 refresh */
#include "motor.h"
#include "drive.h"   /* DriveInputs(큐 메시지 그대로 사용) + 튜닝 노브 */
#include "debug.h"   /* DebugMonitor_t dbg (SWD 미러 통합) */
#include "bno055.h"
#include "ultra.h"
#include "vl53l0x.h" /* 좌/우 측면 ToF (I2C1 공유 — BNO055와 동일 태스크 문맥 직렬화) */
#include "i2c.h"     /* hi2c1 */
#include "usart.h"   /* huart1 — 블루투스 1바이트 IT 수신 + 텔레메트리 IT 송신 */
#include "encoder.h" /* SG-207 휠 엔코더 ×2 (TIM2 CH1=PA15 / CH2=PB3) — 속도 측정 */
#include <stdio.h>   /* snprintf — 텔레메트리 프레임 조립 (정수 전용, float printf 금지) */
#include <string.h>  /* strchr/strcmp — '#KEY=VAL' 파서 */
#include <stdlib.h>  /* atoi */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* MotorTask 큐 수신 timeout [ms]: 정상 최악 센서 사이클(front 6 + left 6 + front 6 + right 6
 * + IMU timeout/복구 여유)의 약 2배 이상. 초과 = 센서 파이프라인 침묵 → Car_Stop + IWDG refresh 생략
 * → 2.048s 지속 시 워치독 리셋 (구 슈퍼루프의 '루프 멈춤=리셋' 의미 보존) */
#define DRIVE_Q_TIMEOUT_MS  150U

/* ---- 측면 ToF(VL53L0X) ---- */
#define TOF_LEFT_ADDR_8BIT  0x60U  /* 좌측 주소 재배치(0x30<<1). 우측은 디폴트 0x52 유지 — 버스 충돌 해소 */
#define TOF_CAP_MM          1000U  /* 상한 캡[mm]: out-of-range(8190/8191)/장거리 지터를 코스 스케일로 클램프 */
#define TOF_BUDGET_US       20000U /* 측정 버짓 = 샘플 주기. 디폴트 33ms는 제어루프(~20ms)보다 느려 측면
                                      반응지연 병목(64% duty서 지연당 3~4.5cm 이동 = 반발존 3cm 초과 → 벽 hug).
                                      20ms(ST 고속 프로파일)로 루프당 새 샘플 1개 확보 (IMG_2977 대응) */
#define TOF_STALE_MS        150U   /* 새 샘플 무갱신 상한. 버짓 20ms의 ~7배 초과 = 센서 정체 → 트임 만료 */
#define TOF_LOOP_DELAY_MS   5U     /* SensorTask 루프 양보. 측면이 논블로킹化돼 사라진 대기 보상(CPU 독점 방지) */

/* ---- 텔레메트리 (아키텍처 §4 / dash_board.html 프로토콜) ----
 * TX 프레임: "T,<t_ms>,<f cm>,<L mm>,<R mm>,<h×10>,<vL>,<vR>,<st>,<fl>\n"  (정수만, 최악 ~46B)
 *   st: 0~6 = DriveState, 7 = MANUAL(수동 또는 전원 OFF) — dash_board STATE_NAMES와 1:1 고정
 *   fl: b0 f_valid, b1 side_valid, b2 imu_live, b3 sys_power, b4 sys_mode
 * RX 명령: 레거시 1바이트('1','0','A','M','U','D','L','R','S') +
 *          '#KEY=VAL'+LF 라인 (VT/TEL/HZ/ML/MR — 라인버퍼 24B, 500ms 타임아웃 리셋)
 * 9600bps에서 46B ≈ 48ms — 10Hz(주기 100ms)면 점유 ~48%: 동작하나 여유 없음 → TX는 반드시
 * IT(논블로킹) + 직전 송신 미완료 시 프레임 스킵(dbg.tel_skip). 보레이트 승격은 R3 옵션 */
#define BT_LINE_TIMEOUT_MS  500U   /* '#' 라인 모드 미완성 버퍼 만료 */
#define TEL_HZ_DEFAULT      10U    /* 기본 프레임 레이트 [Hz] (dash_board/앱 처리 상한) */
#define TEL_HZ_MAX          20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* SensorTask → MotorTask value queue. Depth 2 allows a full frame and one danger event. */
static osMessageQueueId_t driveQ = NULL;

/* ---- 디버그 미러: debug.h의 DebugMonitor_t dbg로 통합 (정의는 drive.c 1곳). 접근 dbg.<member> ---- */

/* ---- 블루투스 원격제어 상태 (BluetoothTask 단일 writer, MotorTask reader) ----
 * 1바이트/int8 원자 접근 → 뮤텍스 불요. volatile = 태스크간 가시성(컴파일러 캐싱 차단) */
volatile uint8_t sys_power = 0;     /* 0=OFF(모터 강제정지) 1=ON. 부팅 OFF(안전) */
volatile uint8_t sys_mode  = 0;     /* 0=자율주행(Drive_Update) 1=수동(manual_* 인가) */
volatile int8_t  manual_left  = 0;  /* 수동 좌 듀티 [-100..100] (+전진 / -후진) */
volatile int8_t  manual_right = 0;  /* 수동 우 듀티 */
volatile uint8_t bt_rx_byte   = 0;  /* USART1 1바이트 IT 수신 버퍼 (RxCpltCallback → uartQ) */

/* SensorTask writes; BluetoothTask reads for telemetry. */
static volatile uint16_t dist_left  = TOF_CAP_MM;
static volatile uint16_t dist_right = TOF_CAP_MM;

/* ---- 텔레메트리 미러 (아키텍처 §2.4 D3/D4 — 필드별 단일 작성자 volatile, 뮤텍스 0 유지) ----
 * 16-bit 정렬 store = M4 원자. 표시용이라 필드 간 ≤1사이클 시차 허용 → 락프리 */
static volatile uint16_t tel_h_x10  = 0;    /* SensorTask: heading×10 [0.1° 단위, 0~3599] */
static volatile uint8_t  tel_sflags = 0;    /* SensorTask: b0 f_valid, b1 side_valid(ToF 양측 기동 성공), b2 imu_live */
static volatile int16_t  tel_vl     = 0;    /* MotorTask: 좌 휠 속도 [cm/s, 부호 = 최근 명령 방향] */
static volatile int16_t  tel_vr     = 0;    /* MotorTask: 우 휠 속도 */
static volatile uint8_t  tel_st     = 7U;   /* MotorTask: 0~6 DriveState / 7 = MANUAL(수동·OFF) */
static volatile int16_t  tel_steer  = 0;    /* MotorTask: 조향 출력 dbg.steer 반올림 [%duty, ±CENTER_STEER_MAX]. 튜닝: 리밋사이클(weaving) 주파수/진폭 관찰용 */
static volatile uint8_t  tel_on     = 1U;   /* BluetoothTask: #TEL=0/1 (기본 ON — 연결 즉시 프레임 흐름) */
static volatile uint8_t  tel_hz     = TEL_HZ_DEFAULT; /* BluetoothTask: #HZ=1..20 */

/* USER CODE END Variables */
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BluetoothTask */
osThreadId_t BluetoothTaskHandle;
const osThreadAttr_t BluetoothTask_attributes = {
  .name = "BluetoothTask",
  .stack_size = 384 * 4,   /* 256→384: 텔레메트리 snprintf 여유(§2.2). .ioc Tasks01에도 384 반영됨 */
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartQ */
osMessageQueueId_t uartQHandle;
const osMessageQueueAttr_t uartQ_attributes = {
  .name = "uartQ"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#if MOTOR_TEST
/* 테스트 시퀀스 전용 대기: IWDG timeout(2.048s)보다 긴 무갱신 대기가 흐르면 시퀀스 중간
 * 리셋 → '전진만 반복' 폭주. 10ms 단위(osDelay)로 갱신하며 대기 (구 test_delay의 RTOS판) */
static void test_delay(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += 10U)
    {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(10);
    }
}
#endif

#if !MOTOR_TEST
/* ---- 측면 ToF 디바이스 (SensorTask 단독 소유) ----
 * 두 센서 모두 부팅 시 디폴트 0x52 — XSHUT 순차 기동으로 좌측만 0x60으로 재배치 */
static VL53L0X tof_left  = { .hi2c = &hi2c1, .addr = VL53L0X_ADDR_DEFAULT_8BIT, .stop_variable = 0 };
static VL53L0X tof_right = { .hi2c = &hi2c1, .addr = VL53L0X_ADDR_DEFAULT_8BIT, .stop_variable = 0 };
static uint8_t tof_left_ok  = 0;   /* Init+연속모드 성공 여부. 실패 센서는 트임(MAX) 고정 */
static uint8_t tof_right_ok = 0;

/* XSHUT 시퀀스로 주소 충돌 해소 후 양쪽 초기화 + 연속측정 시작.
 * 순서: 둘 다 리셋 → 좌만 기동 → 좌 주소 0x52→0x60 → 우 기동(0x52 유지).
 * VL53L0X 주소는 내부 RAM이라 XSHUT low/전원 리셋마다 디폴트 복귀 → 부팅마다 이 시퀀스 필수.
 * osDelay 사용 = 스케줄러 기동 후(SensorTask 문맥) 호출 전제 */
static void Init_ToF_Sensors(void)
{
    HAL_GPIO_WritePin(TOF_LEFT_XSHUT_GPIO_Port,  TOF_LEFT_XSHUT_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_RIGHT_XSHUT_GPIO_Port, TOF_RIGHT_XSHUT_Pin, GPIO_PIN_RESET);
    osDelay(10);

    HAL_GPIO_WritePin(TOF_LEFT_XSHUT_GPIO_Port, TOF_LEFT_XSHUT_Pin, GPIO_PIN_SET);
    osDelay(10);   /* t_boot ~1.2ms — 여유 포함 */
    tof_left_ok  = VL53L0X_SetAddress(&tof_left, TOF_LEFT_ADDR_8BIT);   /* 우측 기동 전 = 버스에 좌측뿐 */
    tof_left_ok &= VL53L0X_Init(&tof_left);

    HAL_GPIO_WritePin(TOF_RIGHT_XSHUT_GPIO_Port, TOF_RIGHT_XSHUT_Pin, GPIO_PIN_SET);
    osDelay(10);
    tof_right_ok = VL53L0X_Init(&tof_right);    /* 디폴트 0x52 — 좌측은 이미 0x60이라 충돌 없음 */

    /* 버짓 20ms 적용 — 실패해도 디폴트 33ms로 동작하므로 ok 플래그는 유지 */
    if (tof_left_ok)  (void)VL53L0X_SetTimingBudget(&tof_left,  TOF_BUDGET_US);
    if (tof_right_ok) (void)VL53L0X_SetTimingBudget(&tof_right, TOF_BUDGET_US);

    if (tof_left_ok)  tof_left_ok  = VL53L0X_StartContinuous(&tof_left);
    if (tof_right_ok) tof_right_ok = VL53L0X_StartContinuous(&tof_right);
}

/* 측면 ToF 1회 폴링 → mm 미러/cm hist 갱신.
 * 반환값 3상 처리: 1=새 샘플(캡 후 기록), 0=아직(버짓 ~33ms vs 루프 ~20ms — 정상, 직전값 유지),
 * -1=I2C 에러(연속 SIDE_FAIL_LIMIT회 → 트임 만료: stale 근접값이 조향을 오래 붙드는 것 차단).
 * 무에러 정체(TOF_STALE_MS 무갱신)도 만료 — 죽지도 살지도 않은 센서의 옛값 고착 방지 */
static void tof_record_side(VL53L0X *dev,
                            uint8_t alive,
                            uint16_t hist[SIDE_MED_WIN],
                            uint8_t *miss,
                            uint8_t *has_sample,
                            uint32_t *t_fresh,
                            volatile uint16_t *out_mm,
                            uint32_t now)
{
    if (!alive)
    {
        *out_mm = TOF_CAP_MM;
        for (uint8_t i = 0U; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
        return;
    }

    uint16_t mm;
    int8_t st = VL53L0X_PollRangeMM(dev, &mm);
    if (st > 0)
    {
        *miss = 0U;
        *has_sample = 1U;
        *t_fresh = now;
        if (mm > TOF_CAP_MM) mm = TOF_CAP_MM;           /* ceiling: out-of-range(8190) 지터 캡 */
        *out_mm = mm;
        uint16_t cm = mm / 10U;
        if (cm > ULTRA_MAX_CM) cm = ULTRA_MAX_CM;       /* drive 계층 스케일(cm, 80 상한) 정합 */
        if (cm == 0U) cm = 1U;                          /* 0은 center_sanitize가 '트임'으로 오독 — 1cm 바닥 */
        for (int i = SIDE_MED_WIN - 1; i > 0; i--) hist[i] = hist[i - 1];
        hist[0] = cm;
    }
    else if (st < 0)
    {
        if (*miss < 255U && ++(*miss) >= SIDE_FAIL_LIMIT)
        {
            *out_mm = TOF_CAP_MM;
            for (uint8_t i = 0U; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
        }
    }
    else if ((now - *t_fresh) > TOF_STALE_MS)
    {
        *out_mm = TOF_CAP_MM;
        for (uint8_t i = 0U; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
    }
}

static void sensor_record_front(uint8_t valid,
                                uint16_t cm,
                                uint16_t hist[FRONT_MED_WIN],
                                uint8_t *front_miss,
                                uint16_t *front_filtered)
{
    if (valid)
    {
        *front_miss = 0U;
        for (int i = FRONT_MED_WIN - 1; i > 0; i--) hist[i] = hist[i - 1];
        hist[0] = cm;
    }
    else if (*front_miss < 255U)
    {
        (*front_miss)++;
        if (*front_miss >= FRONT_FAIL_LIMIT)
        {
            for (uint8_t i = 0U; i < FRONT_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
        }
    }

    *front_filtered = median_n(hist, FRONT_MED_WIN);
    dbg.front_miss = *front_miss;
}

/* Front-only frames are urgent brake events, not normal FSM samples. */
static uint8_t sensor_put_front_danger_event(uint16_t f,
                                             uint8_t f_valid,
                                             uint8_t front_miss,
                                             float heading)
{
    if (f_valid && f < FRONT_DANGER_CM)
    {
        DriveInputs early;
        early.f          = f;
        early.l          = ULTRA_MAX_CM;
        early.r          = ULTRA_MAX_CM;
        early.f_valid    = f_valid;
        early.side_valid = 0U;
        early.left_valid = 0U;
        early.right_valid = 0U;
        early.front_miss = front_miss;
        early.heading    = heading;
        early.imu_live   = 0U;
        early.now        = HAL_GetTick();
        if (osMessageQueuePut(driveQ, &early, 0U, 0U) == osOK) return 1U;
        dbg.q_drop++;
    }
    return 0U;
}

/* ---- BNO055 heading 180° 재영점 (SensorTask 단독 소유 — 뮤텍스 불요) ----
 * 부팅 후 첫 유효 Yaw를 180°로 영점 보정: 시작 자세 근처에서 0/360 경계를 밟지 않아
 * 직진 중 값이 170~190° 범위에서 연속으로 움직인다 (359↔0 점프 제거).
 * 주의 1: drive.c 제어는 wrap180() 차분만 사용 → 오프셋 유무와 제어 거동 무관.
 * 주의 2: BNO055_Init()을 런타임에 재호출하는 코드를 추가하면 칩 내부 heading이
 *         0으로 리셋되므로 yaw_offset_latched도 함께 0으로 클리어할 것.
 * 부수 효과: 보정 후 heading이 항상 [0,360) 보장 → tel_h_x10 음수 캐스팅 위험 제거 */
static float   initial_yaw_offset = 0.0f;   /* 부팅 후 첫 유효 Raw Yaw [deg] */
static uint8_t yaw_offset_latched = 0U;     /* 1 = 영점 래치 완료 */

static bool Sensor_ReadHeading(BNO055_Euler *e)
{
    if (!BNO055_ReadEuler(e)) return false;   /* 실패 시 *e 미변경 — 직전값 유지 규약 그대로 */

    if (!yaw_offset_latched)
    {
        initial_yaw_offset = e->heading;      /* 첫 유효 샘플 = 새 영점 */
        yaw_offset_latched = 1U;
    }

    /* raw − offset + 180 → [0, 360) 정규화.
     * raw−offset ∈ (−360, 360) → +180 후 범위 (−180, 540) = 방어 분기 각 1회로 충분 */
    float h = (e->heading - initial_yaw_offset) + 180.0f;
    if      (h >= 360.0f) h -= 360.0f;
    else if (h <    0.0f) h += 360.0f;
    e->heading = h;
    return true;
}
#endif

/* ---- 블루투스 텔레메트리/파서 헬퍼 (BluetoothTask 단독 소유) ----
 * ⚠ 반드시 이 USER CODE 블록 안에 둘 것 — CubeMX 재생성 시 블록 밖 코드는 삭제된다 */

static char     bt_line[24];      /* 아키텍처 §4.3: '#KEY=VAL' 라인버퍼 ≤24B */
static uint8_t  bt_llen  = 0;
static uint8_t  bt_lmode = 0;     /* 1 = '#' 수신 후 라인 조립 중 */
static uint32_t bt_t_line = 0;    /* 라인 시작 시각 — BT_LINE_TIMEOUT_MS 만료용 */

/* '#KEY=VAL' 파싱 → 원자 반영. 모터 직접 호출 금지 원칙 유지(변수 경유만).
 * VT는 속도 PI(R4) 전까지 저장만(dbg.v_target) — 대시보드 #VT 버튼이 에러 없이 동작. */
static void BT_ParseLine(char *s)
{
    char *eq = strchr(s, '=');
    if (eq == NULL) return;
    *eq = '\0';
    int v = atoi(eq + 1);

    if      (strcmp(s, "VT") == 0)  { dbg.v_target = (int16_t)v; }
    else if (strcmp(s, "TEL") == 0) { tel_on = (v != 0) ? 1U : 0U; }
    else if (strcmp(s, "HZ") == 0)  { if (v < 1) v = 1; if (v > (int)TEL_HZ_MAX) v = (int)TEL_HZ_MAX;
                                      tel_hz = (uint8_t)v; }
    else if (strcmp(s, "ML") == 0)  { if (sys_mode) { if (v > 100) v = 100; if (v < -100) v = -100;
                                      manual_left  = (int8_t)v; } }   /* R2 duty 스윕 캘리브용 */
    else if (strcmp(s, "MR") == 0)  { if (sys_mode) { if (v > 100) v = 100; if (v < -100) v = -100;
                                      manual_right = (int8_t)v; } }
    /* 미정의 KEY 무시 — 프로토콜 전방 호환 */
}

/* 수신 1바이트 처리: 라인 모드 우선, 아니면 레거시 1바이트 명령 */
static void BT_HandleByte(uint8_t b)
{
    if (bt_lmode)
    {
        if (b == '\n' || b == '\r')
        {
            bt_line[bt_llen] = '\0';
            BT_ParseLine(bt_line);
            bt_lmode = 0U;
        }
        else if (bt_llen < (uint8_t)(sizeof(bt_line) - 1U))
        {
            bt_line[bt_llen++] = (char)b;
        }
        else
        {
            bt_lmode = 0U;   /* 오버플로 → 라인 폐기 (프레이밍 재동기) */
        }
        return;
    }

    if (b == '#')
    {
        bt_lmode  = 1U;
        bt_llen   = 0U;
        bt_t_line = HAL_GetTick();
        return;
    }

    switch (b)
    {
      /* ---- 시스템 전원 ---- */
      case '1':                                  /* ON */
        sys_power = 1;
        break;
      case '0':                                  /* OFF: 수동값 리셋 + 정지(어느 모드든) */
        sys_power = 0;
        manual_left = 0;
        manual_right = 0;
        break;

      /* ---- 모터 모드 (모드 선택 = 전원 ON 커플링) ----
       * 앱에 '1'(전원 ON) 버튼이 없어 sys_power가 0에 묶여 모터가 강제정지하던 문제 수정.
       * 모드를 고르는 것 = 주행 의사 → sys_power=1. 부팅은 여전히 OFF(안전). 정지는 '0'. */
      case 'A':                                  /* 자율주행 */
        sys_mode = 0;
        sys_power = 1;
        break;
      case 'M':                                  /* 수동: 진입 시 정지값으로 시작(돌발 구동 방지) */
        sys_mode = 1;
        sys_power = 1;
        manual_left = 0;
        manual_right = 0;
        break;

      /* ---- 수동 조작 (Manual 모드 전용) ---- */
      case 'U': if (sys_mode) { manual_left =  80; manual_right =  80; } break;  /* 전진 */
      case 'D': if (sys_mode) { manual_left = -80; manual_right = -80; } break;  /* 후진 */
      case 'L': if (sys_mode) { manual_left = -60; manual_right =  60; } break;  /* 좌회전 */
      case 'R': if (sys_mode) { manual_left =  60; manual_right = -60; } break;  /* 우회전 */
      case 'S': if (sys_mode) { manual_left =   0; manual_right =   0; } break;  /* 정지 */

      default:                                   /* 미정의 바이트 무시 */
        break;
    }
}

/* 텔레메트리 프레임 조립 + IT 송신. 버퍼는 IT 송신 완료까지 유지 필요 → static.
 * 직전 프레임 송신 미완료(gState != READY)면 이번 프레임 스킵 — 9600bps 병목의 우아한 강등 */
static void BT_SendFrame(void)
{
    static char fb[64];

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        dbg.tel_skip++;
        return;
    }

    uint8_t fl = (uint8_t)(tel_sflags
                         | (sys_power ? 0x08U : 0U)
                         | (sys_mode  ? 0x10U : 0U));
    int n = snprintf(fb, sizeof fb, "T,%lu,%u,%u,%u,%u,%d,%d,%u,%u,%d\n",
                     (unsigned long)HAL_GetTick(),
                     (unsigned)dbg.front,          /* 전방 [cm, 80 캡] */
                     (unsigned)dist_left,          /* 좌 ToF [mm, 1000 캡] */
                     (unsigned)dist_right,         /* 우 ToF [mm] */
                     (unsigned)tel_h_x10,          /* heading×10 [0.1°] */
                     (int)tel_vl, (int)tel_vr,     /* 휠 속도 [cm/s, 부호=명령방향] */
                     (unsigned)tel_st,             /* 0~6 DriveState / 7 MANUAL */
                     (unsigned)fl,                 /* 플래그: b0 f_valid b1 side_valid b2 imu_live b3 power b4 mode */
                     (int)tel_steer);              /* [추가] 조향 출력 [%duty] — 프레임 末 append (기존 idx0~9 불변, 앱 파서 하위호환) */
    if (n <= 0 || n >= (int)sizeof fb) return;

    /* USART1 ISR(RxCplt 재무장 Receive_IT)과 HAL 락 충돌 방지: 짧은 임계구역으로 배타.
     * 미보호 시 드물게 (a) TX가 HAL_BUSY 오탐, (b) RX 재무장 유실(수신 영구정지) 가능 */
    taskENTER_CRITICAL();
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart1, (uint8_t *)fb, (uint16_t)n);
    taskEXIT_CRITICAL();

    if (st == HAL_OK) dbg.tel_tx++;
    else              dbg.tel_skip++;
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartQ */
  uartQHandle = osMessageQueueNew (16, sizeof(uint8_t), &uartQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  driveQ = osMessageQueueNew(2, sizeof(DriveInputs), NULL);
  /* 생성 실패(NULL) 시 put/get이 전부 에러 → MotorTask가 Car_Stop + refresh 생략 → IWDG 리셋.
   * heap_4 15KB에 큐 ~수십B라 실패 가능성 사실상 없음 — 별도 처리 불요 */

  /* 블루투스 수신 개시: 최초 1바이트 IT 무장 (이후 RxCpltCallback가 자가 재무장).
   * uartQ 생성 직후라 콜백 put 대상 존재. USART1은 스케줄러 전 init 완료 → 지금 무장 안전 */
  __HAL_UART_CLEAR_OREFLAG(&huart1);  /* 부팅 라인 글리치로 선-박힌 ORE 제거 후 무장 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&bt_rx_byte, 1);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartDefaultTask, NULL, &MotorTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask02, NULL, &SensorTask_attributes);

  /* creation of BluetoothTask */
  BluetoothTaskHandle = osThreadNew(StartTask03, NULL, &BluetoothTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

#if MOTOR_TEST
  /* 바퀴 방향/속도/제동 눈 확인 + 수치 튜닝용. 센서 로직 건너뜀(SensorTask는 휴면).
   * test_delay 사용 필수: 무갱신 대기는 IWDG 리셋 → 시퀀스 중간 폭주 */
  for(;;)
  {
    Car_Forward(DRIVE_SPEED);   test_delay(1500);   /* 양쪽 다 전진하는지 확인 */
    Car_Brake();                test_delay(400);    /* 능동 제동: 코스트 대비 정지거리 육안 비교 */
    Car_Stop();                 test_delay(800);
    Car_PivotLeft(TURN_SPEED, TURN_INNER);   test_delay(1500);   /* IN1 전진 + IN3 후진소량 → 좌회전 */
    Car_Stop();                              test_delay(800);
    Car_PivotRight(TURN_SPEED, TURN_INNER);  test_delay(1500);   /* IN4 전진 + IN2 후진소량 → 우회전 */
    Car_Stop();                              test_delay(800);
    Car_Backward(DRIVE_SPEED);  test_delay(1500);
    Car_Stop();                 test_delay(800);
  }
#else
  /* 제어 소비자 = 모터 명령 유일 발행자 (폴트 핸들러의 Car_Stop 제외).
   * IWDG는 '신선한 센서 데이터가 흐를 때만' refresh — SensorTask 행/사망, 큐 단절,
   * 본 태스크 행 어느 쪽이든 2.048s 내 워치독 리셋 (구 슈퍼루프 fail-safe 의미 보존) */
  DriveInputs din;
  uint8_t prev_auto = 0;   /* 직전 루프가 자율 구동? M→A / OFF→A 재진입 edge 검출 */
  for(;;)
  {
    if (osMessageQueueGet(driveQ, &din, NULL, DRIVE_Q_TIMEOUT_MS) == osOK)
    {
      HAL_IWDG_Refresh(&hiwdg);   /* 센서 파이프라인 생존 = refresh (주행모드 무관) */

      /* === 모터 제어권 단일 분기 (경쟁 원천 차단; 발행자는 본 태스크 하나) === */
      if (sys_power == 0)
      {
        Car_Stop();                    /* 전원 OFF: 강제 정지 (최우선) */
        prev_auto = 0;
      }
      else if (sys_mode == 1)
      {
        Motor_SetWheels(manual_left, manual_right);  /* 수동: 앱 듀티 직접 인가 (음수=후진) */
        prev_auto = 0;
      }
      else                             /* sys_power==1 && sys_mode==0 → 자율주행 */
      {
        if (!prev_auto)                /* 수동/OFF에서 막 복귀 → 상태머신 리셋(스테일 상태 재개 방지) */
        {
          Drive_Init();
          prev_auto = 1;
        }
        Drive_Update(&din);            /* din.now 기반 비블로킹 제어. early front-only 프레임은 직전 PWM 유지 가능 */
      }
    }
    else
    {
      dbg.q_timeout++;
      Car_Stop();           /* 센서 침묵 동안 안전 정지. refresh 생략 → 지속 시 IWDG 리셋 */
    }

    /* --- 엔코더 속도 미러 (아키텍처 D2→D3: ISR 기록 → 본 태스크 환산 → 텔레메트리) ---
     * din 신선도 무관 매 루프 갱신: 전원 OFF 상태로 손으로 바퀴를 굴려도 dbg/대시보드에
     * 속도가 보여 R0 배선검증·R2 캘리브레이션이 재플래시 없이 가능하다.
     * 부호 = 단채널 한계로 '최근 구동 명령 방향'(motor_dir_*) 채택 (§3.2 명문화) */
    {
      uint32_t enc_now = HAL_GetTick();
      float vl = Encoder_SpeedCmps(ENC_LEFT,  enc_now);
      float vr = Encoder_SpeedCmps(ENC_RIGHT, enc_now);
      dbg.v_l   = vl;
      dbg.v_r   = vr;
      dbg.enc_l = Encoder_Edges(ENC_LEFT);
      dbg.enc_r = Encoder_Edges(ENC_RIGHT);
      tel_vl = (int16_t)((motor_dir_left  < 0) ? -(vl + 0.5f) : (vl + 0.5f));
      tel_vr = (int16_t)((motor_dir_right < 0) ? -(vr + 0.5f) : (vr + 0.5f));
      tel_st = (sys_power == 0U || sys_mode == 1U) ? 7U : dbg.state;   /* 7 = MANUAL/IDLE */
      tel_steer = (int16_t)((dbg.steer >= 0.0f) ? (dbg.steer + 0.5f) : (dbg.steer - 0.5f));  /* st와 같은 프레임 스냅샷(테어링 방지) */
    }
    dbg.hw_motor = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  }
#endif
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  (void)argument;

#if MOTOR_TEST
  /* 모터 점검 모드: 측정/큐 불요 — MotorTask가 시퀀스 전담, 여기는 휴면 */
  for(;;) osDelay(1000);
#else
  /* 측정/필터 상태 (구 main.c 슈퍼루프에서 이동) — 제어 상태머신은 drive.c(MotorTask 문맥).
   * 정면/측면 median 윈도는 독립 노브다. 측면 ToF에도 단발 outlier 방어 계층을 유지한다. */
  static uint16_t hist_f[FRONT_MED_WIN];
  static uint16_t hist_l[SIDE_MED_WIN];
  static uint16_t hist_r[SIDE_MED_WIN];
  for (uint8_t i = 0U; i < FRONT_MED_WIN; i++)
      hist_f[i] = ULTRA_MAX_CM;
  for (uint8_t i = 0U; i < SIDE_MED_WIN; i++)
  {
      hist_l[i] = ULTRA_MAX_CM;
      hist_r[i] = ULTRA_MAX_CM;
  }
  static uint8_t  front_miss = 0;   /* 정면 에코 연속 미회신 (stale front 제거 및 전 센서 상실 판정 입력) */
  static uint8_t  tof_miss[2]  = {0, 0};   /* 좌/우 ToF I2C 에러 연속 카운트 (SIDE_FAIL_LIMIT 만료) */
  static uint8_t  tof_has_sample[2] = {0U, 0U};  /* 좌/우 PollRangeMM 성공 샘플 수신 여부 */
  static uint32_t tof_fresh[2] = {0, 0};   /* 좌/우 마지막 새 샘플 시각 (TOF_STALE_MS 정체 만료) */
  static uint8_t  imu_live = 0;     /* IMU 생존 게이트 */
  static uint8_t  imu_fail = 0;     /* 읽기 연속 실패 카운트 */
  static uint32_t t_imu_retry = 0;  /* 사망 후 재시도 타이머 */
  static uint32_t t_calib = 0;      /* CALIB_STAT 디버그 읽기 스로틀 */
  static uint32_t t_loop = 0;       /* 사이클 주기 측정 */
  static BNO055_Euler imu;          /* 최근 융합값 (read 실패 시 직전값 유지) */

  uint16_t f = ULTRA_MAX_CM, l = ULTRA_MAX_CM, r = ULTRA_MAX_CM;  /* 필터 출력 */
  uint16_t d_front;
  uint8_t  v_front;

  Ultra_Init(osThreadGetId());      /* TIM3 IC start(전방 CH1) + 측정 완료 flag 수신자 등록 */
  Encoder_Init();                   /* SG-207 휠 엔코더: TIM2 전체 런타임 구성 (CubeMX 미사용, 독립 타이머) */
  imu_live = dbg.imu_ok;            /* Init 성공했을 때만 생존 시작 (실패 시 거리-only 고정) */
  Init_ToF_Sensors();               /* XSHUT 순차 기동 + 좌측 주소 0x60 재배치 + 연속측정 시작 */
  tof_fresh[0] = HAL_GetTick();     /* 정체 만료 타이머 기준점 — 기동 직후 오탐 방지 */
  tof_fresh[1] = tof_fresh[0];

  for(;;)
  {
      uint32_t now = HAL_GetTick();
      dbg.loop_ms = now - t_loop;   /* 사이클 주기 미러 (front-left-front-right 반응지연 감시) */
      t_loop = now;

      uint8_t front_cycle_valid = 0U;
      uint8_t danger_event_sent = 0U;

      /* Front is sampled twice around the left ToF poll for low brake latency. */
      v_front = Ultra_Measure(MEAS_WAIT_MS, &d_front);           /* front (TRIG PA5, ECHO TIM3_CH1) */
      front_cycle_valid |= v_front;
      sensor_record_front(v_front, d_front, hist_f, &front_miss, &f);
      danger_event_sent = sensor_put_front_danger_event(f, v_front, front_miss, imu.heading);
      /* Poll each continuously ranging ToF around the second front sample. */
      tof_record_side(&tof_left, tof_left_ok, hist_l, &tof_miss[0], &tof_has_sample[0],
                      &tof_fresh[0], &dist_left, now);
      v_front = Ultra_Measure(MEAS_WAIT_MS, &d_front);
      front_cycle_valid |= v_front;
      sensor_record_front(v_front, d_front, hist_f, &front_miss, &f);
      if (!danger_event_sent)
          (void)sensor_put_front_danger_event(f, v_front, front_miss, imu.heading);

      tof_record_side(&tof_right, tof_right_ok, hist_r, &tof_miss[1], &tof_has_sample[1],
                      &tof_fresh[1], &dist_right, now);

      l = median_n(hist_l, SIDE_MED_WIN);   /* 측면 median 필터 계층 유지 (단발 outlier 방어) */
      r = median_n(hist_r, SIDE_MED_WIN);
      dbg.front = f; dbg.left = l; dbg.right = r;

      /* 3) IMU 샘플 (생존 게이트): 정상 시 매 사이클 heading 갱신. 연속 IMU_FAIL_LIMIT회 실패 →
       *    사망 선언 후 IMU_RETRY_MS 주기로만 재시도 — 죽은 IMU가 사이클마다 I2C timeout(10ms×2)을
       *    태워 반응지연 키우는 것 차단. CalibStatus는 디버그 전용이라 CALIB_POLL_MS 스로틀. */
      uint8_t imu_ctrl_live = 0U;   /* 이번 제어 프레임에서 새 heading을 읽었을 때만 1 */
      if (imu_live)
      {
          if (Sensor_ReadHeading(&imu))   /* 180° 재영점 적용 read (raw는 BNO055_ReadEuler) */
          {
              imu_fail = 0;
              imu_ctrl_live = 1U;
              dbg.heading = imu.heading;
              dbg.roll    = imu.roll;
              dbg.pitch   = imu.pitch;
          }
          else if (++imu_fail >= IMU_FAIL_LIMIT)
          {
              imu_live = 0;
              t_imu_retry = now;
          }
          if ((now - t_calib) >= CALIB_POLL_MS)
          {
              t_calib = now;
              dbg.imu_calib = BNO055_ReadCalibStatus();
          }
      }
      else if (dbg.imu_ok && (now - t_imu_retry) >= IMU_RETRY_MS)
      {
          /* Init 성공 이력 있을 때만 재시도 — Init 실패 상태의 garbage 읽기로 살아있다고
           * 오판하는 것 방지. 버스 wedge는 bno_rd 내부 복구(10회 실패→9클럭)와 협력 */
          t_imu_retry = now;
          if (Sensor_ReadHeading(&imu))   /* 재기동 read도 동일 영점 유지 (오프셋은 부팅 1회 래치) */
          {
              imu_live = 1;
              imu_fail = 0;
              imu_ctrl_live = 1U;
              dbg.heading = imu.heading;
              dbg.roll    = imu.roll;
              dbg.pitch   = imu.pitch;
          }
      }
      dbg.imu_live = imu_ctrl_live;

      uint32_t frame_now = HAL_GetTick();
      uint8_t left_valid = (uint8_t)(tof_left_ok
                                  && tof_has_sample[0]
                                  && tof_miss[0] < SIDE_FAIL_LIMIT
                                  && (frame_now - tof_fresh[0]) <= TOF_STALE_MS);
      uint8_t right_valid = (uint8_t)(tof_right_ok
                                   && tof_has_sample[1]
                                   && tof_miss[1] < SIDE_FAIL_LIMIT
                                   && (frame_now - tof_fresh[1]) <= TOF_STALE_MS);

      /* 3.5) 텔레메트리 센서 미러 (아키텍처 D4 — 본 태스크 단일 작성자).
       *      heading은 ×10 정수화(float printf 회피). side_valid는 양쪽 ToF가 실제 샘플을
       *      수신했고 오류 한도와 stale 기한 안에 있을 때만 표시한다. */
      {
          uint16_t hx = (uint16_t)((imu.heading * 10.0f) + 0.5f);
          if (hx >= 3600U) hx = (uint16_t)(hx - 3600U);
          tel_h_x10  = hx;
          tel_sflags = (uint8_t)((front_cycle_valid ? 0x01U : 0U)
                               | ((left_valid && right_valid) ? 0x02U : 0U)
                               | (imu_ctrl_live ? 0x04U : 0U));
      }

      /* 4) 제어 입력 스냅샷 → 큐 (값 복사 — MotorTask가 IWDG refresh + Drive_Update) */
      DriveInputs din;
      din.f          = f;
      din.l          = l;
      din.r          = r;
      din.f_valid    = front_cycle_valid;
      din.side_valid = 1U;           /* full-frame marker; per-channel validity is below */
      din.left_valid = left_valid;
      din.right_valid = right_valid;
      din.front_miss = front_miss;
      din.heading    = imu.heading;   /* imu_ctrl_live=0이면 drive.c가 무시(거리-only 강등) */
      din.imu_live   = imu_ctrl_live;
      din.now        = frame_now;
      if (osMessageQueuePut(driveQ, &din, 0U, 0U) != osOK) { dbg.q_drop++; }

      dbg.hw_sensor = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

      /* 측면이 블로킹 대기(구 SIDE_WAIT_MS×2)에서 논블로킹 폴링으로 바뀌어 사라진 양보 시간 보상.
       * 루프 주기 ~20ms 유지 (ToF 버짓 ~33ms와 정합 — 과도 폴링/CPU 독점 방지) */
      osDelay(TOF_LOOP_DELAY_MS);
  }
#endif
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the BluetoothTask thread.
*        RX: 레거시 1바이트 + '#KEY=VAL' 라인 / TX: 텔레메트리 프레임 tel_hz Hz (IT 논블로킹).
*        dash_board.html(HM-10 BLE 콘솔)과 프로토콜 1:1 — 아키텍처 §4.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  (void)argument;
  uint8_t  cmd;
  uint32_t t_tx = HAL_GetTick();

  for(;;)
  {
    /* 큐 대기시간 = 다음 프레임 마감까지 (상한 100ms) — 명령은 즉시 처리, TX 스케줄은 지킴 */
    uint32_t now    = HAL_GetTick();
    uint32_t per_ms = 1000U / ((tel_hz == 0U) ? 1U : (uint32_t)tel_hz);
    uint32_t wait   = 100U;
    if (tel_on)
    {
        uint32_t elapsed = now - t_tx;
        wait = (elapsed >= per_ms) ? 0U : (per_ms - elapsed);
        if (wait > 100U) wait = 100U;
    }

    if (osMessageQueueGet(uartQHandle, &cmd, NULL, wait) == osOK)
    {
        do { BT_HandleByte(cmd); }
        while (osMessageQueueGet(uartQHandle, &cmd, NULL, 0U) == osOK);   /* 백로그 소진 */
    }

    now = HAL_GetTick();
    if (bt_lmode && (now - bt_t_line) > BT_LINE_TIMEOUT_MS)
        bt_lmode = 0U;   /* 미완성 '#' 라인 만료 → 버퍼 리셋 (§4.3) */

    if (tel_on && (now - t_tx) >= per_ms)
    {
        t_tx = now;
        BT_SendFrame();
    }
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* BNO055 INT(PA0/EXTI0) 콜백 (구 main.c에서 이동).
 * 주의: 융합 데이터엔 전용 DRDY INT가 없음 → heading은 SensorTask에서 polling.
 *       이 콜백은 INT 배선 검증/모션(any-motion 등) 트리거용 카운터. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IMU_INT_Pin)
    {
        dbg.imu_evt++;
    }
}

/* 블루투스 UART(USART1) 1바이트 수신 완료 콜백 (ISR 문맥).
 * 수신 바이트를 uartQ로 넘기고 즉시 다음 1바이트 IT 재무장 → 연속 수신.
 * osMessageQueuePut timeout 0 = ISR 안전(xQueueSendFromISR). 큐 가득 시 바이트는
 * 버려도 재무장은 무조건 수행해 수신이 멎지 않게 함 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        (void)osMessageQueuePut(uartQHandle, (const void *)&bt_rx_byte, 0U, 0U);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&bt_rx_byte, 1);
    }
}

/* 블루투스 UART(USART1) 에러 콜백 (ISR 문맥).
 * HAL은 ORE(overrun)/FE(framing)/NE(noise) 발생 시 수신을 중단하고 state=READY로 둔 뒤
 * 이 콜백만 호출 → 여기서 재무장하지 않으면 수신이 영구 정지(리셋 전까지 BT 먹통).
 * 모터 구동 전류로 인한 5V 브라운아웃/라인 노이즈로 바이트가 깨질 때가 트리거.
 * 에러 플래그 클리어 후 1바이트 IT를 다시 무장해 수신이 스스로 복구되게 함
 * (RxCpltCallback의 "무조건 재무장" 패턴과 일관). dbg.bt_err = SWD 관찰용 카운터. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        dbg.bt_err++;
        __HAL_UART_CLEAR_OREFLAG(&huart1);   /* SR read + DR read로 ORE 해제 */
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&bt_rx_byte, 1);
    }
}

/* USER CODE END Application */

