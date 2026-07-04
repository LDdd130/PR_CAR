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
#include "usart.h"   /* huart1 — 블루투스 1바이트 IT 수신 */
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* SensorTask → MotorTask 단방향 메일박스 (DriveInputs 값 복사 — 공유변수/뮤텍스 불요).
 * depth 2: full 메시지 + 선제제동 early 메시지 공존용 */
static osMessageQueueId_t driveQ = NULL;

/* ---- 디버그 미러: debug.h의 DebugMonitor_t dbg로 통합 (정의는 drive.c 1곳). 접근 dbg.<member> ---- */

/* ---- 블루투스 원격제어 상태 (BluetoothTask 단일 writer, MotorTask reader) ----
 * 1바이트/int8 원자 접근 → 뮤텍스 불요. volatile = 태스크간 가시성(컴파일러 캐싱 차단) */
volatile uint8_t sys_power = 0;     /* 0=OFF(모터 강제정지) 1=ON. 부팅 OFF(안전) */
volatile uint8_t sys_mode  = 0;     /* 0=자율주행(Drive_Update) 1=수동(manual_* 인가) */
volatile int8_t  manual_left  = 0;  /* 수동 좌 듀티 [-100..100] (+전진 / -후진) */
volatile int8_t  manual_right = 0;  /* 수동 우 듀티 */
volatile uint8_t bt_rx_byte   = 0;  /* USART1 1바이트 IT 수신 버퍼 (RxCpltCallback → uartQ) */

/* ---- 측면 ToF 공개 미러 (SensorTask 단일 writer, 16-bit 정렬 쓰기 = M4 원자적 → 뮤텍스 불요).
 * mm 단위, TOF_CAP_MM 상한. MotorTask 등 타 모듈은 main.h extern으로 읽기 전용 사용 */
volatile uint16_t dist_left  = TOF_CAP_MM;
volatile uint16_t dist_right = TOF_CAP_MM;
volatile int16_t  wall_error = 0;   /* dist_left - dist_right [mm]: +면 좌측이 더 트임 */

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
  .stack_size = 256 * 4,
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
                            uint32_t *t_fresh,
                            volatile uint16_t *out_mm,
                            uint32_t now)
{
    if (!alive)
    {
        *out_mm = TOF_CAP_MM;
        for (int i = 0; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
        return;
    }

    uint16_t mm;
    int8_t st = VL53L0X_PollRangeMM(dev, &mm);
    if (st > 0)
    {
        *miss = 0U;
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
            for (int i = 0; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
        }
    }
    else if ((now - *t_fresh) > TOF_STALE_MS)
    {
        *out_mm = TOF_CAP_MM;
        for (int i = 0; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
    }
}

static void sensor_record_front(uint8_t valid,
                                uint16_t cm,
                                uint16_t hist[SIDE_MED_WIN],
                                uint8_t *front_miss,
                                uint16_t *front_filtered)
{
    if (valid)
    {
        *front_miss = 0U;
        for (int i = SIDE_MED_WIN - 1; i > 0; i--) hist[i] = hist[i - 1];
        hist[0] = cm;
    }
    else if (*front_miss < 255U)
    {
        (*front_miss)++;
        if (*front_miss >= FRONT_FAIL_LIMIT)
        {
            for (int i = 0; i < SIDE_MED_WIN; i++) hist[i] = ULTRA_MAX_CM;
        }
    }

    *front_filtered = median_n(hist, FRONT_MED_WIN);
    dbg.front_miss = *front_miss;
}

static void sensor_put_front_early(uint16_t f,
                                   uint16_t l,
                                   uint16_t r,
                                   uint16_t raw_cm,
                                   uint8_t f_valid,
                                   uint8_t front_miss,
                                   float heading)
{
    uint16_t f_early = (f_valid && raw_cm < FRONT_STOP_CM) ? raw_cm : f;

    if (f_early < FRONT_TURN_CM)
    {
        DriveInputs early;
        early.f          = f_early;
        early.l          = l;            /* 직전 사이클 필터값 — side_valid=0이므로 측면 판단에는 사용 금지 */
        early.r          = r;
        early.f_valid    = f_valid;
        early.side_valid = 0U;
        early.front_miss = front_miss;
        early.heading    = heading;
        early.imu_live   = 0U;           /* 이번 루프 IMU 샘플 전이므로 stale heading 사용 금지 */
        early.now        = HAL_GetTick();
        if (osMessageQueuePut(driveQ, &early, 0U, 0U) != osOK) { dbg.q_drop++; }
    }
}
#endif

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
        Motor_Left(manual_left);       /* 수동: 앱 듀티 직접 인가 (음수=후진) */
        Motor_Right(manual_right);
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
   * hist 윈도 = SIDE_MED_WIN(drive.h): median 샘플수 = '한번씩 튀는 값' 제거 강도 노브.
   * 측면은 ToF로 이관됐지만 median3 필터 계층은 유지 — 유리/저반사 표면의 단발 outlier 방어 */
  static uint16_t hist_f[SIDE_MED_WIN];
  static uint16_t hist_l[SIDE_MED_WIN];
  static uint16_t hist_r[SIDE_MED_WIN];
  for (int i = 0; i < SIDE_MED_WIN; i++)
  {
      hist_f[i] = ULTRA_MAX_CM;
      hist_l[i] = ULTRA_MAX_CM;
      hist_r[i] = ULTRA_MAX_CM;
  }
  static uint8_t  front_miss = 0;   /* 정면 에코 연속 미회신 (stale front 제거 및 전 센서 상실 판정 입력) */
  static uint8_t  tof_miss[2]  = {0, 0};   /* 좌/우 ToF I2C 에러 연속 카운트 (SIDE_FAIL_LIMIT 만료) */
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

      /* 1) 정면 측정 → 필터 → 근접 시 'early' 메시지로 선제 반응(전방 제동 반응 앞당김).
       *    [req1] 관심사 분리: 센서는 제어기 상태(DS_*)를 모름 — f<FRONT_TURN_CM이면 상태 무관 우선 메시지 전송.
       *    side_valid=0으로 표시해 drive.c가 stale l/r로 코너/측면 판단을 하지 않게 한다. MotorTask 고prio라 put 즉시 선점.
       *    모터 명령 발행자는 MotorTask 하나로 유지 */
      v_front = Ultra_Measure(MEAS_WAIT_MS, &d_front);           /* front (TRIG PA5, ECHO TIM3_CH1) */
      front_cycle_valid |= v_front;
      sensor_record_front(v_front, d_front, hist_f, &front_miss, &f);
      sensor_put_front_early(f, l, r, d_front, v_front, front_miss, imu.heading);

      /* 2) 측면 ToF 폴링(논블로킹) — front-left-front-right 순서 유지: 정면을 중간에 재확인해서
       *    고속 코너/장애물 반응을 앞당긴다. ToF는 연속측정 자율 진행이라 폴링 비용 = I2C 2~3 트랜잭션뿐 */
      tof_record_side(&tof_left, tof_left_ok, hist_l, &tof_miss[0], &tof_fresh[0], &dist_left, now);

      v_front = Ultra_Measure(MEAS_WAIT_MS, &d_front);           /* front 재확인 */
      front_cycle_valid |= v_front;
      sensor_record_front(v_front, d_front, hist_f, &front_miss, &f);
      sensor_put_front_early(f, l, r, d_front, v_front, front_miss, imu.heading);

      tof_record_side(&tof_right, tof_right_ok, hist_r, &tof_miss[1], &tof_fresh[1], &dist_right, now);

      l = median_n(hist_l, SIDE_MED_WIN);   /* 측면 median 필터 계층 유지 (단발 outlier 방어) */
      r = median_n(hist_r, SIDE_MED_WIN);
      wall_error = (int16_t)((int32_t)dist_left - (int32_t)dist_right);   /* [mm] +=좌측이 더 트임 */
      dbg.front = f; dbg.left = l; dbg.right = r;

      /* 3) IMU 샘플 (생존 게이트): 정상 시 매 사이클 heading 갱신. 연속 IMU_FAIL_LIMIT회 실패 →
       *    사망 선언 후 IMU_RETRY_MS 주기로만 재시도 — 죽은 IMU가 사이클마다 I2C timeout(10ms×2)을
       *    태워 반응지연 키우는 것 차단. CalibStatus는 디버그 전용이라 CALIB_POLL_MS 스로틀. */
      uint8_t imu_ctrl_live = 0U;   /* 이번 제어 프레임에서 새 heading을 읽었을 때만 1 */
      if (imu_live)
      {
          if (BNO055_ReadEuler(&imu))
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
          if (BNO055_ReadEuler(&imu))
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

      /* 4) 제어 입력 스냅샷 → 큐 (값 복사 — MotorTask가 IWDG refresh + Drive_Update) */
      DriveInputs din;
      din.f          = f;
      din.l          = l;
      din.r          = r;
      din.f_valid    = front_cycle_valid;
      din.side_valid = 1U;
      din.front_miss = front_miss;
      din.heading    = imu.heading;   /* imu_ctrl_live=0이면 drive.c가 무시(거리-only 강등) */
      din.imu_live   = imu_ctrl_live;
      din.now        = HAL_GetTick();
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
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  (void)argument;
  uint8_t cmd;

  for(;;)
  {
    /* uartQ에서 1바이트 블로킹 수신 — 명령 올 때까지 CPU 양보(busy-wait 없음) */
    if (osMessageQueueGet(uartQHandle, &cmd, NULL, osWaitForever) != osOK)
        continue;

    switch (cmd)
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

