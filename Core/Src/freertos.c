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
#include "bno055.h"
#include "ultra.h"
#include "usart.h"   /* huart1 — 블루투스 1바이트 IT 수신 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* MotorTask 큐 수신 timeout [ms]: 정상 최악 센서 사이클(front 25 + side 6+6 + IMU timeout ~20
 * + 복구 ~1ms ≈ 60-70ms)의 약 2배. 초과 = 센서 파이프라인 침묵 → Car_Stop + IWDG refresh 생략
 * → 2.048s 지속 시 워치독 리셋 (구 슈퍼루프의 '루프 멈춤=리셋' 의미 보존) */
#define DRIVE_Q_TIMEOUT_MS  150U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* SensorTask → MotorTask 단방향 메일박스 (DriveInputs 값 복사 — 공유변수/뮤텍스 불요).
 * depth 2: full 메시지 + 선제제동 early 메시지 공존용 */
static osMessageQueueId_t driveQ = NULL;

/* ---- 디버그 미러 (SWD Live Expressions 등록 대상 — 구 main.c에서 이동, 이름 불변) ---- */
volatile uint16_t dbg_front = 0, dbg_left = 0, dbg_right = 0;   /* 필터 후 거리 cm */
volatile uint8_t  dbg_front_miss = 0;
volatile uint32_t dbg_loop_ms = 0;     /* 센서 사이클 1회 소요(ms) — 반응지연 감시 (통상 6~9, 최악 ~51) */

/* IMU(BNO055) 미러: heading/roll/pitch + 상태 */
volatile float    dbg_heading = 0.0f, dbg_roll = 0.0f, dbg_pitch = 0.0f;
volatile uint8_t  dbg_imu_ok = 0;      /* BNO055_Init 성공=1 (main.c USER CODE 2에서 기록) */
volatile uint8_t  dbg_imu_live = 0;    /* 지금 IMU 읽기 신뢰 가능(생존 게이트) — 0이면 제어는 거리-only 강등 */
volatile uint8_t  dbg_imu_calib = 0;   /* CALIB_STAT: [7:6]sys [5:4]gyr [3:2]acc [1:0]mag(IMU모드라 항상 0) */
volatile uint32_t dbg_imu_evt = 0;     /* INT(PA0/EXTI0) 발생 카운터 */

/* ---- RTOS 가시성 미러 (신규) ---- */
volatile uint32_t dbg_q_drop    = 0;   /* 큐 put 실패(가득참) 누적 — 0이어야 정상 */
volatile uint32_t dbg_q_timeout = 0;   /* MotorTask 큐 수신 timeout 누적 — 센서 파이프라인 침묵 횟수 */
volatile uint32_t dbg_hw_sensor = 0;   /* SensorTask 스택 high-water [words] — 64 미만이면 스택 증설 */
volatile uint32_t dbg_hw_motor  = 0;   /* MotorTask 스택 high-water [words] */

/* ---- 블루투스 원격제어 상태 (BluetoothTask 단일 writer, MotorTask reader) ----
 * 1바이트/int8 원자 접근 → 뮤텍스 불요. volatile = 태스크간 가시성(컴파일러 캐싱 차단) */
volatile uint8_t sys_power = 0;     /* 0=OFF(모터 강제정지) 1=ON. 부팅 OFF(안전) */
volatile uint8_t sys_mode  = 0;     /* 0=자율주행(Drive_Update) 1=수동(manual_* 인가) */
volatile int8_t  manual_left  = 0;  /* 수동 좌 듀티 [-100..100] (+전진 / -후진) */
volatile int8_t  manual_right = 0;  /* 수동 우 듀티 */
volatile uint8_t bt_rx_byte   = 0;  /* USART1 1바이트 IT 수신 버퍼 (RxCpltCallback → uartQ) */
volatile uint32_t dbg_bt_err  = 0;  /* USART1 에러콜백(ORE/FE/NE) 발생·복구 횟수 (SWD 관찰용) */

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
        Drive_Update(&din);            /* 호출당 정확히 1회 모터 명령 (din.now 기반 비블로킹) */
      }
    }
    else
    {
      dbg_q_timeout++;
      Car_Stop();           /* 센서 침묵 동안 안전 정지. refresh 생략 → 지속 시 IWDG 리셋 */
    }
    dbg_hw_motor = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
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
   * hist 윈도 = SIDE_MED_WIN(drive.h): 측면 median 샘플수 = '한번씩 튀는 값' 제거 강도 노브 */
  static uint16_t hist[3][SIDE_MED_WIN];
  for (int ch = 0; ch < 3; ch++)
      for (int i = 0; i < SIDE_MED_WIN; i++) hist[ch][i] = ULTRA_MAX_CM;
  static uint8_t  front_miss = 0;   /* 정면 에코 연속 미회신 (drive.c HOLD 판정 입력) */
  static uint8_t  side_miss[3] = {0,0,0};  /* 측면 ch1(L)/ch2(R) 연속 무에코 — 한계 초과 시 만료(ch0 미사용) */
  static uint8_t  imu_live = 0;     /* IMU 생존 게이트 */
  static uint8_t  imu_fail = 0;     /* 읽기 연속 실패 카운트 */
  static uint32_t t_imu_retry = 0;  /* 사망 후 재시도 타이머 */
  static uint32_t t_calib = 0;      /* CALIB_STAT 디버그 읽기 스로틀 */
  static uint32_t t_loop = 0;       /* 사이클 주기 측정 */
  static BNO055_Euler imu;          /* 최근 융합값 (read 실패 시 직전값 유지) */

  uint16_t f = ULTRA_MAX_CM, l = ULTRA_MAX_CM, r = ULTRA_MAX_CM;  /* 필터 출력 (l/r은 early 메시지가 직전값 재사용) */
  uint16_t d_snap[3];
  uint8_t  v_snap[3];

  Ultra_Init(osThreadGetId());      /* TIM3 IC start + 측정 완료 flag 수신자 등록 */
  imu_live = dbg_imu_ok;            /* Init 성공했을 때만 생존 시작 (실패 시 거리-only 고정) */

  for(;;)
  {
      uint32_t now = HAL_GetTick();
      dbg_loop_ms = now - t_loop;   /* 사이클 주기 미러 (반응지연 감시: 통상 6~9ms) */
      t_loop = now;

      /* 1) 정면 측정 → 필터 → 근접 시 'early' 메시지로 선제 제동 진입(코너 반응 앞당김).
       *    MotorTask가 높은 prio라 put 즉시 선점되어 cruise_run 재평가 → 사이드 측정(~12-30ms)을
       *    기다리지 않고 BRAKE→SPIN(정지+제자리 피벗) 진입 = 벽 앞에서 더 일찍 정지·회전.
       *    모터 명령 발행자는 MotorTask 하나로 유지 */
      v_snap[0] = Ultra_Measure(0, MEAS_WAIT_MS, &d_snap[0]);    /* front (TRIG PA5, ECHO CH1) */

      if (v_snap[0])             { front_miss = 0; }   /* 정지 결정은 drive.c HOLD가 담당 */
      else if (front_miss < 255) { front_miss++; }
      dbg_front_miss = front_miss;

      if (v_snap[0])   /* 유효 측정만 hist 푸시 → 무회신을 200(멀다)으로 덮는 fail-open 방지 */
      {
          for (int i = SIDE_MED_WIN - 1; i > 0; i--) hist[0][i] = hist[0][i - 1];
          hist[0][0] = d_snap[0];
      }
      f = min_n(hist[0], FRONT_MIN_WIN);   /* 정면은 최근 FRONT_MIN_WIN개 min (최근접, 지연0, 보수적) */

      if (Drive_GetState() == DS_CRUISE && f < FRONT_TURN_CM)
      {
          DriveInputs early;
          early.f          = f;
          early.l          = l;            /* 직전 사이클 필터값 — 제동 판정엔 f만 쓰임 */
          early.r          = r;
          early.f_valid    = v_snap[0];
          early.front_miss = front_miss;
          early.heading    = imu.heading;
          early.imu_live   = imu_live;
          early.now        = HAL_GetTick();
          if (osMessageQueuePut(driveQ, &early, 0U, 0U) != osOK) { dbg_q_drop++; }
      }

      /* 2) 측면 측정 (순차 발사 — 크로스토크 제거) + 필터 */
      v_snap[1] = Ultra_Measure(1, SIDE_WAIT_MS, &d_snap[1]);    /* left  (TRIG PA4, ECHO CH2) */
      v_snap[2] = Ultra_Measure(2, SIDE_WAIT_MS, &d_snap[2]);    /* right (TRIG PA3, ECHO CH3) */

      for (int ch = 1; ch < 3; ch++)
      {
          if (v_snap[ch])
          {
              side_miss[ch] = 0;
              for (int i = SIDE_MED_WIN - 1; i > 0; i--) hist[ch][i] = hist[ch][i - 1];
              hist[ch][0] = d_snap[ch];
          }
          else if (side_miss[ch] < 255)
          {
              /* 연속 무에코 SIDE_FAIL_LIMIT 도달 → hist를 트임(MAX)으로 비움 → median이 자동 ULTRA_MAX_CM.
                 stale 근접값 무한유지 차단(코너 오조향/오회피 방지). 에코 복귀 시 side_miss=0으로 즉시 재추종 */
              if (++side_miss[ch] >= SIDE_FAIL_LIMIT)
                  for (int i = 0; i < SIDE_MED_WIN; i++) hist[ch][i] = ULTRA_MAX_CM;
          }
      }
      l = median_n(hist[1], SIDE_MED_WIN);   /* 측면은 median (스파이크 제거 — 윈도=SIDE_MED_WIN, 조향 안정성) */
      r = median_n(hist[2], SIDE_MED_WIN);
      dbg_front = f; dbg_left = l; dbg_right = r;

      /* 3) IMU 샘플 (생존 게이트): 정상 시 매 사이클 heading 갱신. 연속 IMU_FAIL_LIMIT회 실패 →
       *    사망 선언 후 IMU_RETRY_MS 주기로만 재시도 — 죽은 IMU가 사이클마다 I2C timeout(10ms×2)을
       *    태워 반응지연 키우는 것 차단. CalibStatus는 디버그 전용이라 CALIB_POLL_MS 스로틀. */
      if (imu_live)
      {
          if (BNO055_ReadEuler(&imu))
          {
              imu_fail = 0;
              dbg_heading = imu.heading;
              dbg_roll    = imu.roll;
              dbg_pitch   = imu.pitch;
          }
          else if (++imu_fail >= IMU_FAIL_LIMIT)
          {
              imu_live = 0;
              t_imu_retry = now;
          }
          if ((now - t_calib) >= CALIB_POLL_MS)
          {
              t_calib = now;
              dbg_imu_calib = BNO055_ReadCalibStatus();
          }
      }
      else if (dbg_imu_ok && (now - t_imu_retry) >= IMU_RETRY_MS)
      {
          /* Init 성공 이력 있을 때만 재시도 — Init 실패 상태의 garbage 읽기로 살아있다고
           * 오판하는 것 방지. 버스 wedge는 bno_rd 내부 복구(10회 실패→9클럭)와 협력 */
          t_imu_retry = now;
          if (BNO055_ReadEuler(&imu)) { imu_live = 1; imu_fail = 0; }
      }
      dbg_imu_live = imu_live;

      /* 4) 제어 입력 스냅샷 → 큐 (값 복사 — MotorTask가 IWDG refresh + Drive_Update) */
      DriveInputs din;
      din.f          = f;
      din.l          = l;
      din.r          = r;
      din.f_valid    = v_snap[0];
      din.front_miss = front_miss;
      din.heading    = imu.heading;   /* imu_live=0이면 drive.c가 무시(거리-only 강등) */
      din.imu_live   = imu_live;
      din.now        = HAL_GetTick();
      if (osMessageQueuePut(driveQ, &din, 0U, 0U) != osOK) { dbg_q_drop++; }

      dbg_hw_sensor = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
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
        dbg_imu_evt++;
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
 * (RxCpltCallback의 "무조건 재무장" 패턴과 일관). dbg_bt_err = SWD 관찰용 카운터. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        dbg_bt_err++;
        __HAL_UART_CLEAR_OREFLAG(&huart1);   /* SR read + DR read로 ORE 해제 */
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&bt_rx_byte, 1);
    }
}

/* USER CODE END Application */

