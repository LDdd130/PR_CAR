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

  /* USER CODE BEGIN RTOS_QUEUES */
  driveQ = osMessageQueueNew(2, sizeof(DriveInputs), NULL);
  /* 생성 실패(NULL) 시 put/get이 전부 에러 → MotorTask가 Car_Stop + refresh 생략 → IWDG 리셋.
   * heap_4 15KB에 큐 ~수십B라 실패 가능성 사실상 없음 — 별도 처리 불요 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartDefaultTask, NULL, &MotorTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask02, NULL, &SensorTask_attributes);

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
  for(;;)
  {
    if (osMessageQueueGet(driveQ, &din, NULL, DRIVE_Q_TIMEOUT_MS) == osOK)
    {
      HAL_IWDG_Refresh(&hiwdg);
      Drive_Update(&din);   /* 호출당 정확히 1회 모터 명령 (상태머신은 din.now 기반 비블로킹) */
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

      /* 1) 정면 측정 → 필터 → 필요 시 'early' 메시지로 선제 제동.
       *    구 main.c의 직접 Car_Brake 힌트 대체: MotorTask가 높은 prio라 put 즉시 선점되어
       *    brake_enter — 사이드 측정(~12-30ms) 동안 제동이 걸려있는 원래 의도 보존,
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

      if (Drive_GetState() == DS_CRUISE && f < FRONT_STOP_CM)
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
              for (int i = SIDE_MED_WIN - 1; i > 0; i--) hist[ch][i] = hist[ch][i - 1];
              hist[ch][0] = d_snap[ch];
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

/* USER CODE END Application */

