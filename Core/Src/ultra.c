/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultra.c
  * @brief   HC-SR04 전방 1채널 측정 구현 — TIM3 CH1 입력캡처 ISR + RTOS flag 대기
  *          (측면 좌/우 초음파(구 CH2/CH3)는 VL53L0X ToF로 이관 — vl53l0x.c.
  *           채널 테이블/순차발사 로직은 전방 단일화로 제거)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ultra.h"
#include "main.h"   /* TRIG/ECHO 핀 매크로 (CubeMX 생성) */
#include "tim.h"    /* htim3 */
#include "delay.h"  /* delay_us — TRIG 10µs 펄스 (TIM11, main에서 start됨) */
#include "drive.h"  /* ULTRA_MAX_CM */
#include "FreeRTOS.h"
#include "task.h"   /* taskENTER/EXIT_CRITICAL */

/* ISR(HAL_TIM_IC_CaptureCallback)과 SensorTask가 공유 → volatile 필수 (컴파일러 캐싱/경쟁 방지) */
static volatile uint16_t IC_Value1   = 0;
static volatile uint16_t IC_Value2   = 0;
static volatile uint16_t echoTime    = 0;
static volatile uint8_t  captureFlag = 0;
static volatile uint16_t distance    = 0;
static volatile uint8_t  meas_valid  = 0;   /* ISR이 완전한 에코(상승+하강) 측정 완료 시 1 */

/* 유효 측정 완료 통지 대상(SensorTask). NULL이면 통지 생략 — 태스크 기동 전 미아 에지 안전 */
static osThreadId_t s_waiter = NULL;

#define ULTRA_FLAG_FRONT (1UL << 0)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;   /* IC 콜백은 전 타이머 공용 — 추후 타이머 추가 대비 */
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    if (captureFlag == 0)
    {
        IC_Value1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        captureFlag = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (captureFlag == 1)
    {
        IC_Value2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

        /* 16-bit 무부호 뺄셈: 타이머 wrap(65535→0)을 mod-65536으로 자동 보정. */
        echoTime = (uint16_t)(IC_Value2 - IC_Value1);

        /* d==0(<58µs)은 물리 불가(HC-SR04 최소 2cm=116µs) = 글리치 에지 → invalid 처리.
         * '멀다(MAX)'로 두면 hist를 fail-open 오염 — meas_valid 미설정으로 front_miss 경로에 맡김 */
        uint16_t d0 = echoTime / 58;
        if (d0 != 0U)
        {
            distance = (d0 > ULTRA_MAX_CM) ? ULTRA_MAX_CM : d0;
            meas_valid = 1;
            if (s_waiter != NULL) osThreadFlagsSet(s_waiter, ULTRA_FLAG_FRONT);   /* 대기 태스크 즉시 wake */
        }
        captureFlag = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    }
}

void Ultra_Init(osThreadId_t waiter)
{
    s_waiter = waiter;
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

/* 전방 측정: 잔여상태 정리 → TRIG 10us 펄스 → 에코 회신까지 osThreadFlagsWait 블로킹 대기.
 * 에코 오면 ISR flag로 즉시 wake(근접=수ms), 무에코/글리치만 wait_ms 상한까지 대기. */
uint8_t Ultra_Measure(uint32_t wait_ms, uint16_t *out_cm)
{
    distance   = ULTRA_MAX_CM;   /* 기본 멀다 (에코 오면 ISR이 실제값으로 덮음) */
    meas_valid = 0;

    /* HC-SR04 무에코 잔류 가드: 직전 사이클이 무에코였다면 ECHO가 아직 high(~38ms 유지)일 수 있고
     * 그 동안의 TRIG는 무시됨 → wait_ms 헛대기 대신 이번 사이클 즉시 skip(무효 처리). */
    if (HAL_GPIO_ReadPin(ECHO_TIM3_CH1_GPIO_Port, ECHO_TIM3_CH1_Pin) == GPIO_PIN_SET)
    {
        *out_cm = ULTRA_MAX_CM;
        return 0;
    }

    /* 잔여 캡처 상태 정리 (IT off → 플래그/극성 리셋 → CC1IF/CC1OF 클리어)
     * + 직전 사이클 늦은 에코가 남긴 stale thread flag 제거 (IT off 후라 경쟁 없음) */
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    captureFlag = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    osThreadFlagsClear(ULTRA_FLAG_FRONT);

    /* TRIG 10us 펄스 + 캡처 IT 활성 */
    HAL_GPIO_WritePin(TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);

    /* 적응형 대기: 유효 측정 완료 시 ISR flag → 즉시 복귀, 무에코/글리치 → timeout */
    (void)osThreadFlagsWait(ULTRA_FLAG_FRONT, osFlagsWaitAny, wait_ms);

    /* ISR 공유값 원자적 스냅샷 (timeout 직후 늦은 에코 edge로부터 일관성 보호).
     * taskENTER_CRITICAL = BASEPRI를 syscall 경계(5)로 → TIM3(prio 5) 마스킹 */
    uint8_t  v;
    uint16_t d;
    taskENTER_CRITICAL();
    v = meas_valid;
    d = distance;
    taskEXIT_CRITICAL();

    *out_cm = d;
    return v;
}

/* N-샘플 중앙값 (스파이크 제거). n개 중 (n-1)/2개까지 튀어도 거름 — 윈도 클수록 강건·둔감.
 * 삽입정렬 후 중앙 인덱스 반환. n은 작음(3~7)이라 정렬 비용 무시 가능. */
uint16_t median_n(const uint16_t *a, uint8_t n)
{
    uint16_t b[16];
    if (n > 16) n = 16;
    for (uint8_t i = 0; i < n; i++) b[i] = a[i];
    for (uint8_t i = 1; i < n; i++)
    {
        uint16_t k = b[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && b[j] > k) { b[j + 1] = b[j]; j--; }
        b[j + 1] = k;
    }
    return b[n / 2];
}

/* N-샘플 최소값 (가장 가까운 최근값). 정면 정지 판정용 — median과 달리 지연 0, 항상 보수적(빨리 멈춤) */
uint16_t min_n(const uint16_t *a, uint8_t n)
{
    uint16_t m = a[0];
    for (uint8_t i = 1; i < n; i++) if (a[i] < m) m = a[i];
    return m;
}
