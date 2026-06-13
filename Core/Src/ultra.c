/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultra.c
  * @brief   HC-SR04 3채널 측정 구현 — TIM3 입력캡처 ISR + 순차 발사 + RTOS flag 대기
  *          (구 main.c의 측정 경로를 모듈로 분리: 캡처 콜백/TRIG 테이블/Ultra_MeasureCh)
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
static volatile uint16_t IC_Value1[3]   = {0};
static volatile uint16_t IC_Value2[3]   = {0};
static volatile uint16_t echoTime[3]    = {0};
static volatile uint8_t  captureFlag[3] = {0};
static volatile uint16_t distance[3]    = {0};
static volatile uint8_t  meas_valid[3]  = {0};   /* ISR이 완전한 에코(상승+하강) 측정 완료 시 1 */

/* 유효 측정 완료 통지 대상(SensorTask). NULL이면 통지 생략 — 태스크 기동 전 미아 에지 안전 */
static osThreadId_t s_waiter = NULL;

/* === 센서별 TRIG 핀 + TIM3 채널 매핑 (인덱스 0=front, 1=left, 2=right) ===
 * 분리 TRIG(front=PA5, left=PA4, right=PA3)로 '순차 발사' → 3센서 동시발사 크로스토크 원천 제거.
 * ECHO는 TIM3 CH1/CH2/CH3(PA6/PA7/PB0) 고정. */
static GPIO_TypeDef* const ULTRA_TRIG_PORT[3] = { TRIG_FRONT_GPIO_Port, TRIG_LEFT_GPIO_Port, TRIG_RIGHT_GPIO_Port };
static const uint16_t      ULTRA_TRIG_PIN [3] = { TRIG_FRONT_Pin,       TRIG_LEFT_Pin,       TRIG_RIGHT_Pin };
static const uint32_t      ULTRA_CH       [3] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };
static const uint32_t      ULTRA_CC_IT    [3] = { TIM_IT_CC1,    TIM_IT_CC2,    TIM_IT_CC3 };
static const uint32_t      ULTRA_CC_FLAG  [3] = { TIM_FLAG_CC1 | TIM_FLAG_CC1OF,
                                                  TIM_FLAG_CC2 | TIM_FLAG_CC2OF,
                                                  TIM_FLAG_CC3 | TIM_FLAG_CC3OF };
/* ECHO 핀 (TIM3 CH1/2/3 = PA6/PA7/PB0): AF 모드여도 IDR 레벨 읽기 가능 — 발사 전 잔류-high 가드용 */
static GPIO_TypeDef* const ULTRA_ECHO_PORT[3] = { ECHO_TIM3_CH1_GPIO_Port, ECHO_TIM3_CH2_GPIO_Port, ECHO_TIM3_CH3_GPIO_Port };
static const uint16_t      ULTRA_ECHO_PIN [3] = { ECHO_TIM3_CH1_Pin,       ECHO_TIM3_CH2_Pin,       ECHO_TIM3_CH3_Pin };

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;   /* IC 콜백은 전 타이머 공용 — 추후 타이머 추가 대비 */

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if(captureFlag[0] == 0)
        {
            IC_Value1[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
            captureFlag[0] = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if(captureFlag[0] == 1)
        {
            IC_Value2[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

            /* 16-bit 무부호 뺄셈: 타이머 wrap(65535→0)을 mod-65536으로 자동 보정.
             * 이전 코드의 +1 누락 오차와 IC_Value2==IC_Value1 시 echoTime 정체 버그를 함께 해결. */
            echoTime[0] = (uint16_t)(IC_Value2[0] - IC_Value1[0]);

            /* d==0(<58µs)은 물리 불가(HC-SR04 최소 2cm=116µs) = 글리치 에지 → invalid 처리.
             * '멀다(200)'로 두면 hist를 fail-open 오염 — meas_valid 미설정으로 front_miss 경로에 맡김 */
            uint16_t d0 = echoTime[0] / 58;
            if (d0 != 0U)
            {
                distance[0] = (d0 > ULTRA_MAX_CM) ? ULTRA_MAX_CM : d0;
                meas_valid[0] = 1;
                if (s_waiter != NULL) osThreadFlagsSet(s_waiter, 1UL << 0);   /* 대기 태스크 즉시 wake */
            }
            captureFlag[0] = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
        }
    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if(captureFlag[1] == 0)
        {
            IC_Value1[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
            captureFlag[1] = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if(captureFlag[1] == 1)
        {
            IC_Value2[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

            /* 16-bit 무부호 뺄셈: wrap 자동 보정 + V2==V1 정체 버그 해결 (CH1과 동일) */
            echoTime[1] = (uint16_t)(IC_Value2[1] - IC_Value1[1]);

            uint16_t d1 = echoTime[1] / 58;
            if (d1 != 0U)   /* CH1과 동일: 글리치(0) → invalid */
            {
                distance[1] = (d1 > ULTRA_MAX_CM) ? ULTRA_MAX_CM : d1;
                meas_valid[1] = 1;
                if (s_waiter != NULL) osThreadFlagsSet(s_waiter, 1UL << 1);
            }
            captureFlag[1] = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
        }
    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        if(captureFlag[2] == 0)
        {
            IC_Value1[2] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
            captureFlag[2] = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if(captureFlag[2] == 1)
        {
            IC_Value2[2] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);

            /* 16-bit 무부호 뺄셈: wrap 자동 보정 + V2==V1 정체 버그 해결 (CH1과 동일) */
            echoTime[2] = (uint16_t)(IC_Value2[2] - IC_Value1[2]);

            uint16_t d2 = echoTime[2] / 58;
            if (d2 != 0U)   /* CH1과 동일: 글리치(0) → invalid */
            {
                distance[2] = (d2 > ULTRA_MAX_CM) ? ULTRA_MAX_CM : d2;
                meas_valid[2] = 1;
                if (s_waiter != NULL) osThreadFlagsSet(s_waiter, 1UL << 2);
            }
            captureFlag[2] = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC3);
        }
    }
}

void Ultra_Init(osThreadId_t waiter)
{
    s_waiter = waiter;
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
}

/* 센서 1개 순차 측정: 해당 채널 잔여상태 정리 → 그 센서 TRIG만 10us 펄스 → 에코 회신까지
 * osThreadFlagsWait 블로킹 대기(구 busy-wait 대체 — 대기 중 CPU는 다른 태스크/idle로).
 * 에코 오면 ISR flag로 즉시 wake(근접=수ms), 무에코/글리치만 wait_ms 상한까지 대기. */
uint8_t Ultra_Measure(uint8_t ch, uint32_t wait_ms, uint16_t *out_cm)
{
    distance[ch]   = ULTRA_MAX_CM;   /* 기본 멀다 (에코 오면 ISR이 실제값으로 덮음) */
    meas_valid[ch] = 0;

    /* HC-SR04 무에코 잔류 가드: 직전 사이클이 무에코였다면 ECHO가 아직 high(~38ms 유지)일 수 있고
     * 그 동안의 TRIG는 무시됨 → wait_ms 헛대기 대신 이번 사이클 즉시 skip(무효 처리).
     * 기존 'miss 2연속(페어)' 현상이 1회 fast-skip으로 줄어 fail-safe 반응이 빨라짐 */
    if (HAL_GPIO_ReadPin(ULTRA_ECHO_PORT[ch], ULTRA_ECHO_PIN[ch]) == GPIO_PIN_SET)
    {
        *out_cm = ULTRA_MAX_CM;
        return 0;
    }

    /* 해당 채널만 잔여 캡처 상태 정리 (IT off → 플래그/극성 리셋 → CCxIF/CCxOF 클리어)
     * + 직전 사이클 늦은 에코가 남긴 stale thread flag 제거 (IT off 후라 경쟁 없음) */
    __HAL_TIM_DISABLE_IT(&htim3, ULTRA_CC_IT[ch]);
    captureFlag[ch] = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, ULTRA_CH[ch], TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(&htim3, ULTRA_CC_FLAG[ch]);
    osThreadFlagsClear(1UL << ch);

    /* TRIG 10us 펄스 + 해당 채널 캡처 IT 활성 */
    HAL_GPIO_WritePin(ULTRA_TRIG_PORT[ch], ULTRA_TRIG_PIN[ch], GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(ULTRA_TRIG_PORT[ch], ULTRA_TRIG_PIN[ch], GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim3, ULTRA_CC_IT[ch]);

    /* 적응형 대기: 유효 측정 완료 시 ISR flag → 즉시 복귀, 무에코/글리치 → timeout */
    (void)osThreadFlagsWait(1UL << ch, osFlagsWaitAny, wait_ms);

    /* ISR 공유값 원자적 스냅샷 (timeout 직후 늦은 에코 edge로부터 일관성 보호).
     * taskENTER_CRITICAL = BASEPRI를 syscall 경계(5)로 → TIM3(prio 5) 마스킹 (구 PRIMASK 블록 대체) */
    uint8_t  v;
    uint16_t d;
    taskENTER_CRITICAL();
    v = meas_valid[ch];
    d = distance[ch];
    taskEXIT_CRITICAL();

    *out_cm = d;
    return v;
}

/* N-샘플 중앙값 (스파이크/크로스토크 제거). n개 중 (n-1)/2개까지 튀어도 거름 — 윈도 클수록 강건·둔감.
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
