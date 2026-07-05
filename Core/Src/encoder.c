/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   SG-207 포토인터럽터 휠 엔코더 ×2 — TIM2 CH1(PA15)/CH2(PB3) 입력캡처 속도 측정
  *
  *          설계 근거 (PR_CAR_Phase1_Architecture.md §3.2, §1.3 B안 계열):
  *          - TIM2는 STM32F411의 32-bit 타이머: 1µs tick에서 랩 71.6분 →
  *            16-bit(TIM3)안에 필수였던 오버플로 확장·UIF 경계 보정이 통째로 사라짐 (산술 단순)
  *          - 전용 타이머 = 전방 초음파(TIM3 CH1)와 캡처 인프라 완전 분리 — 상호 간섭 0
  *          - 상승 단일 에지: 슬롯 듀티 비대칭이라 양 에지를 쓰면 주기가 교대로 출렁임
  *          - ISR은 기록만(주기/에지수/시각) 하고 즉시 복귀 — 환산·필터는 MotorTask (아키텍처 D2)
  *
  *          역할 분담 (CubeMX 재생성 안전):
  *          - CubeMX 생성부: htim2 정의 + MX_TIM2_Init(PSC 100-1=1µs, ARR 32-bit, CH1/CH2 IC
  *            rising/ICFilter 8) + MspInit(GPIO PA15/PB3 AF1, NVIC prio5) + TIM2_IRQHandler(it.c)
  *          - main.c: HAL_TIM_Base_Start(&htim2) — delay_us 타임베이스 겸용 free-run 기동
  *          - 이 모듈: 캡처 채널 IT 기동(Encoder_Init) + 에지 기록 ISR + 속도 환산/필터
  *          ⚠ TIM2 카운터 리셋 금지 — delay_us(delay.c)와 엔코더가 같은 free-run CNT 공유.
  *            (구 delay.c의 SET_COUNTER(0) 방식은 TIM11 전용이었다 — TIM2에서는 diff 방식만)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "encoder.h"
#include "tim.h"      /* htim2 (CubeMX 생성) */
#include "FreeRTOS.h"
#include "task.h"     /* taskENTER/EXIT_CRITICAL — 3워드 스냅샷 정합 (TIM2 IRQ prio5 마스킹) */

/* ---- ISR 기록 상태 (엔코더 ISR 단일 작성자, MotorTask는 임계구역 스냅샷 판독) ---- */
typedef struct
{
    volatile uint32_t period_us;   /* 최근 유효 에지 간 주기 [µs] (32-bit 카운터 직접 차분) */
    volatile uint32_t edges;       /* 유효 에지 누적 (seq — 새 에지 판별용) */
    volatile uint32_t last_ms;     /* 최근 유효 에지 시각 [HAL_GetTick] — 0속 타임아웃 기준 */
    uint32_t last_cap;             /* 최근 에지의 32-bit 캡처값 (ISR 전용) */
    uint8_t  has_prev;             /* 첫 에지 여부 (첫 에지는 주기 산출 불가) */
} EncChannel;

static EncChannel enc[2];          /* [ENC_LEFT]=CH1/PA15, [ENC_RIGHT]=CH2/PB3 */

/* ---- 태스크측 필터 상태 (⚠ 단일 호출자 = MotorTask 전제 — 비보호) ---- */
typedef struct
{
    uint32_t seq;    /* 마지막으로 반영한 edges */
    float    v;      /* EMA 필터 속도 [cm/s] */
} EncFilter;

static EncFilter flt[2];

/* 캡처 채널 IT 기동. 베이스/GPIO/NVIC는 MX_TIM2_Init(+MspInit)이, 카운터 start는 main.c가
 * 이미 수행 — 여기서는 CC1/CC2 활성만. 호출 시점 자유(SensorTask 초기화부) */
void Encoder_Init(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC1OF | TIM_FLAG_CC2 | TIM_FLAG_CC2OF);
    (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

/* TIM2 CC1/CC2 캡처 (ultra.c HAL_TIM_IC_CaptureCallback의 TIM2 분기에서 호출 — ISR 문맥).
 * 32-bit 캡처 직접 차분 — 오버플로 보정 불요 (uint32 wrap 산술이 자동 처리) */
void Encoder_OnCapture(TIM_HandleTypeDef *htim)
{
    uint8_t  idx;
    uint32_t cap;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        idx = ENC_LEFT;                                            /* PA15 = CH1 */
        cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        idx = ENC_RIGHT;                                           /* PB3 = CH2 */
        cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    else return;

    EncChannel *e = &enc[idx];
    if (e->has_prev)
    {
        uint32_t per = cap - e->last_cap;   /* 32-bit wrap-safe 차분 = 주기 [µs] */
        if (per >= ENC_MIN_PERIOD_US)
        {
            e->period_us = per;
            e->edges++;
            e->last_cap  = cap;
            e->last_ms   = HAL_GetTick();
        }
        /* per < MIN = 물리 불가(>5m/s 상당) 채터/EMI 글리치 → 에지 자체를 무시.
         * last_cap을 유지하므로 다음 진짜 에지가 합산 주기로 잡힌다 (주기 왜곡 없음).
         * HW ICFilter(8)와 이중 방어 */
    }
    else
    {
        e->has_prev = 1U;
        e->last_cap = cap;
        e->last_ms  = HAL_GetTick();
    }
}

/* 필터된 휠 속도 [cm/s, 크기]. 방향 부호는 호출측이 motor_dir_*로 곱한다
 * (단채널 = 방향 정보 없음 — §3.2 명문화된 한계) */
float Encoder_SpeedCmps(uint8_t idx, uint32_t now_ms)
{
    if (idx > 1U) return 0.0f;

    uint32_t per, edges, last_ms;
    taskENTER_CRITICAL();            /* period/edges/last_ms 3워드 정합 스냅샷 (TIM2 IRQ prio5 마스킹) */
    per     = enc[idx].period_us;
    edges   = enc[idx].edges;
    last_ms = enc[idx].last_ms;
    taskEXIT_CRITICAL();

    EncFilter *s = &flt[idx];

    if (edges != s->seq)
    {
        s->seq = edges;
        /* 정지 공백 후 첫 에지의 주기는 '멈춰 있던 시간'이라 속도가 아님 → 환산 제외.
         * ZERO_MS(100ms) 이상 주기는 어차피 0속 분해 한계(≈10cm/s) 밖 */
        if ((per != 0U) && (per < ((uint32_t)ENC_ZERO_MS * 1000U)))
        {
            float v = ENC_CM_PER_PULSE * 1.0e6f / (float)per;
            s->v = (s->v <= 0.01f) ? v : (s->v + (ENC_EMA_ALPHA * (v - s->v)));
        }
    }
    else if ((now_ms - last_ms) > (uint32_t)ENC_ZERO_MS)
    {
        s->v = 0.0f;   /* 무에지 타임아웃 → 정지 확정 (T법 맹점 보완) */
    }

    return s->v;
}

uint32_t Encoder_Edges(uint8_t idx)
{
    return (idx <= 1U) ? enc[idx].edges : 0U;
}
