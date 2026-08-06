#include "delay.h"
#include "tim.h"

/* µs 지연 — TIM2 free-run(1µs, 32-bit) 기준 '차분' 대기.
 * 구현 배경: 구 타임베이스 TIM11이 CubeMX 재생성(TIM2 엔코더 추가)에서 제거됨 →
 * TIM2를 공용 타임베이스로 승계. TIM2는 휠 엔코더 캡처(encoder.c)와 공유이므로
 * ⚠ 카운터 리셋(SET_COUNTER(0)) 절대 금지 — 캡처 차분이 깨진다. diff 방식은 리셋 불요이고
 * uint32 wrap(71.6분)도 뺄셈으로 자동 보정된다. main.c가 부팅 시 HAL_TIM_Base_Start(&htim2). */
void delay_us(uint16_t us)
{
    uint32_t t0 = __HAL_TIM_GET_COUNTER(&htim2);
    while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - t0) < (uint32_t)us);
}
