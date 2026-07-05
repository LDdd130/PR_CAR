/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   SG-207 포토인터럽터 휠 엔코더 ×2 — TIM2 CH1(PA15)/CH2(PB3) 입력캡처 T법 속도 측정
  *          아키텍처: PR_CAR_Phase1_Architecture.md §3.2 (+§1.3 B안 계열: TIM2 32-bit)
  *          - T법(펄스 간 주기, 1µs): 저PPR(20슬롯)에서 M법 양자화 잡음 회피 — 사실상 유일해
  *          - TIM2는 32-bit 카운터 → 랩 71.6분: 오버플로 확장/UIF 보정 로직 자체가 불필요
  *          - 0속 판정: 무에지 ENC_ZERO_MS → v=0 (T법 "정지하면 에지가 영원히 없다" 맹점 보완)
  *          - 방향: 단채널(쿼드러처 아님) → 부호는 motor.c 최근 명령 방향(motor_dir_*) 채택
  *
  *          TIM2 베이스/GPIO(AF1)/NVIC(prio5)/IRQ핸들러 = CubeMX 생성(tim.c MX_TIM2_Init +
  *          stm32f4xx_it.c). 카운터는 main.c가 delay_us 타임베이스로 start(free-run 공유).
  *          이 모듈은 캡처 채널 IT 기동 + ISR 기록 + 속도 환산만 담당.
  *          ⚠ TIM2 카운터 리셋 금지 — delay_us(delay.c)와 엔코더가 같은 free-run CNT를 공유.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* TIM_HandleTypeDef */

/* ---- 휠/디스크 제원 (⚠ 가정값 — 실측 후 갱신: 아키텍처 §6.2 체크리스트) ---- */
#define ENC_WHEEL_DIAM_CM   6.5f    /* 휠 지름 [cm] (Ø65mm 가정) */
#define ENC_SLOTS           20.0f   /* 엔코더 디스크 슬롯 수 (20슬롯 가정) */
#define ENC_CM_PER_PULSE    (3.1415926f * ENC_WHEEL_DIAM_CM / ENC_SLOTS)  /* ≈1.021 cm/펄스 */

/* ---- 측정 파라미터 (§3.2) ---- */
#define ENC_ZERO_MS         100U    /* 무에지 이 시간 초과 → v=0 강제. 동시에 최소 분해 속도(≈10cm/s) 경계 */
#define ENC_MIN_PERIOD_US   2000U   /* 이보다 짧은 주기(>5m/s 상당)는 슬롯 채터/EMI 글리치 → 에지 무시 */
#define ENC_EMA_ALPHA       0.35f   /* 속도 EMA 새 샘플 가중치 (슬롯 가공 공차 잡음 평활) */

/* 채널↔바퀴 매핑 (배선: PA15=TIM2_CH1, PB3=TIM2_CH2).
 * ⚠ 좌/우 배선이 반대로 확인되면 여기 두 값만 교환 (encoder.c 분기가 이 매크로를 사용) */
#define ENC_LEFT   0U    /* 좌 휠 = PA15 = TIM2_CH1 가정 */
#define ENC_RIGHT  1U    /* 우 휠 = PB3  = TIM2_CH2 가정 */

void     Encoder_Init(void);                              /* CH1/CH2 캡처 IT 기동 (MX_TIM2_Init 이후 아무 때나) */
void     Encoder_OnCapture(TIM_HandleTypeDef *htim);      /* ISR: ultra.c HAL_TIM_IC_CaptureCallback의 TIM2 분기에서 호출 */
float    Encoder_SpeedCmps(uint8_t idx, uint32_t now_ms); /* 필터 속도 [cm/s, 크기]. ⚠ 내부 필터 상태 — MotorTask 단일 호출자 전용 */
uint32_t Encoder_Edges(uint8_t idx);                      /* 유효 에지 누적 (R0 배선 검증: 손 회전 시 증가 관측) */

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
