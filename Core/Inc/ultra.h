/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultra.h
  * @brief   HC-SR04 3채널 초음파 측정 모듈 (TIM3 입력캡처 + 분리 TRIG 순차 발사)
  *          - 채널 인덱스: 0=front(TRIG PA5/ECHO TIM3_CH1), 1=left(PA4/CH2), 2=right(PA3/CH3)
  *          - RTOS 전용: 에코 대기 = osThreadFlagsWait. TIM3 IC ISR이 '유효 측정 완료' 시
  *            waiter 태스크로 (1<<ch) flag set — TIM3 prio 5 = configMAX_SYSCALL 경계라 합법.
  *            무에코/글리치는 wait_ms timeout으로 끝남 (구 busy-wait와 동일 의미).
  *          - Ultra_Init은 SensorTask 기동 시 1회 (IC Start까지 여기서 — pre-kernel ISR 창 제거)
  *          - 측정 상한/노브(ULTRA_MAX_CM, MEAS/SIDE_WAIT_MS)는 drive.h 단일 블록 유지
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __ULTRA_H__
#define __ULTRA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* waiter(측정 호출 태스크) 등록 + TIM3 IC CH1/2/3 start. 루프 진입 전 1회 호출 */
void Ultra_Init(osThreadId_t waiter);

/* 센서 1개 순차 측정 (블로킹: 에코 회신 즉시 wake, 무에코/글리치만 wait_ms 상한).
 * 한 번에 1개만 발사하므로 옆 센서 에코 혼입(크로스토크) 없음.
 * 반환 1=유효 측정(*out_cm 실제값), 0=무에코/글리치/잔류-high skip(*out_cm=ULTRA_MAX_CM) */
uint8_t Ultra_Measure(uint8_t ch, uint32_t wait_ms, uint16_t *out_cm);

/* N-샘플 필터 헬퍼 (hist 관리는 호출측 — freertos.c SensorTask, 윈도 = drive.h SIDE_MED_WIN) */
uint16_t median_n(const uint16_t *a, uint8_t n);  /* 중앙값: 스파이크 제거 (측면 조향 안정성). n 홀수 권장 */
uint16_t min_n(const uint16_t *a, uint8_t n);     /* 최소값: 최근접·지연0·보수적 (정면 정지판정) */

#ifdef __cplusplus
}
#endif

#endif /* __ULTRA_H__ */
