/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vl53l0x.h
  * @brief   VL53L0X(TOF2000C) 경량 드라이버 — STM32 HAL I2C, 폴링 연속측정
  *          - ST 공식 API(수천 줄) 대신 검증된 최소 초기화 시퀀스(Pololu 계열) 이식:
  *            SPAD 관리 + 기본 튜닝 레지스터 + VHV/Phase ref 캘리브레이션 + back-to-back 연속모드
  *          - 측정 타이밍 버짓은 디폴트(~33ms) 유지 → 새 샘플 ~30Hz. 제어 루프는
  *            PollRangeMM()로 논블로킹 폴링(새 샘플 없으면 0 반환, I2C 대기 없음)
  *          - 멀티 센서: XSHUT로 1개씩 깨워 SetAddress()로 주소 분리 (freertos.c Init_ToF_Sensors)
  *          - 스레드 모델: 모든 호출은 SensorTask 단일 문맥 (BNO055와 I2C1 공유 — 자연 직렬화)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __VL53L0X_H__
#define __VL53L0X_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define VL53L0X_ADDR_DEFAULT_8BIT  0x52U  /* 7-bit 0x29 << 1 (전원 인가 시 항상 이 주소로 복귀) */
#define VL53L0X_I2C_TIMEOUT_MS     10U    /* 단일 레지스터 트랜잭션 상한. 센서 사망 시 루프 지연 제한 */

typedef struct
{
    I2C_HandleTypeDef *hi2c;      /* 공유 버스 핸들 (hi2c1) */
    uint8_t            addr;      /* 8-bit I2C 주소 (HAL 규약) */
    uint8_t            stop_variable; /* init 때 칩에서 읽는 내부값 — StartContinuous 시퀀스에 필요 */
} VL53L0X;

/* 전체 초기화 (XSHUT high + 부팅 ~1.2ms 후 호출). 반환 1=성공 0=실패(I2C 무응답/ID 불일치) */
uint8_t VL53L0X_Init(VL53L0X *dev);

/* I2C 주소 변경 (레지스터 0x8A). new_addr_8bit = 8-bit 주소(예: 0x60). 성공 시 dev->addr 갱신.
 * 주소는 센서 내부 RAM — XSHUT low/전원 리셋 시 0x52로 복귀하므로 부팅마다 재설정 필요 */
uint8_t VL53L0X_SetAddress(VL53L0X *dev, uint8_t new_addr_8bit);

/* 측정 타이밍 버짓 변경 [us] (= 연속모드 샘플 주기). 20000 = ST 고속 프로파일(정확도 소폭 희생).
 * Init 후·StartContinuous 전 호출. 실패해도 디폴트(~33ms)로 동작 — 치명 아님 */
uint8_t VL53L0X_SetTimingBudget(VL53L0X *dev, uint32_t budget_us);

/* back-to-back 연속측정 시작 (버짓 소진 즉시 다음 측정 자동 개시) */
uint8_t VL53L0X_StartContinuous(VL53L0X *dev);

/* 논블로킹 폴링: 새 측정치 있으면 *out_mm에 쓰고 1, 아직이면 0, I2C 에러면 -1.
 * out-of-range는 칩이 8190/8191mm를 반환 — 상한 처리(캡)는 호출측 몫 */
int8_t VL53L0X_PollRangeMM(VL53L0X *dev, uint16_t *out_mm);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_H__ */
