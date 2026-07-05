/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bno055.h
  * @brief   GY-BNO055 9축 IMU 드라이버 (I2C1, IMU 융합모드 = gyro+accel, 지자기 제외)
  *          heading = Z축 yaw = 차량 xy평면 진행각(상대각). 절대 방위 아님(모터 자기간섭 회피).
  *          - 주소: ADD=GND → 0x28 (HAL 8-bit = 0x50)
  *          - 배선: SCL=PB8, SDA=PB9, INT=PA0(EXTI0), RST=PB1(IMU_RST, active-low)
  *          - 블로킹 HAL I2C (timeout 10ms) → 버스 stall 시 모터 장시간 정지 방지
  *          ⚠ 빌드 전제: CubeMX에서 I2C1(PB8/PB9) + PA0(IMU_INT) + PB1(IMU_RST)
  *             생성 필요. (hi2c1, IMU_RST_Pin 등은 CubeMX 생성 심볼)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __BNO055_H__
#define __BNO055_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* ADD 핀=GND → 7-bit 0x28 → HAL 8-bit 주소 */
#define BNO055_I2C_ADDR_8BIT   (0x28U << 1)   /* = 0x50. ADD=3V3면 (0x29<<1)=0x52 */

/* IMU 융합 출력 (단위: degree, UNIT_SEL 기본값 기준) */
typedef struct
{
    float heading;   /* Z축 yaw, 0~360° (차량 xy평면 진행각 — 제어가 쓰는 값) */
    float roll;      /* ° (직진 판단 미사용) */
    float pitch;     /* ° (직진 판단 미사용) */
} BNO055_Euler;

/* RST 핀 토글로 하드웨어 리셋 (Low 1ms → High → POR 부팅 ~650ms 대기) */
void    BNO055_HardReset(void);

/* 초기화: 리셋 → CHIP_ID(0xA0) 확인 → CONFIG → NORMAL → IMU 모드. 성공 시 true */
bool    BNO055_Init(void);

/* 오일러각 6바이트 읽기. 성공 시 *e 갱신+true, 실패 시 *e 미변경+false(직전값 유지는 호출측) */
bool    BNO055_ReadEuler(BNO055_Euler *e);

/* CALIB_STAT(0x35): [7:6]=sys [5:4]=gyr [3:2]=acc [1:0]=mag, 각 0~3(3=완료).
 * IMU 모드는 지자기 미사용 → mag 비트[1:0]는 항상 0(정상). heading 안정은 gyr 캘리브에 의존.
 * 디버그 전용(제어는 상대각만 써서 미캘리브도 동작). 실패 시 0 반환 */
uint8_t BNO055_ReadCalibStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __BNO055_H__ */
