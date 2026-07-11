/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   L298N differential-drive motor API
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Signed wheel duty in percent, clamped to [-100, 100]. Direction changes and
 * brake release keep EN/IN neutral for at least one PWM period. */
void Motor_SetWheels(int16_t left_pct, int16_t right_pct);
void Motor_Left(int8_t speed);
void Motor_Right(int8_t speed);

void Motor_Init(void);
void Car_Forward(uint8_t speed);
void Car_Backward(uint8_t speed);
void Car_PivotLeft(uint8_t outer, uint8_t inner);
void Car_PivotRight(uint8_t outer, uint8_t inner);
void Car_ArcLeft(uint8_t outer, uint8_t inner);
void Car_ArcRight(uint8_t outer, uint8_t inner);
void Car_Stop(void);   /* Coast: IN low, PWM 0. */
void Car_Brake(void);  /* L298N short brake: IN low, PWM 100%. */

/* Direction of the latest nonzero output actually applied, for single-channel
 * encoder telemetry. Stop, brake, and reversal-neutral frames keep the latch. */
extern volatile int8_t motor_dir_left;
extern volatile int8_t motor_dir_right;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
