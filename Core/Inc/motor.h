/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   L298N 모터 드라이버 제어 (전진/후진/좌우/속도)
  *          - 우측 모터: ENA=PB6=TIM4_CH1, IN1=PB12(전진), IN2=PB13(후진)
  *          - 좌측 모터: ENB=PB7=TIM4_CH2, IN4=PB15(전진), IN3=PB14(후진)
  *          속도 단위: 퍼센트 0~100 (내부에서 CCR 0~1000 으로 변환)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 초기화: TIM4 PWM CH1/CH2 start + 정지 상태로 진입 (main 에서 1회 호출) */
void Motor_Init(void);

/* 차량 단위 제어 (speed: 0~100 [%]) */
void Car_Forward(uint8_t speed);   /* 양쪽 전진 */
void Car_Backward(uint8_t speed);  /* 양쪽 후진 */
void Car_PivotLeft(uint8_t outer, uint8_t inner);  /* 좌회전: 우(바깥) 전진 outer + 좌(안쪽) 후진 inner. inner=0=안쪽정지, =outer=풀스핀 */
void Car_PivotRight(uint8_t outer, uint8_t inner); /* 우회전: 좌(바깥) 전진 outer + 우(안쪽) 후진 inner */
void Car_ArcLeft(uint8_t outer, uint8_t inner);  /* 부드러운 좌선회: 양바퀴 전진, 안쪽(좌) 느리게 → 반경↑ */
void Car_ArcRight(uint8_t outer, uint8_t inner); /* 부드러운 우선회: 양바퀴 전진, 안쪽(우) 느리게 → 반경↑ */
void Car_Stop(void);               /* 양쪽 정지 (코스트: IN 전부 low + PWM 0 → 관성으로 굴러감) */
void Car_Brake(void);              /* 능동 제동 (L298N short-brake): 제동토크∝속도라 배터리 무관. 해제는 다음 구동/Stop이 담당 */

/* 저수준 개별 모터 제어 (부호 = 방향: +전진 / -후진, 크기 = 속도 % 0~100, 0 = 정지) */
void Motor_Left(int8_t speed);
void Motor_Right(int8_t speed);

/* 최근 0이 아닌 구동 명령의 부호 (+1/-1). 단채널 엔코더(SG-207) 속도 부호 채택용 — §3.2.
 * 정지(0) 명령은 부호를 바꾸지 않는다(관성 감속 중 부호 유지) */
extern volatile int8_t motor_dir_left;
extern volatile int8_t motor_dir_right;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
