/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   L298N 모터 드라이버 제어 구현
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "tim.h"   /* htim4 */

/* TIM4 ARR = 999 → 한 주기 = 1000 카운트. 듀티 0~100% → CCR 0~1000 */
#define MOTOR_CCR_FULL   1000U

/* 단채널 엔코더(SG-207) 방향 보완: 최근 0이 아닌 구동 명령의 부호 (+1 전진 / -1 후진).
 * 포토인터럽터는 방향 정보가 없어(쿼드러처 아님) 속도 부호는 명령 방향을 채택 — 아키텍처 §3.2.
 * 작성자: Motor_Left/Right 호출 문맥(MotorTask + 폴트 핸들러 Stop뿐) — 사실상 단일 작성자 */
volatile int8_t motor_dir_left  = 1;
volatile int8_t motor_dir_right = 1;

/* 퍼센트(0~100) → CCR(0~1000) 변환 (범위 초과 시 클램프) */
static uint16_t pct_to_ccr(uint8_t pct)
{
    if (pct > 100U)
    {
        pct = 100U;
    }
    return (uint16_t)pct * 10U;
}

/* 속도 %(0~100) 클램프 후 부호 있는 크기로. uint8_t→int8_t 직접 캐스팅 시
 * 100 초과 값(예: 200)이 음수로 뒤집혀 방향이 반대로 가는 버그 방지. */
static int8_t clamp_speed(uint8_t pct)
{
    if (pct > 100U)
    {
        pct = 100U;
    }
    return (int8_t)pct;
}

/* 좌측 모터: 전진 IN4=1/IN3=0, 후진 IN4=0/IN3=1, 속도 = ENB = TIM4_CH2.
 * speed=0 → 양 IN low + PWM 0 으로 완전 정지(궤도식 회전 시 안쪽 바퀴 정지용) */
void Motor_Left(int8_t speed)
{
    if (speed == 0)
    {
        HAL_GPIO_WritePin(Input3_GPIO_Port, Input3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Input4_GPIO_Port, Input4_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        return;
    }
    uint8_t forward = (speed > 0);
    uint8_t mag = forward ? (uint8_t)speed : (uint8_t)(-speed);
    motor_dir_left = forward ? 1 : -1;   /* 엔코더 속도 부호용 최근 방향 래치 */

    HAL_GPIO_WritePin(Input4_GPIO_Port, Input4_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input3_GPIO_Port, Input3_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pct_to_ccr(mag));
}

/* 우측 모터: 전진 IN1=1/IN2=0, 후진 IN1=0/IN2=1, 속도 = ENA = TIM4_CH1.
 * speed=0 → 양 IN low + PWM 0 으로 완전 정지 */
void Motor_Right(int8_t speed)
{
    if (speed == 0)
    {
        HAL_GPIO_WritePin(Input1_GPIO_Port, Input1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Input2_GPIO_Port, Input2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        return;
    }
    uint8_t forward = (speed > 0);
    uint8_t mag = forward ? (uint8_t)speed : (uint8_t)(-speed);
    motor_dir_right = forward ? 1 : -1;   /* 엔코더 속도 부호용 최근 방향 래치 */

    HAL_GPIO_WritePin(Input1_GPIO_Port, Input1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input2_GPIO_Port, Input2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pct_to_ccr(mag));
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   /* ENA (우측) */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   /* ENB (좌측) */
    Car_Stop();
}

void Car_Forward(uint8_t speed)
{
    int8_t s = clamp_speed(speed);
    Motor_Left(s);
    Motor_Right(s);
}

void Car_Backward(uint8_t speed)
{
    int8_t s = clamp_speed(speed);
    Motor_Left(-s);
    Motor_Right(-s);
}

/* 좌회전(궤도식, 마찰 보정): 우(바깥) 전진 outer + 좌(안쪽) 후진 inner → 제자리 선회(CCW)
 * inner=0 → 안쪽 정지(마찰로 안 돌 수 있음), inner 소량 → 깔끔한 선회, inner=outer → 풀 스핀 */
void Car_PivotLeft(uint8_t outer, uint8_t inner)
{
    Motor_Left(-clamp_speed(inner));   /* 좌(안쪽) 후진 → IN3 */
    Motor_Right(clamp_speed(outer));   /* 우(바깥) 전진 → IN1 */
}

/* 우회전(궤도식): 좌(바깥) 전진 outer + 우(안쪽) 후진 inner → 제자리 선회(CW) */
void Car_PivotRight(uint8_t outer, uint8_t inner)
{
    Motor_Left(clamp_speed(outer));    /* 좌(바깥) 전진 → IN4 */
    Motor_Right(-clamp_speed(inner));  /* 우(안쪽) 후진 → IN2 */
}

/* 부드러운 좌선회: 좌(안쪽)=inner, 우(바깥)=outer, 둘 다 전진 → 큰 반경으로 완만하게 좌회전 */
void Car_ArcLeft(uint8_t outer, uint8_t inner)
{
    Motor_Left(clamp_speed(inner));
    Motor_Right(clamp_speed(outer));
}

/* 부드러운 우선회: 좌(바깥)=outer, 우(안쪽)=inner */
void Car_ArcRight(uint8_t outer, uint8_t inner)
{
    Motor_Left(clamp_speed(outer));
    Motor_Right(clamp_speed(inner));
}

void Car_Stop(void)
{
    /* 핀별 포트 매크로 사용 (GPIOB 하드코딩 → 핀 이동 시 일부만 깨지는 문제 방지) */
    HAL_GPIO_WritePin(Input1_GPIO_Port, Input1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input2_GPIO_Port, Input2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input3_GPIO_Port, Input3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input4_GPIO_Port, Input4_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
}

/* 능동 제동 (L298N short-brake): EN=H + IN1==IN2 → 모터 양단 단락 → 역기전력이 제동전류로.
 * 제동토크는 속도에 비례(자기제한)라 배터리 전압과 무관하게 동일 시간으로 동작 — 역펄스 제동의
 * 과제동/기어 백래시 문제 없음. 순서 중요: IN을 먼저 전부 LOW로 내린 *뒤* PWM을 올려야
 * 순간 전진 구동이 생기지 않음. 코스트가 필요하면 Car_Stop() 사용(폴트 핸들러는 Stop 유지). */
void Car_Brake(void)
{
    HAL_GPIO_WritePin(Input1_GPIO_Port, Input1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input2_GPIO_Port, Input2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input3_GPIO_Port, Input3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Input4_GPIO_Port, Input4_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_CCR_FULL);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_CCR_FULL);
}
