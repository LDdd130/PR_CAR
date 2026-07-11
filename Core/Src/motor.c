/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   L298N differential-drive motor implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.h"
#include "tim.h"   /* HAL GPIO definitions, htim4, and PWM channels */

#define MOTOR_MAX_PCT 100

enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
};

volatile int8_t motor_dir_left  = 1;
volatile int8_t motor_dir_right = 1;

typedef struct
{
    GPIO_TypeDef *forward_port;
    uint16_t forward_pin;
    GPIO_TypeDef *reverse_port;
    uint16_t reverse_pin;
    uint32_t pwm_channel;
    volatile int8_t *encoder_dir;
    int16_t target_pct;
    int8_t applied_dir;
    uint8_t neutral_pending;
    uint8_t neutral_started;
    uint32_t neutral_since_ms;
} MotorChannel;

/* Right: ENA=TIM4_CH1, IN1/IN2. Left: ENB=TIM4_CH2, IN4/IN3. */
static MotorChannel channels[MOTOR_COUNT] =
{
    [MOTOR_LEFT] = {
        Input4_GPIO_Port, Input4_Pin,
        Input3_GPIO_Port, Input3_Pin,
        TIM_CHANNEL_2, &motor_dir_left, 0, 0, 0U, 0U, 0U
    },
    [MOTOR_RIGHT] = {
        Input1_GPIO_Port, Input1_Pin,
        Input2_GPIO_Port, Input2_Pin,
        TIM_CHANNEL_1, &motor_dir_right, 0, 0, 0U, 0U, 0U
    }
};

static int16_t clamp_pct(int16_t pct)
{
    if (pct > MOTOR_MAX_PCT) return MOTOR_MAX_PCT;
    if (pct < -MOTOR_MAX_PCT) return -MOTOR_MAX_PCT;
    return pct;
}

static uint32_t pct_to_ccr(uint16_t pct)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
    return (period * (uint32_t)pct) / 100U;
}

static void channel_neutral(MotorChannel *channel)
{
    __HAL_TIM_SET_COMPARE(&htim4, channel->pwm_channel, 0U);
    HAL_GPIO_WritePin(channel->forward_port, channel->forward_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(channel->reverse_port, channel->reverse_pin, GPIO_PIN_RESET);
    channel->applied_dir = 0;
}

static void channel_apply(MotorChannel *channel)
{
    int16_t command = channel->target_pct;
    int8_t requested_dir = (command > 0) ? 1 : ((command < 0) ? -1 : 0);
    uint32_t now = HAL_GetTick();

    if (requested_dir == 0)
    {
        channel_neutral(channel);
        if (channel->neutral_pending && !channel->neutral_started)
        {
            channel->neutral_started = 1U;
            channel->neutral_since_ms = now;
        }
        return;
    }

    if (channel->neutral_pending)
    {
        if (!channel->neutral_started)
        {
            channel_neutral(channel);
            channel->neutral_started = 1U;
            channel->neutral_since_ms = now;
            return;
        }
        if ((now - channel->neutral_since_ms) < 1U)
        {
            channel_neutral(channel);
            return;
        }
        channel->neutral_pending = 0U;
        channel->neutral_started = 0U;
    }

    /* A sign change starts a timed neutral interval before the new direction. */
    if (channel->applied_dir != 0 && requested_dir != channel->applied_dir)
    {
        channel_neutral(channel);
        channel->neutral_pending = 1U;
        channel->neutral_started = 1U;
        channel->neutral_since_ms = now;
        return;
    }

    if (channel->applied_dir != requested_dir)
    {
        /* Also removes a preceding brake duty before enabling a direction. */
        channel_neutral(channel);
        HAL_GPIO_WritePin(channel->forward_port, channel->forward_pin,
                          (requested_dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(channel->reverse_port, channel->reverse_pin,
                          (requested_dir < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    uint16_t magnitude = (uint16_t)((command > 0) ? command : -command);
    __HAL_TIM_SET_COMPARE(&htim4, channel->pwm_channel, pct_to_ccr(magnitude));
    channel->applied_dir = requested_dir;
    *channel->encoder_dir = requested_dir;
}

static void channel_set(MotorChannel *channel, int16_t pct)
{
    channel->target_pct = clamp_pct(pct);
    channel_apply(channel);
}

void Motor_SetWheels(int16_t left_pct, int16_t right_pct)
{
    channels[MOTOR_LEFT].target_pct = clamp_pct(left_pct);
    channels[MOTOR_RIGHT].target_pct = clamp_pct(right_pct);
    channel_apply(&channels[MOTOR_LEFT]);
    channel_apply(&channels[MOTOR_RIGHT]);
}

void Motor_Left(int8_t speed)
{
    channel_set(&channels[MOTOR_LEFT], (int16_t)speed);
}

void Motor_Right(int8_t speed)
{
    channel_set(&channels[MOTOR_RIGHT], (int16_t)speed);
}

void Motor_Init(void)
{
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    Car_Stop();
}

void Car_Forward(uint8_t speed)
{
    int16_t command = clamp_pct((int16_t)speed);
    Motor_SetWheels(command, command);
}

void Car_Backward(uint8_t speed)
{
    int16_t command = clamp_pct((int16_t)speed);
    Motor_SetWheels(-command, -command);
}

void Car_PivotLeft(uint8_t outer, uint8_t inner)
{
    Motor_SetWheels(-clamp_pct((int16_t)inner), clamp_pct((int16_t)outer));
}

void Car_PivotRight(uint8_t outer, uint8_t inner)
{
    Motor_SetWheels(clamp_pct((int16_t)outer), -clamp_pct((int16_t)inner));
}

void Car_ArcLeft(uint8_t outer, uint8_t inner)
{
    Motor_SetWheels(clamp_pct((int16_t)inner), clamp_pct((int16_t)outer));
}

void Car_ArcRight(uint8_t outer, uint8_t inner)
{
    Motor_SetWheels(clamp_pct((int16_t)outer), clamp_pct((int16_t)inner));
}

void Car_Stop(void)
{
    Motor_SetWheels(0, 0);
}

void Car_Brake(void)
{
    uint32_t full_duty = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;

    channels[MOTOR_LEFT].target_pct = 0;
    channels[MOTOR_RIGHT].target_pct = 0;
    channel_neutral(&channels[MOTOR_LEFT]);
    channel_neutral(&channels[MOTOR_RIGHT]);
    __HAL_TIM_SET_COMPARE(&htim4, channels[MOTOR_LEFT].pwm_channel, full_duty);
    __HAL_TIM_SET_COMPARE(&htim4, channels[MOTOR_RIGHT].pwm_channel, full_duty);
    channels[MOTOR_LEFT].neutral_pending = 1U;
    channels[MOTOR_RIGHT].neutral_pending = 1U;
    channels[MOTOR_LEFT].neutral_started = 0U;
    channels[MOTOR_RIGHT].neutral_started = 0U;
}
