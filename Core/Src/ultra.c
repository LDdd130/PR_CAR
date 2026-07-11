#include "ultra.h"
#include "main.h"
#include "tim.h"
#include "delay.h"
#include "drive_config.h"
#include "encoder.h"
#include "FreeRTOS.h"
#include "task.h"

/* Shared only by the TIM3 ISR and SensorTask. */
static volatile uint16_t IC_Value1 = 0;
static volatile uint16_t IC_Value2 = 0;
static volatile uint16_t echoTime = 0;
static volatile uint8_t captureFlag = 0;
static volatile uint16_t distance = 0;
static volatile uint8_t meas_valid = 0;

static osThreadId_t s_waiter = NULL;

#define ULTRA_FLAG_FRONT (1UL << 0)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Encoder_OnCapture(htim);
        return;
    }
    if (htim->Instance != TIM3) return;
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    if (captureFlag == 0U)
    {
        IC_Value1 = (uint16_t)HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        captureFlag = 1U;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,
                                     TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        /* Unsigned subtraction handles the 16-bit timer wrap. */
        IC_Value2 = (uint16_t)HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        echoTime = (uint16_t)(IC_Value2 - IC_Value1);

        uint16_t measured_cm = echoTime / 58U;
        if (measured_cm != 0U)
        {
            distance = (measured_cm > ULTRA_MAX_CM) ? ULTRA_MAX_CM : measured_cm;
            meas_valid = 1U;
            if (s_waiter != NULL)
                (void)osThreadFlagsSet(s_waiter, ULTRA_FLAG_FRONT);
        }

        captureFlag = 0U;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,
                                     TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    }
}

void Ultra_Init(osThreadId_t waiter)
{
    s_waiter = waiter;
    (void)HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

uint8_t Ultra_Measure(uint32_t wait_ms, uint16_t *out_cm)
{
    distance = ULTRA_MAX_CM;
    meas_valid = 0U;

    /* A lingering high echo means the previous measurement has not ended. */
    if (HAL_GPIO_ReadPin(ECHO_TIM3_CH1_GPIO_Port,
                         ECHO_TIM3_CH1_Pin) == GPIO_PIN_SET)
    {
        *out_cm = ULTRA_MAX_CM;
        return 0U;
    }

    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    captureFlag = 0U;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,
                                 TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    (void)osThreadFlagsClear(ULTRA_FLAG_FRONT);

    HAL_GPIO_WritePin(TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, GPIO_PIN_SET);
    delay_us(10U);
    HAL_GPIO_WritePin(TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);

    (void)osThreadFlagsWait(ULTRA_FLAG_FRONT, osFlagsWaitAny, wait_ms);

    uint8_t valid;
    uint16_t measured_cm;
    taskENTER_CRITICAL();
    valid = meas_valid;
    measured_cm = distance;
    taskEXIT_CRITICAL();

    *out_cm = measured_cm;
    return valid;
}

uint16_t median_n(const uint16_t *values, uint8_t n)
{
    uint16_t sorted[16];
    if (n == 0U) return 0U;
    if (n > 16U) n = 16U;

    for (uint8_t i = 0U; i < n; i++) sorted[i] = values[i];
    for (uint8_t i = 1U; i < n; i++)
    {
        uint16_t key = sorted[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && sorted[j] > key)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[n / 2U];
}
