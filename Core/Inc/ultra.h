#ifndef ULTRA_H
#define ULTRA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* Register the SensorTask waiter and start TIM3 input capture. */
void Ultra_Init(osThreadId_t waiter);

/* Measure the front HC-SR04. Returns 1 only for a completed echo. */
uint8_t Ultra_Measure(uint32_t wait_ms, uint16_t *out_cm);

/* Median helper for the task-owned sensor histories (n <= 16). */
uint16_t median_n(const uint16_t *values, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* ULTRA_H */
