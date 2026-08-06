#ifndef DRIVE_H
#define DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "drive_config.h"

/* One immutable sensor snapshot consumed by MotorTask. */
typedef struct
{
    uint16_t f, l, r;       /* filtered distances [cm] */
    uint8_t  f_valid;       /* front measurement is fresh */
    uint8_t  side_valid;    /* full control frame; 0 marks a front-only emergency event */
    uint8_t  left_valid;    /* left ToF is alive and not stale */
    uint8_t  right_valid;   /* right ToF is alive and not stale */
    uint8_t  front_miss;    /* consecutive front timeouts */
    float    heading;       /* BNO055 heading [deg, 0..360, clockwise positive] */
    uint8_t  imu_live;      /* heading was refreshed in this frame */
    uint32_t now;           /* monotonic milliseconds */
} DriveInputs;

typedef enum
{
    DS_CRUISE = 0,
    DS_BRAKE,
    DS_SPIN,
    DS_REVERSE,
    DS_HOLD,
    DS_SIDE_AVOID,
    DS_CORNER
} DriveState;

void Drive_Init(void);
void Drive_Update(const DriveInputs *in);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
