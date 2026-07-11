#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "drive.h"

typedef enum
{
    DRIVE_STEER_BOTH = 0,
    DRIVE_STEER_HEADING = 3,
    DRIVE_STEER_OPEN = 4,
    DRIVE_STEER_SINGLE = 5
} DriveSteerMode;

typedef struct
{
    int16_t left_pct;
    int16_t right_pct;
    float steer_pct;
    float yaw_rate_dps;
    float heading_error_deg;
    float heading_ref_deg;
    uint8_t mode;
    uint8_t axis_resnapped;
} DriveControlOutput;

void DriveControl_Reset(void);
void DriveControl_PrimeHeading(float heading_deg);
void DriveControl_Run(const DriveInputs *in,
                      float heading_ref_deg,
                      float grid_heading_deg,
                      uint32_t cruise_elapsed_ms,
                      DriveControlOutput *out);

#endif /* DRIVE_CONTROL_H */
