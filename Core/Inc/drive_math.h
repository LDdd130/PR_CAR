#ifndef DRIVE_MATH_H
#define DRIVE_MATH_H

#include "drive_config.h"

typedef enum
{
    DRIVE_CORRIDOR_UNKNOWN = 0,
    DRIVE_CORRIDOR_NARROW,
    DRIVE_CORRIDOR_MID,
    DRIVE_CORRIDOR_WIDE
} DriveCorridorClass;

static inline float drive_clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static inline float drive_deadbandf(float value, float band)
{
    if (value > band) return value - band;
    if (value < -band) return value + band;
    return 0.0f;
}

static inline float drive_wrap180(float angle)
{
    if (angle > 180.0f) angle -= 360.0f;
    else if (angle <= -180.0f) angle += 360.0f;
    return angle;
}

static inline float drive_wrap360(float angle)
{
    if (angle >= 360.0f) angle -= 360.0f;
    else if (angle < 0.0f) angle += 360.0f;
    return angle;
}

static inline DriveCorridorClass drive_corridor_class(float left_cm, float right_cm)
{
    float width_cm = left_cm + right_cm + COURSE_CAR_WIDTH_CM;
    if (width_cm <= COURSE_NARROW_MAX_CM) return DRIVE_CORRIDOR_NARROW;
    if (width_cm >= COURSE_WIDE_MIN_CM) return DRIVE_CORRIDOR_WIDE;
    return DRIVE_CORRIDOR_MID;
}

#endif /* DRIVE_MATH_H */
