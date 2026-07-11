#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Single SWD/telemetry snapshot. Writers are noted by section. */
typedef struct
{
    /* drive.c, called by MotorTask */
    uint8_t state;
    uint8_t steer_mode;
    float steer;
    int16_t duty_l;
    int16_t duty_r;
    float hdg_err;
    float yaw_rate;
    float spin_deg;
    uint8_t turn_dir;
    uint8_t rev_cnt;
    uint8_t reverse;
    float course_dev;
    uint8_t graze;
    uint8_t front_near;
    uint8_t launch;

    /* SensorTask */
    uint16_t front;
    uint16_t left;
    uint16_t right;
    uint8_t front_miss;
    uint32_t loop_ms;
    float heading;
    float roll;
    float pitch;
    uint8_t imu_ok;
    uint8_t imu_live;
    uint8_t imu_calib;
    uint32_t imu_evt;

    /* MotorTask / encoder telemetry */
    float v_l;
    float v_r;
    uint32_t enc_l;
    uint32_t enc_r;
    int16_t v_target;

    /* BluetoothTask / RTOS diagnostics */
    uint32_t tel_tx;
    uint32_t tel_skip;
    uint32_t q_drop;
    uint32_t q_timeout;
    uint32_t hw_sensor;
    uint32_t hw_motor;
    uint32_t bt_err;
} DebugMonitor_t;

extern volatile DebugMonitor_t dbg;

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H */
