/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    debug.h
  * @brief   SWD Live Expressions용 디버그 미러 일괄 캡슐화 (DebugMonitor_t)
  *          흩어진 dbg_* 전역을 단일 구조체로 통합 — 정의는 drive.c 1곳, 여기선 extern.
  *          접근: dbg.state, dbg.front, ... (구 dbg_state → dbg.state)
  *          ⚠ SWD Live Expression 워치는 dbg.* 로 재등록 필요.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    /* --- 제어 상태 (구 drive.c) --- */
    uint8_t  state;        /* DriveState 0 CRUISE/1 BRAKE/2 SPIN/3 REVERSE/4 HOLD/5 SIDE_AVOID/6 CORNER */
    uint8_t  steer_mode;   /* 0 양벽 / 1 좌벽 / 2 우벽 / 3 heading-hold / 4 open-loop */
    float    steer;        /* 조향량 u [%duty], + = 좌조향 */
    int16_t  duty_l;       /* CRUISE 좌 바퀴 듀티 (데드밴드 침범 감시) */
    int16_t  duty_r;       /* CRUISE 우 바퀴 듀티 */
    float    hdg_err;      /* heading-hold 오차 [deg] */
    float    yaw_rate;     /* CRUISE yaw-rate [deg/s], CW(우)=+ */
    float    spin_deg;     /* SPIN 진입 후 누적 회전각 [deg] */
    uint8_t  turn_dir;     /* 래치 회전 방향 0=좌 1=우 */
    uint8_t  rev_cnt;      /* 직진 복귀 없이 누적 후진 chunk */
    uint8_t  reverse;      /* [req4] 역방향(course_dev>=COURSE_REV_DEG)=1 → cruise 탈출 차단 중 */
    float    course_dev;   /* [req4] heading − course_heading 편차 [deg] (현재 leg 기준) */
    uint8_t  graze;        /* 전면근접이 측벽 그레이징 의심=1 / 진짜 정면벽=0 */
    uint8_t  front_near;   /* 진짜 정면근접 연속 카운트 (FRONT_CONFIRM_N 도달 시 BRAKE) */
    uint8_t  launch;       /* 1 = 자율 시작 직후 런치 직진 커밋 중(LAUNCH_MS) */

    /* --- 센서/필터 (구 freertos.c) --- */
    uint16_t front;        /* 필터 후 정면거리 cm */
    uint16_t left;         /* 필터 후 좌측거리 cm */
    uint16_t right;        /* 필터 후 우측거리 cm */
    uint8_t  front_miss;   /* 정면 연속 무에코 */
    uint32_t loop_ms;      /* 센서 사이클 주기(ms) — 반응지연 감시 */

    /* --- IMU(BNO055) --- */
    float    heading;      /* heading [deg 0~360, CW+] */
    float    roll;
    float    pitch;
    uint8_t  imu_ok;       /* BNO055_Init 성공=1 (main.c 기록) */
    uint8_t  imu_live;     /* 이번 제어 프레임에서 신선한 IMU 샘플 사용 가능 */
    uint8_t  imu_calib;    /* CALIB_STAT [7:6]sys [5:4]gyr [3:2]acc [1:0]mag */
    uint32_t imu_evt;      /* INT(PA0/EXTI0) 발생 카운터 */

    /* --- RTOS / UART 가시성 --- */
    uint32_t q_drop;       /* 큐 put 실패(가득참) 누적 — 0이 정상 */
    uint32_t q_timeout;    /* MotorTask 큐 수신 timeout 누적 */
    uint32_t hw_sensor;    /* SensorTask 스택 high-water [words] */
    uint32_t hw_motor;     /* MotorTask 스택 high-water [words] */
    uint32_t bt_err;       /* USART1 에러콜백(ORE/FE/NE) 발생·복구 횟수 */
} DebugMonitor_t;

/* 정의는 drive.c 1곳 (volatile DebugMonitor_t dbg;) — 다태스크/SWD 가시성 위해 volatile */
extern volatile DebugMonitor_t dbg;

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
