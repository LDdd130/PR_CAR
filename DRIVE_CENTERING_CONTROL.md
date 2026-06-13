# STM32 차량 중앙 유지 주행 제어 개선 정리

## 개요

좁은 주행 코스에서 차량이 좌우로 반복 진동하는 Hunting/Overshoot 현상을 줄이기 위해 `drive.c`의 CRUISE 주행 조향 로직을 개선했다.

기존의 단순 임계값 기반 Bang-Bang 조향 대신, 좌우 초음파 센서 기반 중앙 유지 제어를 다음 방식으로 변경했다.

- 이동 평균 필터
- 데드존
- PD 제어
- 조향량 비례 가변 감속

현재 주행 결과는 속도는 느리지만 벽에 부딪히지 않고 코스를 안정적으로 완주하는 상태다.

## 적용 파일

- `Core/Src/drive.c`

기존 모듈 구조는 유지했다.

- 초음파 센서 측정: `ultra.c`
- 모터 PWM 출력: `motor.c`
- 주행 판단 및 제어: `drive.c`

새 제어 로직은 `drive.c` 내부에 추가했고, FreeRTOS 태스크에서 기존처럼 `Drive_Update()`를 호출하면 CRUISE 상태에서 자동으로 실행된다.

## 추가된 튜닝 매크로

```c
#define CENTER_MA_WIN              5U
#define CENTER_DEADZONE_CM         2.0f
#define CENTER_KP                  0.80f
#define CENTER_KD                  0.035f
#define CENTER_STEER_MAX_PCT       10.0f
#define CENTER_BASE_SPEED_PCT      32.0f
#define CENTER_MIN_SPEED_PCT       24.0f
#define CENTER_SENSOR_MAX_CM       80U
```

각 값의 역할은 다음과 같다.

| 매크로 | 의미 |
| --- | --- |
| `CENTER_MA_WIN` | 좌/우 초음파 센서 이동 평균 샘플 수 |
| `CENTER_DEADZONE_CM` | 중앙으로 간주할 좌우 거리 오차 범위 |
| `CENTER_KP` | 좌우 거리 오차에 대한 비례 제어 게인 |
| `CENTER_KD` | 오차 변화율에 대한 미분 제어 게인 |
| `CENTER_STEER_MAX_PCT` | 좌우 모터 차동 조향 최대값 |
| `CENTER_BASE_SPEED_PCT` | 직진 또는 작은 조향 시 기본 전진 속도 |
| `CENTER_MIN_SPEED_PCT` | 큰 조향 시 최저 전진 속도 |
| `CENTER_SENSOR_MAX_CM` | 초음파 센서 튐값을 제한하는 최대 거리 |

## 제어 알고리즘

### 1. 이동 평균 필터

좌/우 초음파 센서값을 각각 링 버퍼에 저장하고 최근 `CENTER_MA_WIN`개 평균을 사용한다.

목적은 다음과 같다.

- 순간적인 초음파 Spike 완화
- 센서 응답 지연에 의한 튐값 완화
- 좁은 코스에서 좌우 조향 명령이 급격히 바뀌는 현상 감소

### 2. 데드존

오차는 다음과 같이 계산한다.

```c
error = left_cm - right_cm;
```

`fabsf(error) <= CENTER_DEADZONE_CM`이면 조향 명령을 0으로 둔다.

이 구간에서는 차량이 이미 중앙 근처에 있다고 판단하고, 불필요한 미세 조향을 억제한다.

### 3. PD 제어

데드존을 벗어나면 다음 식으로 조향값을 계산한다.

```c
steer = (CENTER_KP * error) + (CENTER_KD * d_error);
```

`error`가 양수이면 좌측 거리가 더 크고 우측 벽에 가까운 상태이므로 좌회전 보정이 들어간다.

계산된 조향값은 `CENTER_STEER_MAX_PCT` 범위로 제한한다.

### 4. 조향 비례 가변 감속

조향값의 절댓값이 클수록 차량 속도를 선형적으로 줄인다.

```c
steer_ratio = fabsf(steer) / CENTER_STEER_MAX_PCT;
base = CENTER_BASE_SPEED_PCT
     - ((CENTER_BASE_SPEED_PCT - CENTER_MIN_SPEED_PCT) * steer_ratio);
```

효과는 다음과 같다.

- 큰 조향이 필요한 구간에서 전진 속도 감소
- 조향기가 자세를 바로잡을 시간 확보
- 좁은 코스에서 Overshoot 감소
- 벽 충돌 가능성 감소

## 모터 출력 방식

계산된 `base`와 `steer`를 이용해 좌우 모터 PWM을 차동으로 출력한다.

```c
left_pwm  = base - steer;
right_pwm = base + steer;
```

최종 출력은 기존 `motor.c` 인터페이스를 그대로 사용한다.

```c
Motor_Left(...);
Motor_Right(...);
```

따라서 모터 모듈 구조나 PWM 출력 함수는 변경하지 않았다.

## CRUISE 상태 연결

`cruise_run()`에서는 기존 안전 게이트를 유지했다.

- 정면 센서 연속 무응답 시 HOLD
- 정면 장애물 근접 시 BRAKE
- 측면 충돌 직전 시 SIDE_AVOID

위 조건에 걸리지 않으면 새 중앙 유지 제어 함수를 호출한다.

```c
Drive_CenteringPD_Run(in);
```

## 현재 효과

변경 후 차량은 이전처럼 좌우로 크게 튕기는 핑퐁 현상이 줄었고, 속도는 느리지만 좁은 코스를 충돌 없이 안정적으로 완주한다.

현재 튜닝 방향은 안정성 우선이다. 속도를 더 올리고 싶다면 `CENTER_BASE_SPEED_PCT`를 조금씩 올리되, 진동이 다시 발생하면 `CENTER_KD`를 소폭 올리거나 `CENTER_KP`를 낮춰야 한다.
