# PR_CAR 인수인계 문서

작성: 2026-07-04 (IMG_2986 분석 반영)  
기준 영상: `IMG_2986.mov` (30.895s 완주)  
현재 코드 상태: IMG_2986에서 확인된 '직진 지그재그 + 45° 코너 과회전'의 구조 원인(heading 기준 오염)을 없애기 위해 **45° 코스 그리드 스냅** 아키텍처를 적용한 상태. `make -j4` 성공. 아직 이 수정본으로 실차 재주행은 하지 않음.

## 0. 새 세션에서 먼저 볼 것

- 최신 산출물: `build/PR_CAR.bin`
- 최신 검증: `make -j4` 성공, RAM 22056B/128KB, FLASH 43276B/512KB
- 다음 실제 작업: 최신 bin 플래시 후 새 영상 촬영. 예상 이름은 `IMG_2987.mov`.
- 가장 중요한 확인 포인트:
  - 긴 직선(1.6m)에서 좌→우→좌 지그재그가 사라졌는가? (h_ref가 이제 그리드축이라 사라져야 정상)
  - 45R 코너 2개에서 과회전 없이 ~45°만 돌고 빠져나오는가? (grid exit A′)
  - 마지막 구간(방지턱 이후)에서 큰 제자리 회전 없이 통과하는가?
  - 완주 시간이 `IMG_2986`의 30.895초보다 줄어드는가?

새 AI/세션은 코드 질문을 받으면 먼저 `graphify query "<질문>"`로 맥락을 잡는다. 코드 수정 후에는 `graphify update .`를 실행해 그래프를 최신화한다.

## 1. 프로젝트 목표

- STM32F411CEU6 + FreeRTOS 자율주행차.
- 박스 코스 `testtrack.drawio`를 충돌 없이 빠르게 완주. 목표는 20초 이내.
- 포트폴리오 핵심: PID 제어, BNO055 heading-hold, ToF/초음파 센서 융합, median 필터링.
- 차량 크기: 길이 27cm, 폭 16cm.

## 2. 하드웨어와 센서

| 부품 | 연결 | 역할 |
|---|---|---|
| 전방 HC-SR04 | TRIG PA5, ECHO TIM3_CH1 PA6 | 직진/코너/정면 위험 판단 |
| 좌측 VL53L0X ToF | I2C1 PB8/PB9, XSHUT PA1, 주소 0x60 | 좌측 벽 거리 |
| 우측 VL53L0X ToF | I2C1 PB8/PB9, XSHUT PA2, 주소 0x52 | 우측 벽 거리 |
| BNO055 | I2C1, INT PA0, RST PB1 | 차량 진행각 heading 유지 |
| L298N 모터 | TIM4 PWM PB6/PB7, IN1~4 PB12~15 | 좌우 모터 제어 |
| 블루투스 | USART1 PA9/PA10 | 수동/자율 명령 |
| IWDG | 2.048s | MotorTask 큐 수신 성공 시 refresh |

현재 제어에서 BNO055는 차량 평면 진행각 `heading`을 사용한다. roll/pitch는 직진 판단에 쓰지 않는다.

## 3. 런타임 구조

- `Core/Src/freertos.c`
  - `StartTask02`: SensorTask. 전방 초음파, 좌/우 ToF, BNO055를 읽고 `DriveInputs`를 큐로 보낸다.
  - 현재 센서 순서: front 초음파 -> ToF 좌 -> front 재확인 -> ToF 우 -> median3 -> IMU -> 큐 전송.
  - `StartDefaultTask`: MotorTask. 큐 수신 -> IWDG refresh -> `Drive_Update(&din)`.
  - 모터 명령은 MotorTask만 발행한다.

- `Core/Src/drive.c`
  - HAL/I2C 직접 접근 없음. `DriveInputs`와 `motor.h`만 사용.
  - 상태머신: `CRUISE`, `BRAKE`, `SPIN`, `REVERSE`, `HOLD`, `SIDE_AVOID`, `CORNER`.
  - `Drive_Update()`가 상태별 run 함수로 분기한다.
  - `cruise_run()`이 정면/측면 조건을 보고 코너, 제동, 측면회피, 일반 센터링을 결정한다.
  - `Drive_CenteringPD_Run()`이 직진/복도 센터링 조향과 속도 제한을 담당한다.
  - `corner_run()`은 제자리 피벗이 아니라 전진성 아크 코너를 우선 사용한다.

- `Core/Inc/drive.h`
  - 상태 전이 임계값, 코너 속도, heading gain, side avoid 등 공개 튜닝값.

- `Core/Src/vl53l0x.c`
  - VL53L0X 경량 드라이버. 타이밍버짓 20ms. 반환값 1=새 샘플, 0=아직, -1=에러.

## 4. 코스 정보

`testtrack.drawio` 기준:

출발 37cm -> 90R -> 45cm -> 90L -> 43cm 최장직선 1.6m -> 45R -> 55cm -> 45R -> 50cm -> 90L -> 50cm -> 90R 67cm 광폭 -> 90L -> 50cm+방지턱 -> 90R -> 60cm 도착.

- 코너 8개, 전체 약 6.3m.
- 최협 37cm에서 중앙 측면 여유는 `(37 - 16) / 2 = 10.5cm`.
- `SIDE_AVOID_CM 9`는 정상 중앙주행 10.5cm보다 낮게 둔 값.
- 차체 길이 27cm 때문에 heading이 틀어지면 앞/뒤 모서리가 측면센서 위치보다 벽에 더 가까워진다. 그래서 `drive.c`에 yaw 기반 근접벽 가드가 있다.
- 37cm 코너 정션에서는 정상 아크 중에도 전방 초음파가 약 16cm까지 떨어질 수 있다. 그래서 아크 중 중단선은 `CORNER_ABORT_CM 16`으로 분리되어 있다.

## 5. IMG_2986 관찰과 수정 결과 (최신)

### 영상 관찰

- `IMG_2986.mov`: 30.895초 완주 (IMG_2985 대비 -4.0s).
- t=8~11.5s 긴 직선(1.6m): 여전히 좌벽 접근 → 급교정 → 우측으로 흐름 반복. 벽에 거의 닿는 순간 2회.
- t=14~17.5s (45R 코너 2개 구간): 코너 후 한쪽 벽으로 흐르는 대각 주행.
- t=28~29.5s 마지막 90R 부근: 차체가 크게 돌아가는 wiggle(스핀 폴백 추정).

### 구조 진단 (이번 세션 핵심)

직진 지그재그는 게인 문제가 아니라 **heading 기준(h_ref) 오염**이 원인:

1. 구 `turn_locked_heading() = h_entry ± 90°`는 코너 '진입 시점'의 대각 자세 δ(10~15°)를
   다음 레그 기준에 그대로 유전 → 레그 내내 heading-hold가 δ만큼 대각으로 끌고 감
   → 근접벽 가드가 되밀며 지그재그. 긴 직선의 좌우 흐름이 이것.
2. testtrack에는 **45R 코너가 2개** 있는데 코너 탈출 조건(A: 88°, B: 80°+CLEAR)이 전부 90° 전제
   → 45° 코너에서 40°+ 과회전하거나 ARC 타임아웃→SPIN 폴백, 그리고 h_ref가 45° 오염.
3. h_ref 재학습 게이트(`CENTER_HREF_ALIGN_HDG_DEG 8°`)는 기준이 45° 틀어진 상태에서는
   영구히 안 열림 → 오염 기준이 다음 코너까지 지속.

### 적용한 방향: 45° 코스 그리드 스냅

레그축은 전부 `시작방위 + 45°×k` 라는 코스 불변식을 이용:

- `course_zero`: 시작 래치 heading = 그리드 원점 (`Drive_Update` 최초 imu_live 프레임).
- `course_grid_snap(h)`: h에서 가장 가까운 그리드축 반환. 물리 heading이 실제 레그축 ±22.5° 안이면
  정확한 레그축을 복원 — 진입 대각 δ, 45° 오스냅, 흔들리는 차체각 학습을 전부 무효화.
- `cruise_enter`(코너/스핀 탈출): `h_ref = course_grid_snap(현재 heading)` (구 h_entry±90 폐기).
- 런치/fresh/IMU 부활 시 h_ref 래치도 전부 그리드 스냅.
- **코너 grid exit A′**: 아크 중 회전각 35~65° + 그리드축 ±10° 정렬 + 전방 ≥52cm(`CORNER_GRID_EXIT_CM`)
  CLEAR_CONFIRM회 연속이면 조기 탈출 = 45° 코너를 45°만 돌고 나감. 90° 정션 중간(대각)의 전방빔은
  외벽 빗면 ~26..47cm라 52cm 게이트를 못 넘어 오탈출하지 않음. 65° 초과는 exit A(88°)에 위임.
- **레그축 재스냅**: CRUISE 중 양벽 균형(|l−r|≤4cm)·저변화율·저요레이트인데 hdg_err ≥25°가
  12루프 연속이면 기준 쪽이 틀린 것(코너 이벤트 없이 지난 45° 굽이 등) → h_ref/course_heading을
  현재 heading의 그리드 스냅으로 교체. 8° 게이트로 영구 오염되던 문제의 회복 경로.

변경 파일: `Core/Src/drive.c` (course_zero/course_grid_snap/cruise_enter/centering 래치·재스냅/corner_run A′),
`Core/Inc/drive.h` (TURN_LOCK_DEG 삭제 → CORNER_GRID_* 4개 추가).

## 5-구. IMG_2985 관찰과 수정 결과

### 영상 관찰

- `IMG_2985.mov`: 34.895초, 480x854, 29fps.
- 완주는 하지만 직진 구간에서 차가 계속 좌우 벽 쪽으로 기울고 중심 복귀가 늦다.
- 속도를 올릴 기반이 안 잡혀 직진 속도도 잘 살아나지 않는다.
- 영상상 문제는 코너 하나의 문제가 아니라, 전방이 열린 직진에서도 heading 기준과 좌우 벽 균형이 빨리 잡히지 않는 구조에 가깝다.

### 적용한 방향

핵심 변경은 `front-straight-lock`이다.

- 전방 초음파가 코너 임계보다 충분히 열려 있으면 직진 상태로 본다.
- 직진 상태에서는 BNO055 heading 기준을 강하게 유지한다.
- 좌우 ToF는 코너 판단보다 먼저, 좌우 거리값을 비슷하게 맞추는 등거리 보정에 집중한다.
- 직진 중 흔들리는 현재 heading을 `h_ref`로 다시 학습하지 않는다. 대각 주행이 새 기준으로 굳는 것을 막기 위함이다.
- 코너 판단은 전방이 더 가까워진 뒤 front+side 조건으로만 들어간다.

### 변경 위치

- `Core/Src/drive.c`
  - `CENTER_*` 튜닝값 상향/정리.
  - `Drive_CenteringPD_Run()`에 `front_straight_open = in->f_valid && in->f >= FRONT_ARC_CM` 추가.
  - `front_straight_open && side_pair_valid`이면 straight 전용 P/D와 heading blend 사용.
  - straight-lock 중에는 `h_ref` 학습 금지.
  - 직진 기본속도와 fast 조건 상향.

- `Core/Inc/drive.h`
  - `FRONT_ARC_CM 58 -> 52`
  - `KP_HDG 0.45 -> 0.58`
  - `KD_YAW 0.24 -> 0.30`

## 6. 현재 핵심 튜닝값

`Core/Src/drive.c`의 `CENTER_*`:

- `CENTER_BASE_SPEED_PCT 66`
- `CENTER_STRAIGHT_FAST_SPEED_PCT 70`
- `CENTER_NARROW_FAST_SPEED_PCT 66`
- `CENTER_WIDE_FAST_SPEED_PCT 70`
- `CENTER_LPF_ALPHA 0.74`
- `CENTER_KP 0.11`
- `CENTER_KD 0.004`
- `CENTER_STRAIGHT_SIDE_KP 0.18`
- `CENTER_STRAIGHT_SIDE_KD 0.006`
- `CENTER_STEER_MAX_PCT 24`
- `CENTER_STEER_SLEW_PCT 5`
- `CENTER_STRAIGHT_HDG_BLEND 0.78`
- `CENTER_NARROW_KP 0.13`
- `CENTER_NARROW_HDG_BLEND 0.45`
- `CENTER_WIDE_KP 0.07`
- `CENTER_WIDE_HDG_BLEND 0.46`
- `CENTER_HREF_BLEND 0.006`
- `CENTER_NEAR_GUARD_CM 10.1`
- `CENTER_NEAR_GUARD_KP 3.0`
- `CENTER_YAW_DAMP_MAX_PCT 9`
- `CENTER_FRONT_FAST_CM 64`
- `CENTER_FRONT_SLOW_CM 42`

`Core/Inc/drive.h`:

- `FRONT_STOP_CM 28`
- `FRONT_TURN_CM 36`
- `FRONT_CLEAR_CM 44`
- `FRONT_ARC_CM 52`
- `CORNER_ABORT_CM 16`
- 그리드 스냅: `CORNER_GRID_EXIT_CM 52`, `CORNER_GRID_EXIT_MIN_DEG 35`, `CORNER_GRID_EXIT_MAX_DEG 65`, `CORNER_GRID_ALIGN_DEG 10`
- 레그축 재스냅(drive.c): `CENTER_AXIS_RESNAP_DEG 25`, `CENTER_AXIS_RESNAP_YAW_DPS 12`, `CENTER_AXIS_RESNAP_N 12`
- `KP_HDG 0.58`
- `KD_YAW 0.30`
- `TURN_SPEED 42`
- `TURN_INNER 20`
- `ARC_OUTER 50`
- `ARC_INNER 21`
- `ARC_APPROACH_OUTER 36`
- `ARC_APPROACH_INNER 16`
- `ARC_APPROACH_DEG 34`
- `SIDE_AVOID_CM 9`
- `SIDE_AVOID_CLEAR_CM 10`
- `SIDE_AVOID_CLEAR_CONFIRM 3`
- `SIDE_ESCAPE_OUTER 40`
- `SIDE_ESCAPE_INNER 12`
- `CORNER_CONFIRM_N 2`
- wide 코너 오판 방지: `CORNER_WIDE_OPEN_CM 34`, `CORNER_WIDE_ASYM_OPEN_CM 36`, `CORNER_WIDE_ASYM_CM 16`

## 7. 구조적으로 되돌리면 안 되는 원칙

1. 양벽 모드 `SM_BOTH`에서 측벽 반발 조향을 다시 켜지 말 것. 양벽일 때는 센터링 PD가 단독으로 좌우 균형을 맡는다.
2. `SIDE_AVOID` 탈출 시 `h_ref`를 현재 heading으로 잡지 말 것. `course_heading`으로 복귀해야 대각 주행이 기준화되지 않는다.
3. 단일벽 모드는 벽 흡인 금지. 가까운 벽에서 밀어내기만 하고, 진행축은 IMU heading-hold가 잡는다.
4. 코너 판단은 front AND side 구조를 유지한다. ToF 한쪽이 멀다고 바로 코너로 들어가면 넓은 직선에서 오판한다.
5. `h_ref`를 너무 빨리 현재 heading으로 학습시키지 말 것. 흔들리는 차체각을 기준으로 삼으면 직진이 대각으로 굳는다.
6. 코너 아크 중 `CORNER_ABORT_CM 16`을 직선용 `FRONT_STOP_CM 28`과 합치지 말 것. 정상 아크도 전방 빔이 가까운 값을 낼 수 있다.
7. h_ref/course_heading은 **항상 45° 그리드축**(`course_grid_snap`)으로만 래치할 것. 현재 차체각이나 h_entry±90 같은 상대값 래치로 되돌리면 진입 대각 유전·45° 코너 오염이 재발한다.
8. 코너 grid exit A′의 `CORNER_GRID_EXIT_CM 52`를 `FRONT_CLEAR_CM 44`로 낮추지 말 것. 90° 정션 중간(대각 45°)의 외벽 빔이 47cm까지 나와 90° 코너를 45°에서 끊게 된다.

## 8. 다음 튜닝 가이드

새 영상에서 문제를 보고 한 번에 하나만 바꾼다.

- 직진에서 아직 벽 쪽으로 흐름:
  - 우선 `CENTER_STRAIGHT_SIDE_KP 0.18 -> 0.21`
  - 그래도 부족하면 `KP_HDG 0.58 -> 0.64`

- 직진에서 좌우 급반전/와리가리:
  - 우선 `CENTER_STRAIGHT_SIDE_KD 0.006 -> 0.0045`
  - 그래도 과하면 `CENTER_STEER_SLEW_PCT 5 -> 4`

- 코너 진입이 늦어서 정면을 민다:
  - `FRONT_ARC_CM 52 -> 55`
  - 그래도 늦으면 `FRONT_TURN_CM 36 -> 38`

- 코너 오판이 다시 늘어난다:
  - `FRONT_ARC_CM 52`는 유지하고 `CORNER_CONFIRM_N 2 -> 3`
  - 넓은 직선에서만 오판이면 wide 기준을 먼저 올림: `CORNER_WIDE_OPEN_CM 34 -> 38`

- 넓은 직선이 안정적인데 느림:
  - 벽접촉 0회 확인 후 `CENTER_WIDE_FAST_SPEED_PCT 70 -> 72`

## 9. 영상/빌드 작업 방법

영상 정보:

```bash
gst-discoverer-1.0 IMG_XXXX.mov
```

프레임 추출:

```bash
mkdir -p /tmp/pr_car_frames
gst-launch-1.0 -q filesrc location=IMG_XXXX.mov ! decodebin ! videoconvert ! videoscale ! videorate \
  ! "video/x-raw,framerate=2/1,width=240,height=427" ! jpegenc quality=82 \
  ! multifilesink location=/tmp/pr_car_frames/f_%03d.jpg
```

빌드:

```bash
make -j4
```

코드 변경 후 그래프 갱신:

```bash
graphify update .
```

## 10. 디버깅 포인트

SWD/Live Expressions에서 보면 좋은 값:

- `dist_left`, `dist_right` mm
- `wall_error`
- `dbg.state`
- `dbg.steer_mode`
- `dbg.steer`
- `dbg.hdg_err`
- `dbg.yaw_rate`
- `dbg.duty_l`, `dbg.duty_r`
- `dbg.graze`
- `tof_left_ok`, `tof_right_ok`는 `freertos.c` static 스코프라 해당 파일 컨텍스트에서 확인

상태 해석:

- `SM_BOTH`: 양벽 ToF 유효, 좌우 등거리 센터링
- `SM_SINGLE`: 한쪽 벽만 추적, 흡인 금지
- `SM_HDG`: 벽 정보 부족, BNO055 heading-hold
- `DS_CORNER`: 전진성 아크 코너
- `DS_SIDE_AVOID`: 측벽 비상 회피

## 11. 최근 주행 이력 요약

| 영상 | 시간 | 요약 |
|---|---:|---|
| `IMG_2980.mov` | 약 30.0s | 완주 시간 개선, 긴 직선 좌우 헌팅 잔류 |
| `IMG_2982.mov` | 약 30.9s | 헌팅 감소, 코너 탈출 후 한쪽 벽 접촉 잔류 |
| `IMG_2983.mov` | 32.62s | 전체적으로 느림. 직선 fast/코너 감속 재조정 |
| `IMG_2984.mov` | 34.86s | 좁은/넓은 구간별 판단 필요 확인. 폭 프로파일 추가 |
| `IMG_2985.mov` | 34.895s | 직진이 계속 좌우로 흐름. front-straight-lock 적용 |
| `IMG_2986.mov` | 30.895s | 지그재그/45°코너 과회전 잔존 → 원인이 h_ref 오염으로 진단됨. 45° 그리드 스냅 적용 |

현재 문서는 `IMG_2986.mov` 분석 이후 수정된 코드 기준이다. 다음 세션은 새 영상을 먼저 분석하고, 위 튜닝 가이드에서 한 항목만 골라 조정한다. 그리드 스냅 관련 이상 거동(코너를 45°에서 끊거나, 직선에서 h_ref가 엉뚱한 축으로 재스냅)이 보이면 §6의 GRID/RESNAP 값을 먼저 본다.
