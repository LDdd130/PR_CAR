---
marp: true
theme: prcar
paginate: true
size: 16:9
title: PR_CAR 자율주행차 프로젝트
description: STM32F411CEU6 + FreeRTOS 기반 박스 코스 자율주행차 발표 초안
style: |
  :root {
    --deck-blue: #0284c7;
    --deck-sky: #38bdf8;
    --deck-sky-soft: #e0f2fe;
    --deck-sky-wash: #f0f9ff;
    --deck-ink: #172033;
    --deck-line: #bae6fd;
    --deck-shadow: 0 18px 44px rgba(2, 132, 199, 0.12);
  }
  section {
    position: relative;
    overflow: hidden;
    background:
      radial-gradient(circle at 92% 8%, rgba(56, 189, 248, 0.22), transparent 28%),
      radial-gradient(circle at 4% 94%, rgba(14, 165, 233, 0.12), transparent 24%),
      linear-gradient(180deg, #ffffff 0%, #f7fbff 58%, #eff8ff 100%);
  }
  section::before {
    content: "";
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 10px;
    background: linear-gradient(90deg, var(--deck-blue), var(--deck-sky), #7dd3fc);
  }
  h1,
  h2,
  h3 {
    color: var(--deck-ink);
  }
  h2 {
    position: relative;
    border-bottom: 0;
  }
  h2::after {
    content: "";
    position: absolute;
    left: 0;
    bottom: -2px;
    width: 150px;
    height: 5px;
    border-radius: 999px;
    background: linear-gradient(90deg, var(--deck-blue), var(--deck-sky));
  }
  h3 {
    color: #0369a1;
  }
  li::marker {
    color: var(--deck-blue);
  }
  section.title {
    background:
      radial-gradient(circle at 86% 16%, rgba(56, 189, 248, 0.34), transparent 30%),
      radial-gradient(circle at 16% 78%, rgba(2, 132, 199, 0.16), transparent 26%),
      linear-gradient(135deg, #ffffff 0%, #f0f9ff 54%, #e0f2fe 100%);
  }
  section.title h1 {
    color: #0f172a;
    text-shadow: 0 1px 0 rgba(255, 255, 255, 0.9);
  }
  section.title p {
    color: #075985;
  }
  section.title h1::after {
    content: "";
    display: block;
    width: 210px;
    height: 7px;
    margin-top: 24px;
    border-radius: 999px;
    background: linear-gradient(90deg, var(--deck-blue), var(--deck-sky));
  }
  section.split > .left,
  section.split > .right:not(.placeholder) {
    padding: 22px 24px;
    border: 1px solid rgba(125, 211, 252, 0.7);
    border-radius: 8px;
    background: rgba(255, 255, 255, 0.72);
    box-shadow: 0 14px 34px rgba(2, 132, 199, 0.08);
  }
  section.split > .left {
    border-left: 6px solid var(--deck-sky);
  }
  section:not(.split):not(.title):not(.code-focus) > ol {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 14px 18px;
    padding-left: 0;
    list-style: none;
    counter-reset: deck-step;
  }
  section:not(.split):not(.title):not(.code-focus) > ol > li {
    counter-increment: deck-step;
    min-height: 44px;
    margin: 0;
    padding: 13px 16px 13px 54px;
    border: 1px solid rgba(125, 211, 252, 0.72);
    border-radius: 8px;
    background: rgba(255, 255, 255, 0.74);
    box-shadow: 0 10px 26px rgba(2, 132, 199, 0.08);
    position: relative;
  }
  section:not(.split):not(.title):not(.code-focus) > ol > li::before {
    content: counter(deck-step);
    position: absolute;
    left: 15px;
    top: 12px;
    display: grid;
    place-items: center;
    width: 28px;
    height: 28px;
    border-radius: 999px;
    background: linear-gradient(135deg, var(--deck-blue), var(--deck-sky));
    color: #ffffff;
    font-size: 16px;
    font-weight: 800;
  }
  .placeholder {
    display: flex;
    position: relative;
    align-items: center;
    justify-content: center;
    min-height: 250px;
    padding: 22px;
    border: 2px dashed #7dd3fc;
    border-radius: 8px;
    background:
      linear-gradient(135deg, rgba(224, 242, 254, 0.9), rgba(255, 255, 255, 0.86)),
      repeating-linear-gradient(135deg, rgba(56, 189, 248, 0.08) 0 12px, transparent 12px 24px);
    color: #075985;
    font-size: 22px;
    font-weight: 700;
    text-align: center;
    box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.72), var(--deck-shadow);
  }
  .placeholder::before {
    content: "VISUAL";
    position: absolute;
    top: 16px;
    left: 16px;
    padding: 5px 11px;
    border-radius: 999px;
    background: #ffffff;
    color: var(--deck-blue);
    font-size: 13px;
    letter-spacing: 0.06em;
  }
  .placeholder.large { min-height: 390px; }
  .placeholder.flow { min-height: 330px; }
  .kpi-row {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 16px;
    margin-top: 28px;
  }
  .kpi {
    padding: 18px 20px;
    border: 1px solid rgba(125, 211, 252, 0.76);
    border-radius: 8px;
    background: rgba(255, 255, 255, 0.76);
    box-shadow: var(--deck-shadow);
  }
  .kpi strong {
    display: block;
    margin-bottom: 2px;
    color: var(--deck-blue);
    font-size: 34px;
    line-height: 1.1;
  }
  .kpi span {
    color: #075985;
    font-size: 18px;
    font-weight: 650;
  }
  .compact {
    font-size: 24px;
    line-height: 1.38;
  }
  .caption {
    margin-top: 10px;
    color: #6e7781;
    font-size: 18px;
  }
  .callout {
    padding: 18px 20px;
    border: 1px solid var(--deck-line);
    border-radius: 8px;
    background: var(--deck-sky-wash);
  }
  table {
    border-color: var(--deck-line);
    box-shadow: 0 14px 32px rgba(2, 132, 199, 0.08);
  }
  th {
    background: linear-gradient(180deg, #e0f2fe 0%, #f0f9ff 100%);
    color: #075985;
  }
  td {
    background: rgba(255, 255, 255, 0.8);
  }
  code {
    border-color: rgba(56, 189, 248, 0.5);
    background: #e0f2fe;
    color: #075985;
  }
  section.code-focus {
    background:
      radial-gradient(circle at 88% 12%, rgba(56, 189, 248, 0.22), transparent 30%),
      linear-gradient(180deg, #ffffff 0%, #f0f9ff 100%);
  }
  section.code-focus > pre {
    border-color: #7dd3fc;
    box-shadow: 0 18px 42px rgba(2, 132, 199, 0.15);
  }
  .badge {
    border-color: #7dd3fc;
    background: #e0f2fe;
    color: #075985;
  }
  .badge.critical {
    border-color: #f9a8d4;
    background: #fdf2f8;
    color: #be185d;
  }
  .badge.solved {
    border-color: #67e8f9;
    background: #ecfeff;
    color: #0e7490;
  }
---

<!-- _class: title -->

# PR_CAR 자율주행차 프로젝트

STM32F411CEU6 + FreeRTOS 기반 박스 코스 자율주행차

<div class="kpi-row">
  <div class="kpi"><strong>6.3m</strong><span>박스 코스 전체 길이</span></div>
  <div class="kpi"><strong>30.895s</strong><span>최근 완주 기록 IMG_2986</span></div>
  <div class="kpi"><strong>20s</strong><span>목표 랩타임</span></div>
</div>

---

## 발표 흐름

1. 프로젝트 목표와 제약 조건
2. 하드웨어 구성과 센서 역할
3. FreeRTOS 런타임 구조
4. 벽타기 / heading-hold 제어 전략
5. 최근 문제 진단과 45도 그리드 스냅 해결
6. 텔레메트리와 디버깅 체계
7. 검증 상태와 다음 로드맵

---

<!-- _class: split -->

## 프로젝트 목표

<div class="left">

### 핵심 목표

- 박스 코스를 충돌 없이 빠르게 완주
- 전방 / 측면 센서 융합으로 코너와 벽 접근 판단
- BNO055 heading 기반으로 직진 안정성 확보
- 포트폴리오 관점에서 제어, RTOS, 센서 통합을 명확히 보여주는 구조

</div>

<div class="right placeholder large">
차량 주행 사진 또는 대표 이미지 삽입 영역
</div>

---

<!-- _class: split -->

## 코스와 주행 제약

<div class="left compact">

- 코스: `testtrack.drawio`
- 전체 약 6.3m, 코너 8개
- 최협 구간 폭 37cm
- 차량 크기: 길이 27cm, 폭 16cm
- 최협 중앙 주행 여유: 약 10.5cm
- 45도 코너와 90도 코너가 혼재

<span class="badge critical">critical</span> heading 기준이 틀어지면 앞/뒤 모서리가 센서 위치보다 먼저 벽에 접근한다.

</div>

<div class="right placeholder large">
코스맵 / 주행 경로 이미지 삽입 영역<br>
예: testtrack.drawio 캡처
</div>

---

## 하드웨어 구성

| 구성 | 연결 | 역할 |
|---|---|---|
| STM32F411CEU6 | 100MHz, FreeRTOS | 메인 제어기 |
| 전방 HC-SR04 | PA5 / TIM3_CH1 PA6 | 정면 거리, 코너 진입 판단 |
| VL53L0X ToF x2 | I2C1, XSHUT PA1/PA2 | 좌우 벽 거리 |
| BNO055 | I2C1, INT PA0, RST PB1 | 차량 진행각 heading |
| L298N + DC Motor | TIM4 PWM, PB12~15 | 좌우 차동 구동 |
| Bluetooth | USART1 PA9/PA10 | 명령, 텔레메트리 |
| IWDG | 2.048s | 센서/제어 루프 fail-safe |

---

<!-- _class: split -->

## 센서 설계 포인트

<div class="left compact">

### ToF 주소 분리

- VL53L0X는 기본 주소가 모두 `0x52`
- XSHUT로 한 개씩 깨운 뒤 좌측만 `0x60`으로 이동
- 우측은 기본 `0x52` 유지
- IWDG 리셋 뒤에도 S0~S6 초기화 시퀀스로 결정적 복구

### BNO055 사용 방식

- `heading = Z축 yaw`
- 차량 xy 평면 진행각으로 사용
- roll / pitch는 직진 판단에 사용하지 않음

</div>

<div class="right placeholder flow">
센서 배치도 / I2C 버스 구성도 삽입 영역
</div>

---

<!-- _class: split -->

## FreeRTOS 런타임 구조

<div class="left compact">

### SensorTask

- 전방 초음파 측정
- 좌/우 ToF 폴링
- BNO055 heading 폴링
- median 필터와 유효성 게이팅
- `DriveInputs`를 `driveQ`로 발행

### MotorTask

- `driveQ` 수신
- IWDG refresh
- `Drive_Update(&din)`
- 모터 명령의 유일한 발행자

</div>

<div class="right placeholder flow">
SensorTask → driveQ → MotorTask → PWM<br>
BluetoothTask / Telemetry 포함 데이터 플로우 차트 영역
</div>

---

## 주행 상태 머신

| 상태 | 역할 |
|---|---|
| `CRUISE` | 일반 직진, 벽타기, 코너 후보 판단 |
| `BRAKE` | 정면 위험 감속 / 정지 |
| `SPIN` | 제자리 또는 피벗성 회전 보정 |
| `REVERSE` | 막힘 상황 후진 탈출 |
| `HOLD` | 정지 유지 |
| `SIDE_AVOID` | 측벽 근접 비상 회피 |
| `CORNER` | 전진성 아크 코너링 |

<span class="badge solved">solved</span> 모터 명령은 MotorTask 한 곳에서만 발행해 경쟁 조건을 줄인다.

---

<!-- _class: split -->

## 제어 전략: 벽타기 + heading-hold

<div class="left compact">

### 중심 유지

좌우 ToF 차이를 중심 오차로 사용한다.

`wall_error = dist_left - dist_right`

- 양벽 유효: 좌우 등거리 센터링
- 단일벽 유효: 벽 흡인 금지, 가까운 벽에서 밀어내기
- 벽 정보 부족: BNO055 heading-hold

### 직진 안정화

- 긴 직선에서는 heading 기준을 강하게 유지
- 현재 흔들리는 차체각을 새 기준으로 학습하지 않음

</div>

<div class="right placeholder flow">
벽타기 제어식 / 차체 중심 오차 설명 다이어그램 영역
</div>

---

<!-- _class: split -->

## 최근 문제 진단

<div class="left compact">

### IMG_2986 관찰

- 완주 시간: 30.895초
- 1.6m 긴 직선에서 좌우 지그재그 잔류
- 45R 코너 2개에서 과회전 / 대각 주행
- 마지막 90R 부근에서 큰 wiggle 발생

### 구조 원인

<span class="badge critical">critical</span> 코너 진입 시점의 대각 자세가 다음 레그의 `h_ref`로 유전됨

</div>

<div class="right placeholder large">
IMG_2986 프레임 비교 / 문제 구간 캡처 영역
</div>

---

<!-- _class: split -->

## 해결 방향: 45도 그리드 스냅

<div class="left compact">

코스의 모든 레그축은 다음 불변식을 따른다.

`course_zero + 45° × k`

- `course_zero`: 시작 시 heading 래치
- `course_grid_snap(h)`: 현재 heading에서 가장 가까운 그리드축 복원
- 코너 탈출 후 `h_ref`를 현재 차체각이 아니라 그리드축으로 재설정
- 45도 코너는 35~65도 회전 + 전방 clear 조건에서 조기 탈출

<span class="badge solved">solved</span> 진입 대각과 45도 코너 오염을 기준계에서 제거

</div>

<div class="right placeholder flow">
45도 그리드 스냅 개념도 영역<br>
예: course_zero, k축, 현재 heading 투영
</div>

---

<!-- _class: code-focus -->

# 텔레메트리 프레임

```text
T,<t_ms>,<front_cm>,<left_mm>,<right_mm>,<heading_x10>,<vL>,<vR>,<state>,<flags>

state:
0 CRUISE / 1 BRAKE / 2 SPIN / 3 REVERSE / 4 HOLD / 5 SIDE_AVOID / 6 CORNER / 7 MANUAL

flags:
b0 front_valid / b1 side_valid / b2 imu_live / b3 sys_power / b4 sys_mode
```

60초간 10Hz 프레임 안정성, 명령 왕복 지연, 상태 전이를 대시보드에서 확인한다.

---

<!-- _class: split -->

## 디버깅 관측 포인트

<div class="left compact">

### SWD Live Expressions

- `dist_left`, `dist_right`
- `wall_error`
- `dbg.state`, `dbg.steer_mode`
- `dbg.steer`, `dbg.hdg_err`
- `dbg.yaw_rate`
- `dbg.duty_l`, `dbg.duty_r`
- `dbg.v_l`, `dbg.v_r`
- `dbg.tel_tx`, `dbg.tel_skip`

</div>

<div class="right placeholder large">
대시보드 화면 / 실시간 그래프 캡처 영역
</div>

---

## 현재 검증 상태

| 항목 | 상태 |
|---|---|
| 최신 빌드 | `make -j4` 성공 |
| 최근 완주 | `IMG_2986.mov`, 30.895초 |
| 45도 그리드 스냅 | 코드 반영, 실차 재주행 필요 |
| ToF 주소 분리 | as-built 검증 통과 |
| BNO055 heading | xy 평면 진행각 사용 확인 |
| 엔코더 / 텔레메트리 | R0~R3 계층 추가, 벤치 검증 필요 |
| 속도 PI / 조향 캐스케이드 | 후속 로드맵 |

<span class="badge critical">next</span> 최신 bin 플래시 후 `IMG_2987.mov` 촬영과 주행 분석이 다음 기준점이다.

---

<!-- _class: split -->

## 다음 로드맵

<div class="left compact">

### 단기

1. 최신 bin 플래시
2. 새 주행 영상 촬영
3. 긴 직선 지그재그 감소 확인
4. 45R 코너가 45도만 돌고 탈출하는지 확인
5. 문제가 남으면 튜닝값을 한 번에 하나만 변경

</div>

<div class="right compact">

### 중기

1. 엔코더 벤치 체크
2. 텔레메트리 10Hz 안정성 확인
3. duty sweep으로 feed-forward 표 작성
4. 속도 PI 적용
5. 조향 출력을 `%duty`에서 `cm/s` 차동 속도로 전환

</div>

---

<!-- _class: title -->

# 결론

PR_CAR의 현재 핵심은 단순한 게인 조정이 아니라 기준계 안정화다.

45도 그리드 스냅으로 heading 기준 오염을 끊고, 이후 엔코더 기반 속도 루프와 텔레메트리로 튜닝을 숫자로 닫는 것이 다음 단계다.
