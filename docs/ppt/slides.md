---
marp: true
theme: prcar
paginate: true
size: 16:9
title: PR_CAR 자율주행 RC카
description: STM32F411 + FreeRTOS 벽추종 자율주행 RC카 — 발표 & 포트폴리오 덱
---

<!-- _class: title -->
<!-- TODO: 발표자 이름/소속/날짜 추가. KPI 수치는 최신 실차 검증 주행 후 갱신 -->

# PR_CAR 자율주행 RC카

STM32F411 + FreeRTOS 기반 차동구동 벽추종 자율주행 — 설계·제어·트러블슈팅 기록

<div class="kpi-row">
  <div class="kpi"><strong>28.86s</strong><span>최근 완주 기록 (IMG_3018)</span></div>
  <div class="kpi"><strong>20s</strong><span>목표 랩타임</span></div>
  <div class="kpi"><strong>6.3m</strong><span>트랙 길이 · 코너 8개</span></div>
</div>

<p class="caption">기준 수동주행 영상 18.93s — 자율주행으로 이 수준의 코너링 흐름을 목표로 한다.</p>

---

<!-- _class: split -->
<!-- TODO: 최신 완주 영상 대표 프레임(코너 아크 장면 추천) 캡처 삽입. 실차 검증 후 랩타임 추이 갱신 -->

## 결과 먼저: 완주 데모

<div class="left compact">

### 랩타임 추이

| 주행 | 기록 |
|---|---|
| IMG_2986 | 30.90s |
| IMG_3018 | 28.86s |
| 목표 | **20s 이내** |

- 무충돌 완주가 1순위, 랩타임은 그 다음
- 개선의 각 단계가 뒤의 케이스 스터디 한 건씩에 대응

</div>

<div class="right placeholder large">
완주 주행 영상 대표 캡처<br>
(코너 아크 구간 프레임 추천)
</div>

---

## 발표 흐름

1. 프로젝트 목표와 트랙 제약
2. 하드웨어 · 시스템 아키텍처
3. FreeRTOS 펌웨어 구조와 센서 계층
4. 주행 제어: FSM + 캐스케이드 조향
5. 트러블슈팅 케이스 스터디
6. 텔레메트리와 검증 방법론
7. 결과 · 로드맵 · 배운 점

---

<!-- _class: split -->
<!-- TODO: testtrack.drawio 캡처 삽입 (구간 폭 라벨 포함 버전이면 베스트) -->

## 목표와 트랙 제약

<div class="left compact">

### 목표

- 무충돌 완주 최우선, 목표 랩타임 20s
- 코너는 정지 없는 전진 아크로 통과

### 제약

- 트랙 폭 37 → 45 → 43 → 55(45° 챔퍼 ×2) → 50 → **67(방지턱)** → 50 → 60cm
- 차체 27 × 16cm — 최협 구간 측면 여유 약 10cm
- 45° 코너와 90° 코너 혼재, 편도 코스

<span class="badge critical">critical</span> 여유 10cm에서는 heading 오차가 곧 충돌 — 자세 안정화가 핵심 문제다.

</div>

<div class="right placeholder large">
트랙 도면 캡처 영역<br>
testtrack.drawio + 구간 폭 표기
</div>

---

<!-- TODO: 블록도 제작 (MCU 중심, 센서/구동/통신 3그룹 + 버스·타이머 라벨) -->

## 시스템 아키텍처

<div class="placeholder large">
하드웨어 블록도 영역<br>
STM32F411 ← HC-SR04(TIM3 IC) · VL53L0X×2(I2C1) · BNO055(I2C1) · SG-207×2(TIM2)<br>
STM32F411 → L298N(TIM4 PWM) · HM-10 BLE(USART1)
</div>

<p class="caption">센서 4종 → 판단(FSM+조향) → 차동 구동, BLE 텔레메트리로 전 과정 관측 가능.</p>

---

## 하드웨어 구성

| 구성 | 연결 | 역할 |
|---|---|---|
| STM32F411CEU6 | 100MHz Cortex-M4, FreeRTOS | 메인 제어기 |
| HC-SR04 전방 | TIM3 CH1 입력캡처, 에코 버짓 6ms(~1m) | 정면 거리 · 코너 판정 |
| VL53L0X ToF ×2 | I2C1, XSHUT 순차 기동(좌측 0x60 재배치) | 좌우 벽 거리 |
| BNO055 IMU | I2C1, 부팅 상대축 heading | 차체 진행각 |
| SG-207 엔코더 ×2 | TIM2 (`delay_us`와 공유) | 휠 속도 (텔레메트리) |
| L298N + TT모터 4륜 | TIM4 PWM (CH1 우 / CH2 좌) | 차동 구동 |
| HM-10 BLE | USART1 9600 | 명령 수신 · 텔레메트리 10Hz |
| IWDG | 2.048s | 제어 루프 fail-safe |

---

<!-- _class: split -->
<!-- TODO: 태스크/큐 데이터 플로우 다이어그램 제작 -->

## 펌웨어 아키텍처 — FreeRTOS 3태스크

<div class="left compact">

### SensorTask

- 센서 측정 → median 필터 · 유효성 게이팅
- `DriveInputs`를 `driveQ`로 발행 (12cm 미만은 비상 이벤트)

### MotorTask

- `driveQ` 소비 → `Drive_Update()`, 수신 시에만 IWDG refresh
- **모터 명령의 유일한 발행자** — 경쟁 조건 차단

### BluetoothTask

- RX 명령 파싱 + TX 텔레메트리 10Hz

</div>

<div class="right placeholder flow">
SensorTask → driveQ → MotorTask → PWM<br>
BluetoothTask 양방향 · IWDG 경로 표시
</div>

---

<!-- _class: split -->
<!-- TODO: 센서 배치도(측면/평면) 또는 I2C 버스 구성도 삽입 -->

## 센서 계층 설계

<div class="left compact">

### 신뢰할 수 있는 입력 만들기

- ToF ×2 기본 주소 동일(`0x52`) → XSHUT 순차 기동, 좌측만 `0x60` 재배치 — 리셋 후에도 결정적 복구
- 초음파 에코 버짓 6ms(~1m) — **타임아웃은 "먼 거리"가 아니라 `f_valid=0`**
- BNO055는 부팅 상대축 사용 (실내 드리프트 대응)
- invalid → valid 첫 샘플은 LPF를 raw로 재초기화

<span class="badge solved">solved</span> 유효성 플래그를 모든 소비자에 강제 — "타임아웃 80cm = 개구부" 오독 버그 차단.

</div>

<div class="right placeholder flow">
센서 배치도 / I2C 버스 구성도 영역
</div>

---

## 주행 상태 머신

| 상태 | 역할 |
|---|---|
| `CRUISE (0)` | 직진 · 벽추종 · 코너 후보 판단 |
| `BRAKE (1)` | 정면 위험 감속 / 정지 |
| `SPIN (2)` | 피벗 회전 — **폴백 전용** |
| `REVERSE (3)` | 막힘 상황 후진 탈출 |
| `HOLD (4)` | 정지 유지 |
| `SIDE_AVOID (5)` | 측벽 근접 비상 회피 |
| `CORNER (6)` | 전진 아크 코너링 — **정상 경로** |

정상 코너 = `0→6→0`. `0→1→2` 반복이 보이면 코너 판정 문제 — 상태값이 텔레메트리 `st`와 1:1이라 로그만으로 진단 가능.

---

<!-- _class: split -->
<!-- TODO: 캐스케이드 블록 다이어그램 제작 (외루프→내루프→믹싱, 벽 반발 주입점 표시) -->

## 조향: 2단 캐스케이드 + 벽 반발

<div class="left compact">

### 외루프 — 어디를 볼 것인가

- 횡오차 `(L−R)` P/D → 목표 heading 오프셋
- 캡 ±8° — 코너 진입 자세 보호

### 내루프 — 어떻게 돌 것인가

- heading 오차 → 차동 duty + yaw-rate 댐핑

### ref-무관 안전층

- 11cm 점진 벽 반발 (양벽 시 센터링 스프링 합성)
- 9cm 근접 가드
- heading 기준이 오염돼도 동작하는 유일한 복구 조향력

</div>

<div class="right placeholder flow">
캐스케이드 제어 블록 다이어그램 영역
</div>

---

<!-- _class: split -->
<!-- TODO: duty-속도 특성(스톨 하한) 실측 그래프 or 개념도 삽입 -->

## 속도 제어와 액추에이터의 현실

<div class="left compact">

### 실측으로 확인한 물리 한계

- 직진 스톨 하한 **30% duty** — (0, 30) 명령은 실현 불가
- 피벗(4륜 스키드)은 돌파 duty가 그보다 **훨씬 높음** — 전 바퀴 횡슬립
- 하한 위반은 `drive_config.h` `#error` 컴파일 가드로 차단

### 속도 스케줄

- 폭 / 전방 거리 / heading 오차 기반 속도 캡
- sub-stall 구간은 `mix_substall()`: 요 모멘트는 보존, 공통 속도만 감속 → "보정 중 감속"

</div>

<div class="right placeholder flow">
duty–거동 특성 그래프 영역<br>
(직진 스톨 30% vs 피벗 돌파 duty)
</div>

---

<!-- _class: split -->
<!-- 케이스 공통 템플릿: 좌=증상(영상+관찰) / 우=원인→수정→검증. TODO: IMG_3014/3016 지그재그 구간 프레임 삽입 -->

## 케이스 #1 — 직선 지그재그 리밋사이클

<div class="left compact">

### 증상 <span class="badge critical">critical</span>

- 직선에서 좌우 발산 진동, 반대벽 오버슈트 반복
- steer가 캡(±18%)에 반복 포화

<div class="placeholder">
지그재그 구간 영상 프레임 영역
</div>

</div>

<div class="right compact">

### 근본 원인

sub-stall에서 안쪽 바퀴를 0으로 떨구며 바깥 바퀴는 유지 → 실제 차동이 명령의 2~3배로 **증폭** → 요 킥 → 리밋사이클

### 수정

`mix_substall()`: 안쪽 (0,30)% 구간에서 **요 모멘트는 명령 그대로 보존**, 공통 속도만 감속

### 검증 <span class="badge solved">solved</span>

직선 steer 포화 소멸 확인 — 텔레메트리 `steer` 필드로 정량 확인

</div>

---

<!-- _class: split -->
<!-- TODO: IMG_3012 포켓 왕복 회전 프레임 삽입 -->

## 케이스 #2 — 180° wrap 회전각 버그

<div class="left compact">

### 증상 <span class="badge critical">critical</span>

- 코너 포켓에서 좌↔우 왕복 헛돌기, 탈출 실패

<div class="placeholder">
포켓 왕복 회전 영상 프레임 영역
</div>

</div>

<div class="right compact">

### 근본 원인

`wrap180(now − entry)`는 180° 초과 회전을 표현 불가 — +179° → −180° 점프를 "역회전"으로 오판 → 방향 반전 → 무한 왕복

### 수정

프레임 증분 누적 `turn_accum_deg`로 교체 + 진행 ≥180°면 코스 기준 게이트 무효화(기준 노화 예외)

### 검증 <span class="badge solved">solved</span>

실주행에서 포켓 탈출 확인 — **원칙: 각도 진행량은 증분 누적으로**

</div>

---

<!-- _class: split -->
<!-- TODO: dash_board 웹앱 실화면 캡처 삽입 -->

## 텔레메트리 & 대시보드

<div class="left compact">

BLE 10Hz, 웹 대시보드와 인덱스 계약 — **필드는 append만, 변경 금지.**

```text
T,<t_ms>,<f cm>,<L mm>,<R mm>,
  <h×10>,<vL>,<vR>,<st>,<fl>,<steer>
```

- `st` = FSM 상태 1:1 → 로그만으로 상태 전이 복원
- `fl` 비트: f_valid / side_valid / imu_live / power / mode
- `steer` = 조향 출력 — 진동·포화 진단용
- RX: `A/M/U/D/L/R/S` + `#KEY=VAL` 런타임 튜닝

</div>

<div class="right placeholder large">
웹 대시보드 실화면 캡처 영역<br>
(실시간 그래프 + 상태 표시)
</div>

---

<!-- _class: split -->
<!-- TODO: 영상+로그 동기 분석 예시 캡처 (같은 시각 프레임과 로그 나란히) -->

## 검증 방법론 — 시뮬레이터 없는 회귀 테스트

<div class="left compact">

호스트 시뮬레이터 없음 → **유일한 회귀 테스트 = 실차.** 그래서 관측을 계층화했다.

1. **영상 + BLE 로그 동기 분석** — 프레임 타임스탬프와 `st/steer/fl` 대조
2. **SWD Live Expressions** — `dbg.*` 구조체로 내부 상태 실시간 관측
3. **수동주행 실측 캘리브레이션** — 트랙 각 지점의 L/R/F 실측값으로 코너 판정 임계 도출

<span class="badge solved">solved</span> 실측 조합 검사: 직선 오검출 0건, 코너 방향 오판 0건 (±2cm 범위 포함)

</div>

<div class="right placeholder large">
영상 프레임 + 동시각 BLE 로그<br>
동기 분석 예시 영역
</div>

---

<!-- TODO: 실차 검증 후 수치 갱신 — 이 슬라이드가 덱의 신뢰성 앵커 -->

## 결과와 현재 상태

<div class="kpi-row">
  <div class="kpi"><strong>28.86s</strong><span>완주 기록 (IMG_3018)</span></div>
  <div class="kpi"><strong>10.76%</strong><span>FLASH 사용 (56KB/512KB)</span></div>
  <div class="kpi"><strong>17.08%</strong><span>RAM 사용 (22KB/128KB)</span></div>
</div>

| 항목 | 상태 |
|---|---|
| 코너 진입/방향 판정 | 수동주행 실측 기반 재캘리브레이션 완료 |
| 마지막 대칭 코너 | 코스축 기반 우회전 폴백 추가 |
| 최신 Release 빌드 | `-Wall` 통과 — **실차 재검증 대기** |

<span class="badge critical">next</span> 다음 기준점 = 최신 hex 플래시 후 전체 주행 + BLE 로그 동시 기록.

---

<!-- _class: split -->

## 로드맵과 배운 점

<div class="left compact">

### 로드맵

1. 최신 Release 실차 검증 (무충돌 · 코너 방향 · 완주 시간)
2. 무충돌 확보 후 속도 5% 단위 상향 → 20s
3. 엔코더 기반 속도 PI (데이터는 이미 20ms 주기 수집 중)
4. 방지턱: 부하 보상 + IMU pitch 게이트

</div>

<div class="right compact">

### 배운 점 → 설계 원칙

- 각도 진행량은 **증분 누적** (wrap 차분 금지)
- duty 명령은 **실현 가능 범위** 안에서 — 컴파일 가드로 강제
- 액추에이터 명령은 **단일 발행자**
- 기준값 게이트에는 **"기준이 낡았을 때" 예외 경로** 필수
- 튜닝은 실측 먼저, 게인은 그 다음

</div>

---

<!-- _class: title -->
<!-- TODO: repo 링크 / 연락처 / QR 삽입 -->

# 정리

이 프로젝트의 핵심은 게인 튜닝이 아니라 **신뢰할 수 있는 기준계와 관측 체계**를 만든 것이다.

센서 유효성 → 기준계 안정화 → 텔레메트리 기반 정량 검증의 사이클로, 30.9s에서 28.9s까지 왔고 20s를 향해 간다.

<p class="caption">Repo · 연락처 영역 (TODO)</p>

---

<!-- _class: inverse -->

# Appendix

트러블슈팅 케이스 추가 4건 · 실측 캘리브레이션 · 프로토콜 상세 · 엔지니어링 원칙

---

<!-- _class: split -->
<!-- TODO: IMG_3009 제자리 왕복 프레임 삽입 -->

## A1. 케이스 — 피벗 스톨과 스키드 돌파 duty

<div class="left compact">

### 증상 <span class="badge critical">critical</span>

- 제자리 왕복 회전, 목표각 도달 전 멈춤 (IMG_3009)
- 이후 "턴 힘 부족" 재보고

<div class="placeholder">
제자리 왕복 회전 프레임 영역
</div>

</div>

<div class="right compact">

### 근본 원인

4륜 스키드 피벗은 전 바퀴 횡슬립 — 돌파 duty가 직진 스톨(30%)보다 훨씬 높은데 회전 PID 하한이 그 아래

### 수정

`TURN_SPEED 56 / TURN_INNER 34`, PID 하한 44/40 — 전부 `#error` 컴파일 가드로 고정

### 검증 <span class="badge solved">solved</span>

실주행 회전 응답 즉시 개선 확인

</div>

---

<!-- _class: split -->

## A2. 케이스 — 전방 타임아웃 시맨틱

<div class="left compact">

### 증상 <span class="badge critical">critical</span>

- 회전 탈출 게이트가 안 풀려 코너에서 고착
- 대시보드는 `f=80`으로 "멀쩡해 보임"

</div>

<div class="right compact">

### 근본 원인

에코 6ms 타임아웃(>1m)은 `f_valid=0`인데 탈출 게이트가 전부 `f_valid &&`로 잠김 — **표시값 80은 유효 신호가 아님** (`fl` b0 확인 필수)

### 수정

`front_open_at()` = valid면 거리 비교, invalid면 연속 miss 횟수로 개방 판단. 순간 결손은 `front_recent_below()`로 최근 median 유지

### 검증 <span class="badge solved">solved</span>

탈출 게이트 정상화. 잔여: 저장착 시 바닥 에코는 SW 구분 불가 — 하드웨어 장착 높이 이슈로 분리

</div>

---

<!-- _class: split -->

## A3. 케이스 — 모터 기동 시 IMU 브라운아웃

<div class="left compact">

### 증상 <span class="badge critical">critical</span>

- 출발/피벗 시작 순간 `imu_live` 사망
- IMU 의존 게이트가 시작부터 무력화

</div>

<div class="right compact">

### 근본 원인

모터 인러시 전류 → 전압 강하 → BNO055 응답 상실. 소프트웨어 버그가 아니라 **전원 하드웨어 특성**

### 수정

IMU 의존 게이트마다 IMU-무관 백스톱 시간창(`SPIN_BLIND_MS`, `LAUNCH_MS`) — 브라운아웃 동안 시간 기반으로 판단 유지

### 검증 <span class="badge solved">solved</span>

기동 직후 오동작 제거. 원칙: 센서 의존 게이트에는 항상 센서-무관 백스톱

</div>

---

<!-- _class: split -->
<!-- TODO: IMG_3013 벽 접착 프레임 삽입 -->

## A4. 케이스 — 직선 벽 접착

<div class="left compact">

### 증상 <span class="badge critical">critical</span>

- 벽에 붙은 뒤 스스로 못 떨어짐 (IMG_3013)

<div class="placeholder">
벽 접착 구간 프레임 영역
</div>

</div>

<div class="right compact">

### 근본 원인

① `forward_floor`가 sub-stall 안쪽 바퀴를 30%로 올려 명령 차동을 반토막
② 11cm 벽 반발이 단일벽 전용이라 양벽 직선에서 미동작 — 반발은 heading ref와 무관한 **유일한 복구 조향력**인데 꺼져 있었음

### 수정

벽 반발 상시 적용 + sub-stall 처리 경로 정리 (→ 본편 케이스 #1로 연결)

### 검증 <span class="badge solved">solved</span>

접착 해소 — 부작용(지그재그)은 케이스 #1에서 마무리

</div>

---

<!-- TODO: 실차 검증 결과로 "판정 결과" 열 채우기 -->

## A5. 수동주행 실측 → 판정 캘리브레이션

| 위치 | 실측 (cm) | 기대 판정 |
|---|---|---|
| 초기 좁은 직선 | L=20, R=15 | 직선 유지 |
| 첫 직각 코너 | F≈20, L=23, R=36 | 우회전 |
| 둘째 직각 코너 | L=33, R=17 | 좌회전 |
| 넓고 긴 직선 | L=24, R=19~20 | 직선 유지 |
| 곡선 진입 | F≈26, L=19~20, R=46~50 | 우회전 |
| 방지턱 전 마지막 직각 | L≈R | 코스축 폴백 우회전 |

- 실측 ±2cm 조합 검사: 직선 오검출 0, 코너 방향 오판 0
- 도출 임계: 일반 open 26 / 넓은 구간 30 / asym 10cm, `CORNER_ABORT` 24→16cm
- 마지막 대칭 코너: 전방 ≤34cm + \|L−R\|≤6cm + 시작축 대비 +90°±20° 3회 확인 시 우회전

---

<!-- _class: code-focus -->

# A6. BLE 프로토콜 상세

```text
TX @10Hz:
T,<t_ms>,<f cm>,<L mm>,<R mm>,<h×10>,<vL>,<vR>,<st>,<fl>,<steer>

st : 0 CRUISE / 1 BRAKE / 2 SPIN / 3 REVERSE / 4 HOLD / 5 SIDE_AVOID / 6 CORNER
fl : b0 f_valid / b1 side_valid / b2 imu_live / b3 power / b4 mode

RX:
1/0 전원 · A 자율 · M 수동 · U/D/L/R/S 수동 조작
#KEY=VAL : VT(목표속도) · TEL · HZ · ML/MR (런타임 튜닝)
```

부팅 시 모터 OFF — `A` 수신으로 시작. 기존 필드 인덱스는 웹앱과의 계약이라 변경 금지, 추가는 끝에 append.

---

## A7. 엔지니어링 원칙 (재발 방지 목록에서)

1. 각도 진행량은 증분 누적 — `wrap180(now−entry)` 금지
2. 모터 명령은 단일 발행자 (MotorTask)
3. duty는 실현 가능 범위 확인 — 스톨/돌파 하한을 컴파일 가드로
4. 기준값 게이트에는 "기준이 낡았을 때" 예외 경로
5. 유효성 플래그는 모든 소비자에 일괄 적용
6. invalid→valid 첫 샘플은 필터 raw 재초기화
7. 확인 카운터에는 방향 포함 (좌1+우1 ≠ 2연속)
8. 텔레메트리 인덱스 = 외부 계약, append-only
