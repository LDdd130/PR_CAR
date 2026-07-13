# PR_CAR 인수인계 (HANDOVER)

> STM32F411 + FreeRTOS 차동구동 벽추종 자율주행 RC카.
> 최종 갱신: **2026-07-11 (7차)** — 기존 리팩터링/Claude 수정 이력 + Codex의
> IMG_3016/IMG_3018 분석 + 기준 YouTube 영상(18.93s) 비교 + 사용자 수동주행 센서 실측 반영.
> 작성: Claude + Codex (영상 프레임, testtrack.drawio, 실측값, git diff, 실코드 대조).
>
> 표기: 코드로 검증 못 한 내용은 **[추정]**. 코드/diff 사실이 세션 대화보다 우선.

---

## 0. TL;DR — 다음 에이전트가 알아야 할 것

1. **아키텍처**: `drive.c`(FSM) / `drive_control.c`(CRUISE 조향·속도) / `drive_config.h`(모든 튜닝 노브)
   / `drive_math.h`(유틸) / `motor.c`(`Motor_SetWheels` 단일 경로). 2026-07-10~11 Codex가 1,609줄
   drive.c를 이렇게 분리했고, 이후 Claude/Codex가 실주행-기반 수정을 이어갔다 (§5).
2. **현재 상태**: IMG_3018은 28.86s이고 코너가 2~4s씩 걸리며 마지막 벽을 오래 미는 문제가 있었음.
   이를 고친 최신 Release는 **빌드만 검증, 실차 미검증** (§5.5~5.6). 목표는 무충돌 우선,
   가능하면 testtrack.drawio를 20s 이내 완주.
3. 빌드: `make -j4` → `build/PR_CAR.hex`. 검증 수단 = 실차 + BLE 대시보드(`st`/`steer` 필드) + SWD `dbg`.
4. 이 문서의 §6(재발 방지 목록)과 §8(불변식)을 코드 수정 전에 반드시 읽을 것.
5. 튜닝 철학: 무충돌 우선. 센서 실측 범위로 코너/직선을 먼저 분리하고, 그 다음 속도를 5~10%씩 조정.
6. 작업 트리는 원래부터 대규모 dirty/untracked 상태다. **`git reset --hard`, `git clean` 금지.**

---

## 1. 시스템 개요

- MCU STM32F411(Cortex-M4) + FreeRTOS(CMSIS-OS2). IWDG 2.048s(큐 수신 성공 시에만 refresh).
- 센서: HC-SR04 전방(TIM3 CH1 입력캡처, **에코 대기 6ms = 최대 ~1m**), VL53L0X 측면 ToF ×2
  (I2C1, XSHUT 순차 기동으로 좌측 0x60 재배치, 버짓 20ms), BNO055 IMU(heading, 부팅 시 180° 재영점),
  SG-207 휠 엔코더 ×2(TIM2 — **delay_us와 공유, CNT 리셋 금지**; 아직 제어 미사용, 텔레메트리 전용).
- 액추에이터: L298N + TT모터 4륜 (TIM4 PWM ch1=우/ch2=좌). **직진 스톨 하한 실측 30% duty.
  피벗(스키드 조향)은 돌파 듀티가 그보다 훨씬 높음** — 4바퀴 횡슬립.
- 통신: HM-10 BLE(USART1 9600). 웹앱 `docs/monitor_web_app/index.html`.
- 태스크: SensorTask(측정→`driveQ` full frame + 12cm 미만 front-only 비상 이벤트) →
  MotorTask(`Drive_Update` 소비, IWDG refresh) / BluetoothTask(RX 명령 + TX 텔레메트리 10Hz).
- 차체: 길이 27cm × 폭 16cm. 트랙(testtrack.drawio): 폭 37→45→43→(45°챔퍼×2, 55)→50→
  **67(방지턱 구간)**→50→60cm, 출발→도착 편도.

### 상태머신 (drive.c) — 텔레메트리 `st`와 1:1
`DS_CRUISE(0) → DS_BRAKE(1) → DS_SPIN(2) / DS_REVERSE(3) / DS_HOLD(4) / DS_SIDE_AVOID(5) / DS_CORNER(6)`
- 정상 코너 = `0→6→0` (전진 아크). 피벗(2)은 폴백. `0→1→2` 반복이면 코너 판정 문제.
- front-only 프레임(side_valid=0)은 12cm 비상 제동 전용 — FSM 카운터 오염 금지 (§6-9).

### CRUISE 조향 (drive_control.c) — 2단 캐스케이드
- 외루프: 횡오차(l−r) P/D → 목표 heading 오프셋 (캡 ±`CENTER_LATERAL_CMD_MAX_DEG`=8°)
- 내루프: heading 오차 → 차동 duty (`CENTER_HDG_KP_PCT_PER_DEG` 0.65) + yaw-rate 댐핑
- **ref 무관 보조력**: 11cm 점진 벽 반발(양벽 시 센터링 스프링으로 합성) + 9cm 근접 가드
- 속도: 폭/전방/측면/heading 오차 기반 캡 스케줄 + `mix_substall()` (§5.5)

---

## 2. 히스토리 요약 (2026-07-10 ~ 07-11)

| 단계 | 내용 | 검증 |
|---|---|---|
| Codex 리팩터링 | 모듈 분리, motor.c 재작성, 센서 유효성 체계, 안전 보완 다수 | 빌드만 |
| Claude 1차 (§5.1) | IMG_3009: 제자리 왕복 회전 → 피벗 스톨 하한 + front 타임아웃 탈출 게이트 | 실주행 개선 확인 |
| Claude 2차 (§5.1b) | 턴 힘 부족 → 피벗 듀티 스키드 돌파값으로 상향 (56/34, PID 44/40) | 실주행 "빠릿" 확인 |
| Claude 3차 (§5.2) | IMG_3012: 포켓 헛돌기 → turn_progress 누적화 + 코스 게이트 예외 | 실주행 개선 확인 |
| Claude 4차 (§5.3) | IMG_3013: 벽 접착 → 벽 반발 상시화 + sub-stall 반올림 수정 | 접착 해소, 지그재그 부작용 |
| Claude 5차 (§5.4) | IMG_3014: 직선 지그재그 → `mix_substall` 요-보존 감속 믹싱 | 후속 Codex 수정으로 대체 |
| Codex 6차 (§5.5) | IMG_3016/3018: 직선 필터·속도 완화, graze 안전 우회 제거, `56/0` 구조 아크 제거 | 빌드 통과, IMG_3018에서 구 수정 문제 확인 |
| Codex 7차 (§5.6) | 수동 실측값 + drawio: 구간별 코너 기준, 마지막 대칭 코너 전용 우회전 폴백 | 실차: 직진 좌우 위빙 (IMG_3028) |
| Claude 6차 (§5.7) | IMG_3028: 직진 위빙 → deadband를 헤딩 노이즈 항에만 적용 + 노이즈 노브 완화 | 실차: 위빙 잔존 (deadband 구성 자체가 결함) |
| Claude 7차 (§5.8) | 위빙 재발 → 헤딩오차 LPF + deadband 축소 + 측면 3cm 진짜 데드존; 코너 방향 = 전방벽+좌우비교 폴백 | 실차: 전방 튐 → 조기/오발 회전 |
| Claude 8차 (§5.9) | 전방 스파이크 → §5.8 44cm 폴백 철회; BRAKE=벽 검증(스파이크→크루즈 복귀) + 20cm 크립 확정 후 좌우 판정 | 실차: 직진 위빙 해소, 턴 이후 사행 (IMG_3029) |
| Claude 9차 (§5.10) | IMG_3029+mapp 교차분석: 크루즈 ref 90°축 스냅(45°대각 금지), 코너 게이트 68→44cm, side-only 30→36cm, TURN_MIN 45→30° | **최신, 실차 미검증** |

Codex 세션 원본: `C:\Users\user\CODEX_SESSION_{1,2,3}.jsonl` (1=메인, 2=제어 리뷰 서브에이전트,
3=영상분석 서브에이전트). 상세 복원은 git 히스토리의 본 문서 이전 판 참조.

---

## 3. 파일 맵 + 튜닝 노브 위치

| 파일 | 역할 | 주요 노브 |
|---|---|---|
| `Core/Inc/drive_config.h` | **모든 튜닝 상수** + `#error` 불변식 가드 | 전부 |
| `Core/Src/drive.c` | FSM, 회전 진행각 누적, front_open 판정 | (로직만) |
| `Core/Src/drive_control.c` | CRUISE 캐스케이드 조향/속도/믹싱 | (로직만) |
| `Core/Src/motor.c` | `Motor_SetWheels`, 방향 반전 중립, `Car_Brake` | — |
| `Core/Src/freertos.c` | 3태스크, 센서 필터/유효성, BLE 프로토콜 | 프로토콜 상수 |
| `Core/Src/ultra.c` | HC-SR04 측정 + `median_n` | — |

자주 만지는 노브 (전부 drive_config.h):
- 피벗: `TURN_SPEED 56`, `TURN_INNER 34`, `TURN_PID_MIN_PCT 44`, `TURN_PID_FINE_MIN_PCT 40`
- 코너 아크: `ARC_OUTER 72 / ARC_INNER 30`, 접근 `56/30`, 구조 피벗 `48/-32`, `ARC_MAX_MS 850`
- 직진 속도: 기본/직선고속/좁은/넓은 = `60/64/60/68`, `CENTER_MIN_SPEED_PCT 36`
- 벽 보호: `CENTER_SIDE_SOFT_CM 11 / HARD 8 / REPEL_KP 1.35`, `CENTER_NEAR_GUARD_CM 9 / KP 1.2`
- 전방 안전거리: danger/abort/stop/turn/clear/arc = `12/16/34/44/52/68cm`
- 코너 판정: 일반 open/asym-open/asym=`26/28/10cm`, 넓은 구간=`30/30/10cm`, 확인 2회
- 마지막 대칭 코너: 좌우차 `<=6cm`, 전방 `<=34cm`, 시작축 대비 heading `+90+/-20deg`, 3회 확인 후 우회전
- 직진 헤딩: lateral cmd cap `8deg`, heading deadband `1.8deg`, yaw deadband `6dps`, steer cap `18%`

---

## 4. 빌드·실행·검증

```
make -j4                # Debug → build/PR_CAR.hex
make RELEASE=1 -j4      # Release → build/release/PR_CAR.hex
```
- 툴체인: arm-none-eabi-gcc 14.3 (STM32CubeIDE 2.1.0 번들, PATH 등록됨). cmake는 PATH에 없음
  (CubeIDE plugins 경로에 있음). 호스트 gcc/시뮬레이터 없음 → **유일한 회귀 테스트 = 실차**.
- 모터 벤치: `drive_config.h` `MOTOR_TEST 1` → 전진/제동/피벗/후진 시퀀스.
- 텔레메트리: `"T,<t_ms>,<f cm>,<L mm>,<R mm>,<h×10>,<vL>,<vR>,<st>,<fl>,<steer>\n"` @10Hz.
  `fl` 비트: b0 f_valid, b1 side_valid, b2 imu_live, b3 power, b4 mode.
  RX: `1/0/A/M/U/D/L/R/S` + `#KEY=VAL`(VT/TEL/HZ/ML/MR). **부팅 시 모터 OFF — `A` 수신으로 시작.**
- 필드 추가는 프레임 끝 append만 (기존 인덱스 = 웹앱 계약).
- 환경변수 없음. 외부 의존성 없음 (HAL/FreeRTOS vendored). graphify: 코드 수정 후 `graphify update .`.

### 실주행 체크리스트 (플래시 직후 순서대로)
1. 아래 최신 Release를 플래시하고 BLE 로그 + 전체 영상을 동시에 기록.
2. 초기/넓은 직선: `st=0`, `steer`가 `+/-18`에 반복 포화되지 않는지 확인.
3. 정상 코너: `st=6`, `steer` 약 `+/-21`(72/30 아크), 0.5~1.0s 안에 `st=0` 복귀.
4. 가까운 벽/회전 부족 구조: `st=6`, `steer` 약 `+/-40`(48/-32 피벗). 벽을 밀면 안 됨.
5. 코너 미확정: 전방 34cm 이하에서 `st=1` 제동이 먼저 나와야 함. `st=0`으로 충돌하면 센서/판정 문제.
6. 마지막 방지턱 전 대칭 코너: 시작축 대비 heading 약 +90deg에서 3프레임 후 우회전하는지.
7. 방지턱: 진입 각도 정면에 가까운가? 턱 위에서 FSM 오동작 없나? (§7).

---

## 5. 후속 수정 상세 (전부 2026-07-11)

### 5.1 제자리 왕복 회전 (IMG_3009)
- `TURN_PID_*` 하한이 스톨 하한(30%) 미만이라 회전이 목표 전에 멈춤 → 34/32로 상향(이후 2차에서 44/40).
- **전방 초음파 타임아웃(>1m 무에코) 시 `f_valid=0`인데 SPIN/CORNER 탈출 게이트가 전부
  `f_valid &&`로 잠김** → `front_open_at(in, cm)` = `f_valid ? f≥cm : front_miss≥FRONT_FAIL_LIMIT(5)`
  로 교체. **대시보드 f=80은 f_valid=1을 의미하지 않는다 (b0 플래그 확인).**
- `REV_MAX_CHUNKS 1→2`, corner 진입 시 후진 예산 리셋, HOLD에 측면 복구 탈출 추가.

### 5.1b 피벗 = 스키드 조향 (사용자 "턴 힘 부족")
- 4륜 제자리턴은 전 바퀴 횡슬립 → 돌파 듀티 ≫ 직진 스톨 30%.
- `TURN_SPEED 42→56`, `TURN_INNER 20→34`(안쪽 역회전이 실제로 돌도록), PID 하한 44/40,
  turn_pid inner에 `MOTOR_MIN_PCT` 하한 + 컴파일 가드. 결과: "턴 빠릿" (사용자 확인).

### 5.2 코너 포켓 헛돌기 (IMG_3012)
- **`wrap180(heading−entry)`는 180° 초과 회전 표현 불가** — +179→−180 점프가 "역회전" 오판 →
  방향 반전 → 왕복 헛돌기. → `turn_progress_reset/update()` 프레임 증분 누적(`turn_accum_deg`)으로 교체.
- `forward_ok`(코스 역주행 방지 115°)에 **반회전 예외**: progress ≥ 180°면 코스 기준이 무효.
- 96° cutoff 탈출에 `front_open_at(FRONT_TURN_CM)` 추가 — 열린 방향 찾을 때까지 회전 탐색 유지.

### 5.3 직선 벽 접착 (IMG_3013)
- `forward_floor`가 sub-stall 안쪽 바퀴를 30%로 올려 **명령 차동을 반토막** → 벽에서 못 나옴.
- 11cm 점진 벽 반발이 `!pair_valid` 전용이라 양벽 직선에서 아예 안 돌았음 → 상시 적용.
  반발항은 heading_ref와 무관한 유일한 조향력 = ref가 벽 쪽으로 틀어졌을 때의 유일한 복구 수단.

### 5.4 직선 지그재그 (IMG_3014) — 과거 이력, §5.5가 대체
- 5.3에서 sub-stall 안쪽 바퀴를 0으로 떨굴 때 바깥 바퀴를 speed+steer로 유지 → 차동이 명령의
  2~3배로 증폭 → 요 킥 → 반대벽 오버슈트 → 지그재그 리밋사이클.
- `mix_substall(&l,&r)`: 안쪽이 (0,30)%면 **안쪽=0, 바깥=명령 차동(2×steer, 최소 30)** —
  요 모멘트는 컨트롤러 명령 그대로 보존, 공통 속도만 깎임 = "보정 중 감속".
- 여전히 과하면: `CENTER_HDG_KP_PCT_PER_DEG 0.65↓` 또는 `CENTER_STEER_SLEW_PCT_PER_S 250↓`(부드럽게),
  진동 주기가 길고 완만하면 `CENTER_YAW_KD_PCT_PER_DPS 0.36↑`.

### 5.5 Codex IMG_3016/IMG_3018 + 기준 영상 비교

#### 영상 관찰
- 기준 YouTube `oMZH6d0gJ1U`: 568 frames @30fps = **18.93s**. 코너가 대략 0.5~0.8s이며
  멈춰 벽을 밀지 않고 짧은 전진 아크로 직선에 연결됨.
- IMG_3018: 837 frames @28.999fps = **28.86s**. 10s 이후 코너당 2~4s, 마지막은 벽 앞에서
  장시간 회전. IMG_3016도 직선 리밋사이클과 늦은 코너 진입이 관찰됨.

#### 원인과 수정
- 과거 `ARC_TIGHT 56/0`: 이 4륜 스키드 차체에서는 안쪽 바퀴가 끌려 회전이 느려짐.
  삭제하고 구조 시 `CORNER_RESCUE 48/-32` 능동 피벗으로 교체. `motor.c`의 방향반전 1ms 중립 사용.
- 정상 코너는 `72/30` 아크. heading 누적 진행이 시간 대비 부족하거나 전방 30cm 이내면 구조 피벗.
  진행 55deg 이상 + 전방 확보 후 정상 아크 복귀.
- `graze`가 실제 정면 벽까지 무시하던 안전 공백 제거. 코너 미확정 상태에서 전방 34cm 이하면
  graze와 무관하게 제동. 코너 후보 확인 프레임도 가까운 벽이면 전진하지 않고 브레이크 유지.
- 전방 echo가 잠깐 빠질 때 `front_recent_below()`로 `FRONT_FAIL_LIMIT` 전까지 최근 낮은 median 유지.
- front 완전 상실 시에도 강한 측면 코너(open>=30cm, wide>=46cm)를 3회 확인하면 회전 가능.
- 직진: 측면 오차 미분/yaw-rate LPF 및 상한 추가, heading/yaw deadband 확대, steer cap/slew 완화.
  정상 sub-stall은 양 wheel 공통 lift로 연속화하고, 실제 벽 위험일 때만 0/차동 감속 사용.

### 5.6 사용자 수동주행 실측 + testtrack.drawio 전용 판정 — **최신**

웹앱 값은 L/R은 mm, 코드 입력은 cm다. 아래는 코드 단위로 환산한 값:

| 위치 | 실측 | 기대 판정 |
|---|---:|---|
| 초기 좁은 직선 | L=20, R=15cm 전후 | none/직선 |
| 첫 직각 코너 | F≈20, L=23, R=36cm | right |
| 둘째 직각 코너 | L=33, R=17cm | left |
| 넓고 긴 직선 | L=24, R=19~20cm | none/직선 |
| 곡선 진입 | F≈26, L=19~20, R=46~50cm | right |
| 방지턱 전 마지막 직각 | L≈R | generic none, course fallback right |

- 위 값과 각 `+/-2cm` 범위를 조합 검사: 초기/넓은 직선 오검출 0, 세 코너 방향 오판 0.
- 일반 open을 26cm로 올려 초기 `20/15` 및 넓은 직선 `24/19`를 코너로 보지 않게 함.
- wide open/asym을 `30/30/10cm`로 내려 `33/17` 좌회전과 `23/36` 우회전을 모두 검출.
- 실측 코너 F=20cm를 유효 코너로 받아야 하므로 `CORNER_ABORT_CM 24→16`; 구조 진입은 30cm.
- drawio 진행축: 출발은 북향, 마지막 방지턱 전 코너는 동향(+90deg)→남향 **우회전**.
  `symmetric_course_direction()`은 전방<=34cm, |L-R|<=6cm, heading=+90+/-20deg를 모두 만족할 때만
  right를 반환. 3회 확인하며, 일반 코너 및 BRAKE→SPIN tie에도 적용.
- BNO055 절대 방위가 아니라 부팅 상대축을 쓰며 주행은 약 20s이므로 이 한정된 course-axis 사용만 허용.
  다른 절대 heading 의존 로직으로 일반화하지 말 것.

### 5.7 직진 좌우 위빙 (IMG_3028) — **최신**

#### 관찰
- §5.6 릴리스 실주행(IMG_3028)에서 직진 구간 좌우 흔들림. 사용자 확인: 진동 때문에 직진 중에도
  heading이 양옆으로 몇 도씩 튀는 것이 정상 — 과하게 헤딩을 잡으려 하면 안 됨.

#### 원인 (구조 결함)
- `compute_steer` 양벽 브랜치가 `deadband(heading − ref + lateral_cmd)`로 **노이즈와 센터링 명령의
  합**에 deadband를 걸었음. 위치오차 약 7cm까지의 센터링 명령(<1.8°)이 전부 deadband에 흡수 →
  11cm 벽 반발 존 사이에서 위치 무규제 → 표류→반발 킥→반대편 표류의 저속 위빙 리밋사이클.
- deadband를 키우면(노이즈 억제) 위치 불감대가 커지고, 줄이면 노이즈가 통과 — 두 목적이 한 노브에
  묶여 튜닝 불가 상태였음. 단일벽 브랜치는 이미 heading 항만 deadband하는 올바른 구조.
- 요 댐퍼가 지터 미분을 증폭: cap 6% = 조향 권한(18%)의 1/3을 노이즈가 랜덤 주입.

#### 수정
- `drive_control.c`: `steer = KP × (deadband(heading − ref) + lateral_cmd)` — deadband는 측정
  노이즈 항에만, 센터링 명령은 직결 (단일벽 브랜치와 동일 구조로 통일).
- 노브: `CENTER_HDG_KP 0.65→0.50`(§5.4 처방), `CENTER_HDG_DEADBAND 1.8→2.5`(지터 대역, 이제
  위치제어와 무관하므로 안전), `CENTER_YAW_LPF_ALPHA 0.35→0.22`, `CENTER_YAW_RATE_DEADBAND 6→10`,
  `CENTER_YAW_DAMP_MAX 6→4`.
- 실측 판정(§5.6) 및 코너 FSM 무변경. 빌드 통과, 실차 미검증.
- **실차 결과: 위빙 잔존.** `deadband(노이즈)+명령` 구성은 평형점이 deadband 가장자리로 밀려
  임의의 작은 보정에도 차체가 2.5°+명령만큼 크랩 → 오버슈트 → 반대편 반복. §5.8이 대체.

### 5.8 위빙 재발 + 코너 방향 단순화 (사용자 폼보드 실주행) — **최신**

#### 교훈 (§5.7의 오류)
- hard deadband는 어느 쪽에 걸어도 결함: 합에 걸면 위치 무규제 대역(§5.7 원인), 노이즈 항에만
  걸면 평형점이 대역 가장자리로 이동(달성 헤딩 = deadband+명령, 소명령 증폭). **노이즈는 주파수
  영역에서(LPF), deadband는 채터 방지용 소폭만.**

#### 수정 1 — 직진 조향
- `CENTER_HDG_ERR_LPF_ALPHA 0.25` 신설: 조향/속도캡용 헤딩오차를 1-pole LPF(fc≈2.3Hz)로 필터.
  진동 지터(±2~3°, >5Hz) 3~5배 감쇠, 실제 선회(<2Hz)는 통과. href 게이트는 raw 유지.
  기준축 스텝(fresh/resnap/IMU 부활) 시 필터 재시드 — 의도된 축 변경을 문지르지 않음.
- `CENTER_HDG_DEADBAND_DEG 2.5→0.6` (채터 방지 소폭), 구성은 `deadband(오차_f+명령)` 복원.
- 측면 데드존 진짜 불감대화: `CENTER_DEADZONE_CM 2→3`, `CENTER_INNER_KP_SCALE 0.35→0`.
  사용자 요구: ToF가 0.5cm를 확신 못 하면 미세 센터링은 하지 말 것 — |L−R|<3cm(중심이탈
  1.5cm)는 "이미 센터"로 간주, 헤딩 유지만 수행.

#### 수정 2 — 코너 방향 판정 (사용자 명세)
- `corner_direction()` 폴백 추가: asym/open 패턴 불충족이어도 **전방벽 확인(≤FRONT_TURN 44cm)
  시 좌우 비교(SIDE_HYST 6cm)로 방향 결정** → DS_CORNER 진입. 폼보드 등으로 좁아진 통로에서
  open 문턱(26/30cm)을 영원히 못 넘어 brake→spin으로 떨어지던 경로 제거.
- 실측 직선 |L−R|≤5cm < hyst 6 → 직선 오발화 없음. 대칭 코너(<6cm)는 여전히 -1 →
  코스축 우회전 폴백 소유. 센터링 데드존/LPF와 코너 판정은 비결합(코너는 raw 프레임 값 사용).

#### 산출물
- Debug/Release `-Wall` 무경고 빌드. 플래시: `build/release/PR_CAR.hex`.
- **실차 결과: 전방이 튀어 조기/오발 좌우회전.** 44cm 폴백이 스파이크 래치(`front_recent_below`)로
  발화 + `FRONT_STOP 34` 즉시 brake→spin이 스파이크 무검증. §5.9가 대체.

### 5.9 전방 스파이크 무력화 — 좌우 판정은 20cm 확정 후 (사용자 명세) — **최신**

#### 원인
- HC-SR04 전방이 튐(바닥 에코/바운스). 스파이크가 median(3)을 뚫으면:
  (a) §5.8의 44cm 폴백이 래치 기반 `front_recent_below`로 발화 → 직선 한복판 가짜 코너,
  (b) `FRONT_STOP 34` 단일 프레임 brake → `brake_run`이 80ms 후 **무검증 즉시 spin** → 가짜 피벗.

#### 수정 (사용자 명세: "전방 20cm 밑 확실히 확정되는 순간에 좌우회전 판단")
- §5.8의 44cm 좌우비교 폴백 **철회** — 약한 증거 회전 경로 자체 제거.
- `brake_run` 재작성 = 벽 검증 단계:
  1. 벽 증발(`!front_recent_below(44)` × `CLEAR_CONFIRM 3`) → **cruise 복귀** (스파이크 무해화).
  2. 벽 유지 + 전방 > 20cm → `BRAKE_CREEP_PCT 32`로 크립 접근 (최대 `BRAKE_CREEP_MAX_MS 700`).
  3. `f_valid && f <= FRONT_DECIDE_CM 20` **연속 `FRONT_DECIDE_CONFIRM_N 2` 프레임 (fresh만,
     dropout 래치 불인정)** → 정지 자세에서 좌우 비교 → `spin_enter`.
  4. 700ms 내 20cm 미해결(경사벽/흡음) → 현 위치에서 구판정 폴백 (교착 방지).
  5. `FRONT_DANGER 12` 미만 → 기존 reverse/spin 경로 그대로.
- 강한 asym/open 실코너 롤링 아크(§5.6)와 대칭 코너 코스축 우회전은 무변경.
  spin 진입이 ~20cm라 `symmetric_course_direction`의 front≤34 게이트 자연 충족.
- 컴파일 가드: `DANGER < DECIDE < STOP`, `BRAKE_CREEP_PCT ≥ MOTOR_MIN_PCT`.
- Debug/Release 무경고 빌드. 플래시: `build/release/PR_CAR.hex`.
- **실차 결과 (IMG_3029+mapp.mp4): 직진 위빙 해소 확인, 그러나 턴 이후 사행/상태 스래시.** §5.10이 원인 규명.

### 5.10 턴 이후 사행 — 영상+텔레메트리 교차분석 (IMG_3029.mov ↔ mapp.mp4) — **최신**

#### 분석 방법
- 웹앱 녹화(mapp.mp4, 37.9s)를 0.5s 간격 crop 시트로, 주행영상(IMG_3029.mov, 32.5s)을 1s
  몽타주로 추출해 상태/Δheading/F/L/R/steer 타임라인 재구성. 시작 오프셋 ≈1.5s, 웹앱이 끝에서 ~5s 김.

#### 규명된 3중 원인 (모두 데이터로 확인)
1. **45° 그리드 ref 스냅**: 첫 코너 오버슈트 Δ+115 → `course_grid_snap`(45° 단위)이 ref를
   **+135 대각선**으로 스냅 → 차가 복도를 대각 횡단 → F16 벽 → 스핀 → 반복 (t=2.5~8).
2. **코너 오발**: 복도에서 치우치기만 해도 비대칭이 생기는데 전방 게이트가 `FRONT_ARC 68cm`라
   F=46/49에서 CORNER 발화 (t=9.0: Δ−0.9 F46 L114/R299 → 가짜 우회전). front 타임아웃 시
   side-only 경로도 far=33cm로 발화 (t=28.5, 문턱 30).
3. **회전 양자화**: SPIN 최소 45°/목표 88° — 코너 후 20~30° 어긋남 보정에 88°를 돌려 반대로
   과회전, BRAKE/SPIN/REVERSE 스래시 (전체 타임라인의 대부분이 이 반복).

#### 수정
- `course_axis_snap()` 신설(90° 단위) — **크루즈 heading_ref는 코스축만 허용**: cruise_enter,
  launch, resnap 타깃, IMU 부활 4개소 교체. 45° `course_grid_snap`은 코너 그리드 탈출
  정렬 검사(§CORNER_GRID) 전용으로 유지.
- 코너 게이트 `front_near(68)` → `front_wall(FRONT_TURN 44)`: 실측 코너 진입 F=20~26이므로
  여유 충분, 오프센터 비대칭 오발(F≥46) 차단. `CORNER_SIDE_ONLY_OPEN_CM 30→36`(오프센터
  far 33 차단, 실측 side-only 코너 36~50).
- `TURN_MIN_DEG 45→30`: 전방 열림 확인(52cm×3) 시 30°부터 스핀 탈출 — 소각 보정 허용.
- 부가 발견: **우측 엔코더 전 구간 0 cm/s** (좌만 17~76). 제어 미사용이라 주행 무영향이나
  속도 PI(R4) 전에 하드웨어 점검 필요.
- Debug/Release 무경고 빌드. 플래시: `build/release/PR_CAR.hex`. 실차 미검증.

#### 최신 빌드/산출물
- Debug/Release `-Wall` 빌드 통과, `git diff --check` 통과, 실측 판정 조합 테스트 통과.
- Release 메모리: FLASH 56,436B/512KB (10.76%), RAM 22,392B/128KB (17.08%).
- 플래시 파일: `build/release/PR_CAR.hex` (2026-07-11 18:19 KST 생성).
- `graphify update .` 완료: 3,192 nodes / 4,282 edges / 130 communities.
- **이 Release는 아직 실차 미검증. 다음 작업은 코드 수정이 아니라 먼저 전체 주행+BLE 로그 확보.**

---

## 6. 재발 방지 목록 (Codex+Claude가 만든/잡은 실수 — 코드 수정 전 필독)

1. 헤더 분리 시 매크로 참조 잔존/이중 정의 → 즉시 컴파일 확인.
2. 새 소스는 Makefile + CMakeLists.txt **양쪽** 등록.
3. 구조체 필드 추가 시 모든 생성 지점 초기화 (`DriveInputs` 사례: 스택 쓰레기값이 큐로).
4. LPF와 유효성 상호작용: invalid→valid 첫 샘플은 raw 재초기화 (`left_prev_valid` 패턴).
5. 유효성 플래그 도입 시 모든 소비자 일괄 적용 (한 곳만 고치면 80cm를 개구부로 오독).
6. **duty 명령은 실현 가능 범위 확인**: 직진 (0,30) 불가, 피벗 돌파는 그 이상. 컴파일 가드 활용.
   최신 `mix_substall`: 정상 센터링은 양 wheel 공통 lift로 명령 차동을 연속 보존하고, 실제 wall-risk에서만
   안쪽=0/바깥=차동으로 감속·회피한다. 과거 한쪽 0 고정 로직으로 되돌리면 지그재그가 재발한다.
7. 타이머 폴백이 실제 진행도(IMU)를 우회하지 않게; "이번 프레임 누락"≠"이 회전 내내 없음".
8. 확인 카운터에는 방향 포함 (좌1+우1 ≠ 2연속).
9. front-only 이벤트는 루프당 1회, FSM 카운터 접근 금지.
10. **각도 진행은 증분 누적** — `wrap180(now−entry)` 절대 금지 (180° wrap = 방향 반전 버그).
11. 기준값(heading_ref/course_heading) 게이트에는 항상 "기준이 낡았을 때" 예외 경로를 둘 것.
12. CubeMX regen: USER CODE 블록 밖 코드 소멸. `.ioc` = 소스오브트루스 (BT 스택 384, TIM2 PSC).
13. CRLF repo — `git diff --check` 습관화. graphify-out/은 생성물이라 whitespace 경고 무시.
14. 텔레메트리 인덱스/DS_ enum 값 = 웹앱 계약. 변경 금지, 추가는 append.

## 7. 남은 문제 (우선순위순)

1. **§5.6 최신 Release 실차 검증**: 무충돌 여부, 각 코너 방향, 마지막 대칭 우회전, 완주 시간 측정.
   실패 시 영상 타임스탬프와 같은 시각 BLE `F/L/R/h/st/fl/steer`를 함께 남길 것.
2. **20s 목표**: 무충돌 성공 후에만 속도 조정. 현재 60/64(기본/직선고속)를 한 번에 5% 이내 상향.
3. **방지턱(67cm 구간)**: 턱 위 스톨/미끄러짐 + 턱에서 센서 기하 붕괴(피치업 → 전방이 바닥/허공).
   근본 해결 = 엔코더 부하 보상(속도 PI, 구 R4 계획) + [추정] IMU pitch 게이트(|pitch| 큰 동안
   코너/브레이크 판정 유보). 엔코더 데이터는 이미 20ms마다 갱신됨(`dbg.v_l/v_r`) — 제어 연결만 남음.
4. `#VT` 속도 목표는 저장만 됨 (`dbg.v_target`) — 속도 PI 미구현.
5. 전방 초음파 저장착 이슈: 사용자가 하드웨어 재배치 [추정: 완료 여부 미확인]. 바닥 에코는
   소프트웨어로 구분 불가 — 직진 중 산발적 가짜 BRAKE가 보이면 먼저 장착 높이/각도 확인.
6. BNO055 heading은 실내에서 드리프트 — 절대값 의존 로직 추가 금지, 차분/단기 참조만.

## 8. 불변식 (깨면 조용히 망가짐)

- `FRONT_DANGER < FRONT_STOP < FRONT_TURN < FRONT_CLEAR < FRONT_ARC` (컴파일 가드 있음)
- `ARC_INNER, ARC_APPROACH_INNER, TURN_INNER, TURN_PID_*_MIN ≥ MOTOR_MIN_PCT` (컴파일 가드 있음)
- `CORNER_RESCUE_OUTER/INNER ≥ MOTOR_MIN_PCT`, `CORNER_ABORT < CORNER_TIGHTEN < FRONT_CLEAR` (가드 있음)
- `CENTER_LATERAL_CMD_MAX_DEG(8°) < CORNER_ENTRY_HDG_MAX_DEG(12°)` — 코너 접근 시 차체가 코스축에서
  과도하게 벗어나지 않도록 유지 (가드 없음, 수동 유지)
- 회전 진행각: 누적 방식 유지 (§6-10)
- IMU 의존 게이트에는 IMU-무관 백스톱 (`SPIN_BLIND_MS`, `LAUNCH_MS`) — 모터 기동 순간
  인러시 브라운아웃으로 imu_live가 죽는 하드웨어 특성 때문
- MotorTask만 모터 명령 발행 (예외: 폴트 핸들러 Car_Stop)
