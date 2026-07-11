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
| Codex 7차 (§5.6) | 수동 실측값 + drawio: 구간별 코너 기준, 마지막 대칭 코너 전용 우회전 폴백 | **최신, 실차 미검증** |

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
