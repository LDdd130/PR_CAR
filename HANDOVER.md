# PR_CAR 인수인계 (HANDOVER)

> STM32F411 + FreeRTOS 차동구동 벽추종 자율주행 RC카.
> 최종 갱신: **2026-07-13 (§5.12, Codex 11차)** — 실주행 영상+웹앱 텔레메트리 교차분석 기반 수정 반복 중.
> 표기: 코드로 검증 못 한 내용은 **[추정]**. 코드/diff 사실이 세션 대화보다 우선.

---

## 0. TL;DR — 다음 에이전트가 알아야 할 것

1. **아키텍처**: `drive.c`(FSM) / `drive_control.c`(CRUISE 캐스케이드 조향·속도) /
   `drive_config.h`(**모든 튜닝 노브** + `#error` 가드) / `drive_math.h`(유틸) / `motor.c`(단일 모터 경로).
2. **현재 상태**: IMG_3032↔mappv3에서 첫/둘째 코너 뒤 긴 직선의 오발 `CORNER`가 확인됨.
   §5.12에서 넓은 직선의 벽 밀착값이 느슨한 코너 기준으로 폴백되는 결함과, 계산만 하고 쓰지 않던
   전방 측벽반사(`graze`) 게이트를 수정 — **Debug/Release 빌드 검증, 실차 미검증**.
   목표: 무충돌 완주 우선, 이후 20s 이내.
3. **작업 방식**: 사용자가 주행영상(iPhone) + 웹앱 화면녹화를 거의 동시에 찍어 줌 →
   §6의 교차분석 워크플로로 오발 프레임을 특정 → 근본원인 수정. **증상 패치 금지, 근본원인만**
   (사용자 강한 선호). 하드웨어 문제면 코드로 덮지 말고 하드웨어라고 말할 것.
4. 빌드: `make -j8` / `make RELEASE=1 -j8` → `build[/release]/PR_CAR.hex`. 유일한 회귀 테스트 = 실차.
5. §7(재발 방지)과 §9(불변식)를 코드 수정 전에 반드시 읽을 것.
6. 작업 트리는 원래부터 대규모 dirty/untracked 상태다. **`git reset --hard`, `git clean` 금지.**
7. 코드 수정 후 `graphify update .` 실행 (AST-only, 무료).

---

## 1. 시스템 개요

- MCU STM32F411(Cortex-M4) + FreeRTOS(CMSIS-OS2). IWDG 2.048s(큐 수신 성공 시에만 refresh).
- 센서: HC-SR04 전방(TIM3 CH1 입력캡처, **에코 대기 6ms = 최대 ~1m, 초과 시 f_valid=0인데
  대시보드 F=80으로 표시됨**), VL53L0X 측면 ToF ×2(I2C1, 좌측 0x60 재배치), BNO055 IMU(heading,
  부팅 시 180° 재영점, **모터 인러시 브라운아웃으로 기동 순간 죽을 수 있음**),
  SG-207 휠 엔코더 ×2(TIM2 — delay_us와 공유, CNT 리셋 금지; 제어 미사용, 텔레메트리 전용.
  **우측 엔코더 전 구간 0 출력 — 하드웨어 점검 필요**).
- 액추에이터: L298N + TT모터 4륜 (TIM4 ch1=우/ch2=좌). **직진 스톨 하한 30% duty.
  피벗(스키드 조향)은 돌파 듀티가 그보다 훨씬 높음** — 4바퀴 횡슬립.
- 통신: HM-10 BLE(USART1 9600). 웹앱 `docs/monitor_web_app/index.html`.
- 태스크: SensorTask(측정→`driveQ` full frame + 12cm 미만 front-only 비상 이벤트) →
  MotorTask(`Drive_Update` 소비, IWDG refresh) / BluetoothTask(RX 명령 + TX 텔레메트리 10Hz).
- 차체 27×16cm. 트랙(testtrack.drawio): 폭 37→45→43→(45°챔퍼×2, 55)→50→**67(방지턱)**→50→60cm.
- 코스 축: 출발 북(0°) → 첫 직각 우회전(동 +90°) → 둘째 직각 좌회전(북 0°) → 넓고 긴 직선 →
  곡선 우회전 → … → 마지막 방지턱 전 직각: 동(+90°)→남(180°) 우회전 (좌우 대칭이라 기하 판정 불가,
  코스축 폴백 전용).

### 실측 센서 값 (사용자 수동주행, 코드 단위 cm — 판정 기준의 근거 데이터)

| 위치 | 실측 | 기대 판정 |
|---|---:|---|
| 초기 좁은 직선 | L=20, R=15 전후 | 직선 (코너 아님) |
| 첫 직각 코너 | F≈20, L=23, R=36 | right |
| 둘째 직각 코너 | L=33, R=17 | left |
| 넓고 긴 직선 | L=24, R=19~20 | 직선 |
| 곡선 진입 | F≈26, L=19~20, R=46~50 | right |
| 방지턱 전 마지막 직각 | L≈R, F≤34 | 코스축 폴백 right |
| **긴 직선 치우침 시 (오발 소스)** | far측 28.6~33.3 관측 | 직선 — 코너로 보면 안 됨 |

- 직진 중에도 진동으로 heading이 몇 도씩 튀는 게 정상 — 과하게 잡으면 안 됨 (사용자 확인).
- 트랙 측면 여유 ~10cm — 벽에 닿는 문제는 속도를 낮춰 풀지, 거리 문턱을 올려 풀지 말 것.

### 상태머신 (drive.c) — 텔레메트리 `st`와 1:1
`DS_CRUISE(0) → DS_BRAKE(1) → DS_SPIN(2) / DS_REVERSE(3) / DS_HOLD(4) / DS_SIDE_AVOID(5) / DS_CORNER(6)`
- 정상 코너 = `0→6→0` (전진 아크). 피벗(2)은 폴백. `0→1→2` 반복이면 코너 판정 문제.
- BRAKE는 §5.9부터 "벽 검증" 단계: 스파이크면 크루즈 복귀, 진짜 벽이면 20cm까지 크립 후 좌우 판정.
- front-only 프레임(side_valid=0)은 12cm 비상 제동 전용 — FSM 카운터 오염 금지.

### CRUISE 조향 (drive_control.c) — 2단 캐스케이드
- 외루프: 횡오차(l−r) P/D → 목표 heading 오프셋 (캡 ±8°). **|L−R|<3cm는 진짜 불감대**
  (ToF를 0.5cm까지 못 믿으므로 미세 센터링 안 함 — 사용자 방침).
- 내루프: heading 오차(**1-pole LPF α=0.25로 필터, deadband는 0.6° 채터방지만**) → 차동 duty
  + yaw-rate 댐핑(LPF 0.22, deadband 10dps, cap 4%).
- ref 무관 보조력: 11cm 점진 벽 반발 + 9cm 근접 가드.
- **heading_ref는 `course_axis_snap()`(90° 단위)만 사용.** 45° `course_grid_snap`은 코너
  그리드 탈출 정렬 검사 전용.

---

## 2. 히스토리 요약 (전체 상세는 git 히스토리의 본 문서 이전 판)

| 단계 | 내용 | 실차 결과 |
|---|---|---|
| Codex 리팩터링 | 모듈 분리 (drive.c 1,609줄 → 4파일) | — |
| Claude 1~3차 | 피벗 스톨/스키드 돌파 듀티, front 타임아웃 탈출(`front_open_at`), turn_progress 누적화 | 개선 확인 |
| Claude 4~5차 + Codex 6차 | 벽 반발 상시화, `mix_substall`, 직선 필터/속도 완화 | 부분 개선 |
| Codex 7차 | 실측값 기반 구간별 코너 기준 + 대칭 코너 코스축 우회전 폴백 | 직진 위빙 (IMG_3028) |
| Claude 6~7차 (§5.7~5.8) | 위빙: deadband 재구성 2회 실패 → **헤딩오차 LPF + deadband 0.6° + 측면 3cm 불감대**로 해결 | 위빙 해소 |
| Claude 8차 (§5.9) | 전방 스파이크 오발턴 → BRAKE=벽검증 + **20cm 크립 확정 후 좌우 판정** | 턴 이후 사행 (IMG_3029) |
| Claude 9차 (§5.10) | 교차분석: **ref 45°대각 스냅이 주범** → 90° 축 스냅, 코너 게이트 68→44, TURN_MIN 45→30 | 대폭 개선 (IMG_3030) |
| Claude 10차 (§5.11) | 긴 직선 오발턴 + SIDE_AVOID 데드락 (아래 §5.11) | **미검증** |
| Codex 11차 (§5.12) | IMG_3032↔mappv3: wide→normal 폴백·미사용 graze 게이트 수정, 확정 중 롤링 아크, wide 속도 -4.5% | **빌드 통과, 실차 미검증** |

---

## 3. 파일 맵 + 현재 노브 값 (2026-07-13 기준)

| 파일 | 역할 |
|---|---|
| `Core/Inc/drive_config.h` | **모든 튜닝 상수** + `#error` 불변식 가드 |
| `Core/Src/drive.c` | FSM, 코너/스핀/브레이크, 회전 진행각 누적, `course_axis_snap` |
| `Core/Src/drive_control.c` | CRUISE 캐스케이드 조향/속도/`mix_substall` |
| `Core/Src/motor.c` | `Motor_SetWheels`, 방향 반전 1ms 중립, `Car_Brake` |
| `Core/Src/freertos.c` | 3태스크, 센서 필터/유효성, BLE 프로토콜 |

자주 만지는 노브 (전부 drive_config.h):
- 피벗: `TURN_SPEED 56 / TURN_INNER 34`, PID 하한 `44/40`, `TURN_MIN_DEG 30`, `TURN_TARGET_DEG 88`
- 코너 아크: `ARC_OUTER 72 / ARC_INNER 30`, 구조 피벗 `CORNER_RESCUE 48/-32` (SIDE_AVOID 탈출도 이것)
- 전방 안전거리: danger/abort/**decide**/stop/turn/clear/arc = `12/16/20/34/44/52/68cm`
- 코너 판정: **전방 44cm 3프레임 연속확정(`FRONT_WALL_CONFIRM_N 3`) 필수** +
  open/asym-open/asym = `34/34/14cm`, wide = `34/34/12cm`, side-only(front 상실 시) open `36`, 확인 2회
- 벽 판정 후 좌우 결정: `FRONT_DECIDE_CM 20` fresh 연속 2프레임, `BRAKE_CREEP_PCT 32`, 캡 700ms
- 직진 조향: `CENTER_HDG_ERR_LPF_ALPHA 0.25`, `CENTER_HDG_KP 0.50`, deadband `0.6°`,
  lateral cap `8°`, 측면 불감대 `3cm`(내부 스케일 0), yaw 댐퍼 `0.24/10dps/4%`, steer cap `18%`
- 직진 속도: 기본/직선고속/좁은/넓은 = `60/64/60/65`, min `36` (넓은 settle `63`)

---

## 4. 빌드·실행·검증

```
make -j8                # Debug → build/PR_CAR.hex
make RELEASE=1 -j8      # Release → build/release/PR_CAR.hex  ← 주행용
```
- 툴체인: arm-none-eabi-gcc 14.3 (STM32CubeIDE 2.1.0 번들, PATH 등록됨). 호스트 테스트 불가 →
  **유일한 회귀 테스트 = 실차 + 영상 교차분석(§6)**.
- 모터 벤치: `MOTOR_TEST 1` → 전진/제동/피벗/후진 시퀀스.
- 텔레메트리: `"T,<t_ms>,<f cm>,<L mm>,<R mm>,<h×10>,<vL>,<vR>,<st>,<fl>,<steer>\n"` @10Hz.
  `fl` 비트: b0 f_valid, b1 side_valid, b2 imu_live, b3 power, b4 mode.
  RX: `1/0/A/M/U/D/L/R/S` + `#KEY=VAL`. **부팅 시 모터 OFF — `A` 수신으로 시작.**
- 필드 추가는 프레임 끝 append만 (기존 인덱스 = 웹앱 계약). DS_ enum 값 변경 금지.

### 실주행 체크리스트 (§5.12 빌드용)
1. 최신 Release 플래시, 주행영상 + 웹앱 화면녹화 동시 기록 (아래 §6 워크플로용).
2. 긴 직선: 한쪽 8~12cm/반대쪽 34~38cm/F≈37cm가 되어도 CORNER/BRAKE 없이 CRUISE 유지 +
   센터링 복귀. 이때 정상 `steer`는 대략 한 자릿수이며 21/40이면 상태 오발을 의심.
3. 둘째 코너(L33/R17): 아크(st=6)로 도는지, §5.9 폴백 피벗으로 도는지 관찰.
   피벗이면 open 문턱 34→32 미세조정 여지 (단 긴 직선 far 33.3 관측과 1cm 차이 — 신중히).
4. 벽 4~6cm 밀착 시: SIDE_AVOID가 3초 내 회전 탈출하는지 (구버전은 10초 데드락).
5. 마지막 대칭 코너: 코스축 +90° 부근에서 우회전 폴백.
6. 무충돌 완주 후에만 속도 상향 (한 번에 5% 이내).

---

## 5. 최신 수정 요지

### 5.12 wide→normal 폴백 + 미사용 graze 게이트 (IMG_3032 ↔ mappv3.mp4) — **최신, 실차 미검증**

영상 길이: 실주행 40.035s, 웹앱 45.733s. 시작은 거의 일치하며 첫 `CORNER`가 양쪽 약 0.8s.
정상 첫 코너는 t=0.8 `F30/L24.3/R42.7`, 정상 둘째 코너는 t=2.7
`F36/L43.6/R19.8`로, 열린 쪽 차이가 크면서 가까운 쪽도 15cm 이상이었다. 반면 긴 직선 오발은
t=5.0 `F37/L36.1/R10.5`에서 `CORNER`가 발화했다. 좌우 차이는 25.6cm로 오히려 더 크므로
**좌우 차 문턱만 올리는 것은 해결이 아니며**, 한쪽 벽에 10.5cm로 붙은 직선과 실제 개구부를 구분해야 한다.

근본원인:
1. `wide_corner_context()`가 넓이 판정과 `양쪽>=15cm`를 한 함수에 묶었다. 넓은 직선에서 한쪽이
   15cm 미만으로 붙는 순간 wide가 해제되어 더 느슨한 normal 기준(`near>=7`)으로 폴백했다.
2. `front_graze_suspected()`가 이 경우의 전방 측벽 반사를 올바르게 계산해도 `dbg.graze` 표시만 하고
   실제 코너/제동 게이트에는 사용하지 않았다.
3. §5.11의 `FRONT_WALL_CONFIRM_N=3`도 `front_recent_below()`를 써서 fresh 1회+dropout 래치가
   연속 3회처럼 누적될 수 있었다.

수정:
- 넓이 분류를 near 조건과 분리하고, wide 코너 증거에는 별도로 `near>=15cm`를 강제.
- `graze`이면 danger(12cm) 비상제동은 유지하되 일반 front-wall/STOP/TURN 게이트에서는 제외.
- front-wall 3연속 확인은 `f_valid` fresh 샘플만 누적.
- fresh 전방벽+강한 좌우 형상이 확인된 코너는 마지막 방향확인 프레임부터 `ARC 72/30`을 명령해
  BRAKE 후 피벗 대신 전진하며 턴을 시작. 전방 상실 side-only 경로는 기존처럼 정지 확인.
- 완주 속도를 크게 희생하지 않도록 문제 구간인 wide fast/settle만 `68/66 → 65/63`
  (약 4.4~4.5%)로 하향. 코너 아크 `72/30`과 나머지 속도는 유지.

기대되는 첫 연쇄 차단점은 t=5.0이다. 여기서 새 로직은 wide+near 부족+graze로 `corner_direction=-1`,
front 게이트 억제 후 CRUISE 벽 반발을 계속한다. 기존 영상의 t=5 이후 `CORNER→BRAKE→SPIN→REVERSE`
연쇄는 이 오발의 결과이므로 먼저 이 지점이 사라지는지 실차로 확인할 것.

### 5.11 긴 직선 간헐 오발턴 + SIDE_AVOID 데드락 (IMG_3030 ↔ mappv2.mp4) — **이전 빌드**

관측: (a) 긴 직선에서 살짝 치우침 → 비스듬한 벽 때문에 전방값 1~2프레임 딥 + far측 28.6/33.3cm
개방 → CORNER 오발 → 복구 스핀 (웹앱 t=4.5, 6.5). (b) 종반 벽 4cm 밀착 → SIDE_AVOID 탈출 아크
안쪽 12% duty < 스톨 30% → 스키드에서 회전 불능 → avoid→brake→cruise→재-avoid 10초 무한 루프.

수정: (1) `FRONT_WALL_CONFIRM_N 3` — 코너 게이트는 전방<44cm **3프레임 연속**일 때만.
(2) 개구 문턱 34cm 상향 (오발 far ≤33.3 < 34 ≤ 실코너 33~50; 경계 miss는 §5.9 폴백이 올바른
방향으로 흡수). (3) SIDE_AVOID 탈출 = `CORNER_RESCUE 48/-32` 능동 피벗 (`SIDE_ESCAPE_*` 삭제).

### 직전 수정들의 핵심 (§5.7~5.10 요지 — 자세한 서사는 git 이전 판)

- **위빙 해결 (§5.7~5.8)**: hard deadband는 어느 구성이든 결함 — 합에 걸면 위치 무규제,
  노이즈 항에만 걸면 평형점이 대역 가장자리로 이동. **노이즈는 LPF로, deadband는 0.6° 채터방지만.**
  기준축 스텝(fresh/resnap/IMU 부활) 시 LPF 재시드 필수.
- **전방 스파이크 (§5.9)**: 회전 방향 결정은 **fresh 판독 연속확정**으로만. `front_recent_below`는
  dropout 래치라 스파이크를 유지시킴 — 방향 결정에 쓰지 말 것. BRAKE = 벽 검증 단계
  (벽 증발 → 크루즈 복귀 = 스파이크 무해화).
- **턴 후 사행 (§5.10)**: 크루즈 heading_ref에 45° 스냅 금지 (코너 오버슈트 +115 → ref +135
  대각 고착 → 복도 대각 횡단). ref는 `course_axis_snap`(90°)만. SPIN은 전방 열리면(52cm×3)
  30°부터 탈출 — 88° 강제 회전이 스래시의 원인이었음.

---

## 6. 주행영상 ↔ 웹앱 녹화 교차분석 워크플로 (핵심 작업 방식)

사용자가 두 영상을 준다: **주행영상**(iPhone, 차 추적)과 **웹앱 화면녹화**(같은 주행의 텔레메트리).
길이가 다르다 (웹앱을 먼저 켜고 늦게 끔, 보통 4~5s 김). 이걸로 오발 프레임을 특정해 근본원인을 찾는다.
ffmpeg 없음, **python + opencv(cv2 4.12) 있음**. 산출물은 스크래치 디렉토리에.

### 6.1 절차
1. 메타데이터 확인 (fps/길이/해상도) → 두 영상 길이 차 기록.
2. 웹앱 프레임 1장 떠서 **크롭 좌표 캘리브레이션** (녹화마다 해상도 다름 — 아래 좌표는 참고치).
3. 웹앱을 0.5s 간격으로 크롭-조합한 "텔레메트리 시트" 생성 (13행/장) → 이미지로 판독.
4. 주행영상을 1s 간격 6×3 몽타주로 생성 → 차 위치/자세 판독.
5. **정렬**: 첫 코너(state CORNER 첫 등장 ↔ 영상에서 차가 첫 코너 도달)를 앵커로 오프셋 계산.
6. 상태/Δheading/F/L/R/steer 타임라인을 표로 재구성 → 오발 시점의 센서 값 조합을 실측표(§1)와
   대조 → 판정 로직의 어느 게이트가 뚫렸는지 특정.

### 6.2 웹앱 시트 생성 스크립트 (실사용본, 크롭 좌표만 조정)
```python
import cv2, numpy as np
out = r"<스크래치 경로>"
cap = cv2.VideoCapture(r"c:/Users/user/STM/PR_CAR/<웹앱녹화>.mp4")
rows = []
for t in np.arange(0.0, <길이>, 0.5):
    cap.set(cv2.CAP_PROP_POS_MSEC, t*1000)
    ok, fr = cap.read()
    if not ok: continue
    # 580x710 녹화 기준 크롭 (해상도 다르면 프레임 1장 떠서 재캘리브레이션):
    hdg   = fr[100:225, 150:440]   # Δheading 큰 숫자 + RAW
    lr    = fr[398:435, 10:570]    # 좌/우 mm 행
    front = fr[478:560, 60:260]    # 전방 cm
    spd   = fr[478:565, 295:570]   # 속도 + L/R 엔코더 + steer
    chip  = fr[578:632, 100:500]   # 상태 칩 (CRUISE/CORNER/SPIN/...)
    H = 58
    def rs(img):
        s = H / img.shape[0]
        return cv2.resize(img, (int(img.shape[1]*s), H))
    label = np.zeros((H, 90, 3), np.uint8)
    cv2.putText(label, f"{t:4.1f}", (2, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,255), 2)
    rows.append(cv2.hconcat([label, rs(chip), rs(hdg), rs(front), rs(spd), rs(lr)]))
cap.release()
W = max(r.shape[1] for r in rows)
rows = [cv2.copyMakeBorder(r, 0, 0, 0, W-r.shape[1], cv2.BORDER_CONSTANT) for r in rows]
for i in range(0, len(rows), 13):
    cv2.imwrite(f"{out}/sheet_{i//13}.png", cv2.vconcat(rows[i:i+13]))
```

### 6.3 주행영상 몽타주 스크립트
```python
cap = cv2.VideoCapture(r"c:/Users/user/STM/PR_CAR/<주행영상>.mov")
tiles = []
for t in np.arange(0.0, <길이>, 1.0):
    cap.set(cv2.CAP_PROP_POS_MSEC, t*1000)
    ok, fr = cap.read()
    if not ok: continue
    fr = cv2.resize(fr, (240, 427))
    cv2.putText(fr, f"{t:.0f}s", (6, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,255), 3)
    tiles.append(fr)
cap.release()
rowsimg = [cv2.hconcat(tiles[i:i+6]) for i in range(0, len(tiles)-5, 6)]
for j in range(0, len(rowsimg), 3):
    cv2.imwrite(f"{out}/grid_{j//3}.png", cv2.vconcat(rowsimg[j:j+3]))
```

### 6.4 판독 요령
- **Δ HEADING** = raw − 코스 기준축(부팅 방위). 코스축 대비 각: 첫 코너 후 +90, 둘째 후 0,
  마지막 남향 180. CRUISE인데 Δ가 축에서 20°+ 벗어나 있으면 ref 문제.
- **좌/우 mm**를 cm로 환산해 실측표(§1)와 대조. 오발 코너 = 판정 프레임의 far/diff가
  실측 코너 값에 못 미치는데 발화한 경우.
- **속도 우측 `L n R n`** = 엔코더 cm/s (duty 아님). 우측은 현재 항상 0 (하드웨어 이슈).
- **F=80 + FRONT 칩 회색** = 타임아웃(>1m)이지 80cm가 아님.
- 상태 칩이 0.5s 간격에서 CORNER/SPIN/BRAKE로 널뛰면 상태 스래시 — 어떤 게이트가 반복
  발화하는지 각 행의 센서 값으로 역추적.
- 오프셋: 시트의 첫 CORNER 시각 ↔ 몽타주의 첫 코너 도달 시각. 이번 두 세트는 각각 ~1.5s, ~2s였음.

---

## 7. 재발 방지 목록 (코드 수정 전 필독)

1. 헤더 분리 시 매크로 참조 잔존/이중 정의 → 즉시 컴파일 확인.
2. 새 소스는 Makefile + CMakeLists.txt **양쪽** 등록.
3. 구조체 필드 추가 시 모든 생성 지점 초기화 (`DriveInputs` 사례: 스택 쓰레기값이 큐로).
4. LPF와 유효성 상호작용: invalid→valid 첫 샘플은 raw 재초기화 (`left_prev_valid` 패턴).
5. 유효성 플래그 도입 시 모든 소비자 일괄 적용 (한 곳만 고치면 80cm를 개구부로 오독).
6. **duty 명령은 실현 가능 범위 확인**: 직진 (0,30) 불가, 피벗/탈출은 스키드 돌파 듀티 필요.
   sub-stall 안쪽 바퀴 아크로 벽에서 탈출하려던 SIDE_ESCAPE 40/12가 10초 데드락 만든 전례.
   컴파일 가드 활용.
7. 타이머 폴백이 실제 진행도(IMU)를 우회하지 않게; "이번 프레임 누락"≠"이 회전 내내 없음".
8. 확인 카운터에는 방향 포함 (좌1+우1 ≠ 2연속).
9. front-only 이벤트는 루프당 1회, FSM 카운터 접근 금지.
10. **각도 진행은 증분 누적** — `wrap180(now−entry)` 절대 금지 (180° wrap = 방향 반전 버그).
11. 기준값(heading_ref/course_heading) 게이트에는 "기준이 낡았을 때" 예외 경로를 둘 것.
12. CubeMX regen: USER CODE 블록 밖 코드 소멸. `.ioc` = 소스오브트루스 (BT 스택 384, TIM2 PSC).
13. CRLF repo — `git diff --check` 습관화. graphify-out/은 생성물이라 whitespace 경고 무시.
14. 텔레메트리 인덱스/DS_ enum 값 = 웹앱 계약. 변경 금지, 추가는 append.
15. **hard deadband로 노이즈를 막지 말 것** — LPF로 막고 deadband는 채터방지 소폭만 (§5.8 교훈).
16. **회전 방향 결정에 래치/스파이크 값 금지** — fresh 판독 연속확정만 (§5.9 교훈).
17. **크루즈 heading_ref는 90° 코스축만** — 45° 스냅은 대각 고착 사고 전례 (§5.10 교훈).
18. 코너 판정 문턱을 내리기 전에 "치우친 직선"이 그 값을 만들 수 있는지 실측표(§1)와 대조 (§5.11 교훈).

## 8. 남은 문제 (우선순위순)

1. **§5.12 Release 실차 검증** (체크리스트 §4). 실패 시 영상 2종 + §6 워크플로로 분석.
2. **20s 목표**: 무충돌 완주 후에만 속도 상향, 한 번에 5% 이내.
3. **방지턱(67cm 구간)**: 턱 위 스톨/피치업으로 전방이 바닥/허공 → 근본 해결 = 엔코더 속도 PI
   (R4 계획, `#VT` 저장만 됨) + [추정] IMU pitch 게이트. **선행: 우측 엔코더 0 출력 하드웨어 수리.**
4. 전방 초음파 저장착: 바닥 에코 = 소프트웨어 구분 불가. 직진 중 산발 가짜 BRAKE가 §5.9 검증으로
   무해화됐지만 빈발하면 장착 높이/각도부터 확인.
5. BNO055 heading 실내 드리프트 — 절대값 의존 로직 추가 금지, 차분/단기 참조만.

## 9. 불변식 (깨면 조용히 망가짐)

- `FRONT_DANGER < FRONT_DECIDE < FRONT_STOP < FRONT_TURN < FRONT_CLEAR < FRONT_ARC` (가드 있음)
- `ARC_INNER, ARC_APPROACH_INNER, TURN_INNER, TURN_PID_*_MIN, CORNER_RESCUE_*, BRAKE_CREEP_PCT
  ≥ MOTOR_MIN_PCT` (가드 있음)
- `CORNER_ABORT < CORNER_TIGHTEN < FRONT_CLEAR` (가드 있음)
- `CENTER_LATERAL_CMD_MAX_DEG(8°) < CORNER_ENTRY_HDG_MAX_DEG(12°)` (가드 없음, 수동 유지)
- 회전 진행각 누적 방식 유지 (§7-10)
- IMU 의존 게이트에는 IMU-무관 백스톱 (`SPIN_BLIND_MS 650`, `LAUNCH_MS 120`) — 모터 기동
  인러시 브라운아웃으로 imu_live가 죽는 하드웨어 특성
- MotorTask만 모터 명령 발행 (예외: 폴트 핸들러 Car_Stop)
- 크루즈 heading_ref = `course_axis_snap`(90°)만. BNO055 절대 방위 금지, 부팅 상대축만.
