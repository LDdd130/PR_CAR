/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bno055.c
  * @brief   GY-BNO055 IMU 드라이버 구현 (I2C1, NDOF). 블로킹 + 짧은 timeout.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bno055.h"
#include "i2c.h"   /* CubeMX 생성: hi2c1, MX_I2C1_Init */
#include "delay.h" /* delay_us — 버스 복구 시퀀스 타이밍 (TIM11 시작 후에만 호출됨) */

/* ---- 레지스터 (Page 0) ---- */
#define REG_CHIP_ID      0x00U   /* = 0xA0 */
#define REG_PAGE_ID      0x07U
#define REG_EUL_H_LSB    0x1AU   /* Heading LSB부터 6바이트: H,R,P (LSB/MSB) */
#define REG_CALIB_STAT   0x35U
#define REG_UNIT_SEL     0x3BU
#define REG_OPR_MODE     0x3DU
#define REG_PWR_MODE     0x3EU
#define REG_SYS_TRIGGER  0x3FU

/* ---- 값 ---- */
#define CHIP_ID_VAL      0xA0U
#define OPR_MODE_CONFIG  0x00U
#define OPR_MODE_NDOF    0x0CU   /* 9축 절대방위 융합 (지자기 포함 — 모터/L298N 자기간섭으로 미사용) */
#define OPR_MODE_IMU     0x08U   /* gyro+accel 융합 (지자기 제외). 주행 제어용 상대 heading:
                                    브러시드 모터+L298N 옆 지자기는 10~40° 점프 유발 → IMU 모드가 안정.
                                    드리프트 ~1-3°/min 이지만 제어는 수 초 윈도의 상대각만 사용.
                                    부팅 시 heading≈0 기준(절대 방위 아님). mag calib 비트는 항상 0. */
#define BNO_PWR_NORMAL   0x00U   /* (구 PWR_MODE_NORMAL — HAL legacy 매크로와 충돌해 개명) */

#define BNO_I2C_TIMEOUT  10U     /* ms. 버스 stall 시 모터 장시간 정지 방지 */
#define BNO_FAIL_RECOVER 10U     /* 읽기 연속 실패 N회 → I2C 버스 복구 시도 */

/* ---- 클럭 소스 선택 (heading 드리프트에 직접 영향) ----
 * 0 = 내부 오실레이터 (모든 보드에서 안전. 융합 드리프트 ~1-3°/min).
 * 1 = 외부 32.768kHz 크리스탈 (GY-BNO055/Adafruit류는 크리스탈 탑재 → 융합 정확도↑, heading 드리프트↓).
 *     ⚠ 크리스탈 미탑재 순정/저가 모듈에서 1로 두면 융합이 죽어 heading 상실 → 반드시 보드 확인 후 켤 것.
 * heading이 곧 진행각이라(z축 yaw) 이 값 하나가 heading-hold 품질을 좌우한다. 기본은 무조건 동작하는 0. */
#define BNO_USE_EXT_CRYSTAL  0

/* ---- 축 정렬(AXIS_MAP) 전제 ----
 * 이 드라이버는 AXIS_MAP_CONFIG(0x41)/SIGN(0x42)을 건드리지 않음 = 디폴트 P1 매핑 사용.
 * 전제: 보드를 차체에 '평평하게'(칩 Z축이 지면 수직 위) 장착. 그래야 Euler heading(=Z축 yaw)이
 * 곧 차량 xy평면 진행각이 된다. 보드를 세워/뒤집어 달면 heading이 roll/pitch 축에서 나와 조향이 무너짐.
 * 장착 방향이 다르면 여기서 AXIS_MAP을 재설정해야 함(현재는 평면 장착 가정). */

/* 1° = 16 LSB (UNIT_SEL 기본 degree) */
#define EUL_LSB_PER_DEG  16.0f

static uint8_t bno_fail_cnt = 0;

/* I2C 버스 wedge 복구: 노이즈/F4 아날로그필터 errata로 slave가 SDA를 물면 재부팅 전까지
 * 전 트랜잭션 실패(HAL_BUSY 고착) → IMU 영구 상실. 표준 복구: SCL 수동 9클럭으로 slave
 * 시프트레지스터를 비워 SDA를 놓게 하고 STOP 생성 후 페리프 재초기화.
 * PB8=SCL/PB9=SDA 하드코딩은 i2c.c MspInit과 동일 전제. 소요 <1ms — IWDG/제어주기 안전. */
static void bno_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};

    HAL_I2C_DeInit(&hi2c1);

    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);      /* SDA 해제 시도 */
    for (int i = 0; i < 9; i++)                              /* SCL 9펄스 (~100kHz) */
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(5);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);    /* STOP 조건: SCL high 상태에서 */
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);      /* SDA low→high */
    delay_us(5);

    MX_I2C1_Init();                                          /* MspInit이 AF_OD 복원 */
}

static HAL_StatusTypeDef bno_rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_8BIT, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, n, BNO_I2C_TIMEOUT);
    if (st == HAL_OK)
    {
        bno_fail_cnt = 0;
    }
    else if (++bno_fail_cnt >= BNO_FAIL_RECOVER)
    {
        bno_bus_recover();
        bno_fail_cnt = 0;
    }
    return st;
}

static HAL_StatusTypeDef bno_wr8(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR_8BIT, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, BNO_I2C_TIMEOUT);
}

void BNO055_HardReset(void)
{
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET); /* active-low */
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(700);   /* POR 부팅 ~650ms: 이후에야 I2C 응답 */
}

bool BNO055_Init(void)
{
    uint8_t id = 0;

    BNO055_HardReset();

    /* 부팅 직후 NACK 가능 → 재시도 */
    for (int i = 0; i < 10; i++)
    {
        if (bno_rd(REG_CHIP_ID, &id, 1) == HAL_OK && id == CHIP_ID_VAL) break;
        HAL_Delay(10);
    }
    if (id != CHIP_ID_VAL) return false;   /* 통신/주소/전원 문제 */

    /* CONFIG 모드에서 설정 */
    if (bno_wr8(REG_OPR_MODE, OPR_MODE_CONFIG) != HAL_OK) return false;
    HAL_Delay(20);

    bno_wr8(REG_PAGE_ID, 0x00);
    if (bno_wr8(REG_PWR_MODE, BNO_PWR_NORMAL) != HAL_OK) return false;
    HAL_Delay(10);

    /* SYS_TRIGGER.CLK_SEL: 0x00=내부 osc(안전), 0x80=외부 32.768kHz 크리스탈.
     * BNO_USE_EXT_CRYSTAL로 선택 — 외부 크리스탈은 heading 드리프트를 줄인다(보드에 크리스탈 있을 때만). */
    bno_wr8(REG_SYS_TRIGGER, BNO_USE_EXT_CRYSTAL ? 0x80U : 0x00U);
    HAL_Delay(10);

    bno_wr8(REG_UNIT_SEL, 0x00);   /* deg, m/s^2, dps, ℃, Windows orientation */

    /* 융합 시작 — IMU 모드(gyro+accel, 지자기 제외). NDOF는 OPR_MODE_NDOF로 복귀 가능 */
    if (bno_wr8(REG_OPR_MODE, OPR_MODE_IMU) != HAL_OK) return false;
    HAL_Delay(20);   /* CONFIG→operating 전환 ≥7ms */

    return true;
}

bool BNO055_ReadEuler(BNO055_Euler *e)
{
    uint8_t b[6];
    if (bno_rd(REG_EUL_H_LSB, b, 6) != HAL_OK) return false;

    int16_t h = (int16_t)((uint16_t)b[1] << 8 | b[0]);
    int16_t r = (int16_t)((uint16_t)b[3] << 8 | b[2]);
    int16_t p = (int16_t)((uint16_t)b[5] << 8 | b[4]);

    e->heading = (float)h / EUL_LSB_PER_DEG;
    e->roll    = (float)r / EUL_LSB_PER_DEG;
    e->pitch   = (float)p / EUL_LSB_PER_DEG;
    return true;
}

uint8_t BNO055_ReadCalibStatus(void)
{
    uint8_t c = 0;
    (void)bno_rd(REG_CALIB_STAT, &c, 1);
    return c;
}
