/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vl53l0x.c
  * @brief   VL53L0X 경량 드라이버 구현 — 검증된 최소 초기화(Pololu vl53l0x-arduino 계열 이식)
  *          시퀀스: 2.8V 모드 → stop_variable 래치 → 리밋체크 완화 → SPAD 맵 재구성
  *                  → 기본 튜닝 블롭 → 인터럽트(GPIO) 설정 → VHV/Phase ref 캘리브레이션
  *          측정: back-to-back 연속모드 + RESULT_INTERRUPT_STATUS 폴링 (논블로킹)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "vl53l0x.h"

/* ---- 사용 레지스터 (데이터시트/ST API 명명) ---- */
#define REG_SYSRANGE_START               0x00U
#define REG_SYSTEM_SEQUENCE_CONFIG       0x01U
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0AU
#define REG_SYSTEM_INTERRUPT_CLEAR       0x0BU
#define REG_RESULT_INTERRUPT_STATUS      0x13U
#define REG_RESULT_RANGE_STATUS          0x14U  /* bits[6:3] = device range status code */
#define REG_RESULT_RANGE_MM              0x1EU  /* RESULT_RANGE_STATUS(0x14) + 10: 16-bit mm */
#define REG_FINAL_RANGE_MIN_CNT_RATE     0x44U
#define REG_MSRC_CONFIG_TIMEOUT_MACROP   0x46U
#define REG_PRE_RANGE_VCSEL_PERIOD       0x50U
#define REG_PRE_RANGE_TIMEOUT_HI         0x51U
#define REG_MSRC_CONFIG_CONTROL          0x60U
#define REG_FINAL_RANGE_VCSEL_PERIOD     0x70U
#define REG_FINAL_RANGE_TIMEOUT_HI       0x71U
#define REG_GPIO_HV_MUX_ACTIVE_HIGH      0x84U
#define REG_VHV_CONFIG_PAD_SCL_SDA_HV    0x89U
#define REG_I2C_SLAVE_DEVICE_ADDRESS     0x8AU
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_0 0xB0U  /* 6바이트 SPAD enable 맵 시작 */
#define REG_GLOBAL_CONFIG_REF_EN_START   0xB6U
#define REG_DYNAMIC_SPAD_NUM_REQ_REF     0x4EU
#define REG_DYNAMIC_SPAD_REF_EN_START    0x4FU
#define REG_IDENTIFICATION_MODEL_ID      0xC0U  /* == 0xEE */

#define VL53L0X_CAL_TIMEOUT_MS           500U   /* ref 캘리브레이션/SPAD info 대기 상한 */

/* ---- I2C 프리미티브 (성공=1) ---- */
static uint8_t wr_n(VL53L0X *d, uint8_t reg, const uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Write(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)buf, n, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK;
}

static uint8_t rd_n(VL53L0X *d, uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, n, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK;
}

static uint8_t wr8(VL53L0X *d, uint8_t reg, uint8_t v)  { return wr_n(d, reg, &v, 1); }
static uint8_t rd8(VL53L0X *d, uint8_t reg, uint8_t *v) { return rd_n(d, reg, v, 1); }

static uint8_t wr16(VL53L0X *d, uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };   /* 빅엔디언 */
    return wr_n(d, reg, b, 2);
}

static uint8_t rd16(VL53L0X *d, uint8_t reg, uint16_t *v)
{
    uint8_t b[2];
    if (!rd_n(d, reg, b, 2)) return 0;
    *v = (uint16_t)((b[0] << 8) | b[1]);
    return 1;
}

/* ---- SPAD 정보 (칩 NVM에서 reference SPAD 수/타입 추출 — ST 시퀀스 고정) ---- */
static uint8_t get_spad_info(VL53L0X *d, uint8_t *count, uint8_t *type_is_aperture)
{
    uint8_t ok = 1, tmp;

    ok &= wr8(d, 0x80, 0x01); ok &= wr8(d, 0xFF, 0x01); ok &= wr8(d, 0x00, 0x00);
    ok &= wr8(d, 0xFF, 0x06);
    ok &= rd8(d, 0x83, &tmp); ok &= wr8(d, 0x83, tmp | 0x04);
    ok &= wr8(d, 0xFF, 0x07); ok &= wr8(d, 0x81, 0x01);
    ok &= wr8(d, 0x80, 0x01);
    ok &= wr8(d, 0x94, 0x6B); ok &= wr8(d, 0x83, 0x00);
    if (!ok) return 0;

    uint32_t t0 = HAL_GetTick();
    do {
        if (!rd8(d, 0x83, &tmp)) return 0;
        if ((HAL_GetTick() - t0) > VL53L0X_CAL_TIMEOUT_MS) return 0;
    } while (tmp == 0x00U);

    ok &= wr8(d, 0x83, 0x01);
    ok &= rd8(d, 0x92, &tmp);
    *count = tmp & 0x7FU;
    *type_is_aperture = (uint8_t)((tmp >> 7) & 0x01U);

    ok &= wr8(d, 0x81, 0x00); ok &= wr8(d, 0xFF, 0x06);
    ok &= rd8(d, 0x83, &tmp); ok &= wr8(d, 0x83, tmp & ~0x04);
    ok &= wr8(d, 0xFF, 0x01); ok &= wr8(d, 0x00, 0x01);
    ok &= wr8(d, 0xFF, 0x00); ok &= wr8(d, 0x80, 0x00);
    return ok;
}

/* ---- 단일 ref 캘리브레이션 (VHV: vhv_init_byte=0x40, Phase: 0x00) ---- */
static uint8_t ref_calibration(VL53L0X *d, uint8_t vhv_init_byte)
{
    uint8_t s;
    if (!wr8(d, REG_SYSRANGE_START, 0x01U | vhv_init_byte)) return 0;

    uint32_t t0 = HAL_GetTick();
    do {
        if (!rd8(d, REG_RESULT_INTERRUPT_STATUS, &s)) return 0;
        if ((HAL_GetTick() - t0) > VL53L0X_CAL_TIMEOUT_MS) return 0;
    } while ((s & 0x07U) == 0U);

    if (!wr8(d, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) return 0;
    return wr8(d, REG_SYSRANGE_START, 0x00);
}

/* ---- ST 기본 튜닝 블롭 (공식 API DefaultTuningSettings 동일값 — 의미 비공개 레지스터 포함) ---- */
static const uint8_t vl53l0x_tuning[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

uint8_t VL53L0X_Init(VL53L0X *dev)
{
    uint8_t ok = 1, tmp;

    /* 0) 생존/정체 확인: 모델 ID 0xEE (부팅 완료 전이면 NACK → 실패 반환) */
    if (!rd8(dev, REG_IDENTIFICATION_MODEL_ID, &tmp) || tmp != 0xEEU) return 0;

    /* 1) 2.8V I/O 모드 (보드가 2.8V 풀업 레벨 — TOF2000C 모듈 표준) */
    ok &= rd8(dev, REG_VHV_CONFIG_PAD_SCL_SDA_HV, &tmp);
    ok &= wr8(dev, REG_VHV_CONFIG_PAD_SCL_SDA_HV, tmp | 0x01U);

    /* 2) 표준 모드 + stop_variable 래치 (StartContinuous 재시작 시퀀스에 필요) */
    ok &= wr8(dev, 0x88, 0x00);
    ok &= wr8(dev, 0x80, 0x01); ok &= wr8(dev, 0xFF, 0x01); ok &= wr8(dev, 0x00, 0x00);
    ok &= rd8(dev, 0x91, &dev->stop_variable);
    ok &= wr8(dev, 0x00, 0x01); ok &= wr8(dev, 0xFF, 0x00); ok &= wr8(dev, 0x80, 0x00);

    /* 3) MSRC/PRE_RANGE 리밋체크 해제 + 최소 신호율 0.25 MCPS (9.7 고정소수: 0.25*128=32) */
    ok &= rd8(dev, REG_MSRC_CONFIG_CONTROL, &tmp);
    ok &= wr8(dev, REG_MSRC_CONFIG_CONTROL, tmp | 0x12U);
    ok &= wr16(dev, REG_FINAL_RANGE_MIN_CNT_RATE, 32U);
    ok &= wr8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    if (!ok) return 0;

    /* 4) reference SPAD 맵 재구성 (NVM 권장 수/타입 반영 — 공장 캘리브레이션 보존) */
    uint8_t spad_count, spad_is_aperture;
    if (!get_spad_info(dev, &spad_count, &spad_is_aperture)) return 0;

    uint8_t spad_map[6];
    if (!rd_n(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_0, spad_map, 6)) return 0;

    ok &= wr8(dev, 0xFF, 0x01);
    ok &= wr8(dev, REG_DYNAMIC_SPAD_REF_EN_START, 0x00);
    ok &= wr8(dev, REG_DYNAMIC_SPAD_NUM_REQ_REF, 0x2C);
    ok &= wr8(dev, 0xFF, 0x00);
    ok &= wr8(dev, REG_GLOBAL_CONFIG_REF_EN_START, 0xB4);

    uint8_t first_spad = spad_is_aperture ? 12U : 0U;   /* aperture SPAD는 12번부터 */
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48U; i++)
    {
        if (i < first_spad || enabled == spad_count)
            spad_map[i / 8U] &= (uint8_t)~(1U << (i % 8U));
        else if ((spad_map[i / 8U] >> (i % 8U)) & 0x01U)
            enabled++;
    }
    ok &= wr_n(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_0, spad_map, 6);
    if (!ok) return 0;

    /* 5) 기본 튜닝 블롭 */
    for (uint16_t i = 0; i < (sizeof(vl53l0x_tuning) / sizeof(vl53l0x_tuning[0])); i++)
        if (!wr8(dev, vl53l0x_tuning[i][0], vl53l0x_tuning[i][1])) return 0;

    /* 6) 인터럽트: '새 샘플' 플래그 소스로만 사용 (GPIO 핀 미배선 — 상태 레지스터 폴링) */
    ok &= wr8(dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);   /* new sample ready */
    ok &= rd8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, &tmp);
    ok &= wr8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, tmp & ~0x10U);
    ok &= wr8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* 7) VHV + Phase ref 캘리브레이션 → 주행 시퀀스(DSS+PRE+FINAL=0xE8) 복원 */
    ok &= wr8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!ok || !ref_calibration(dev, 0x40)) return 0;
    if (!wr8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x02) || !ref_calibration(dev, 0x00)) return 0;
    return wr8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
}

/* ---- 측정 타이밍 버짓 (ST API SetMeasurementTimingBudget 이식) ----
 * 버짓 = 1회 레인징의 총 시간 → 연속모드 샘플 주기. 디폴트 ~33ms는 제어루프(~20ms)보다 느려
 * 측면 반응지연의 병목 — 20ms(ST 고속 프로파일)로 낮춰 루프당 새 샘플 1개를 보장한다.
 * 원리: 고정 오버헤드/활성 시퀀스 스텝 시간을 빼고 남는 시간을 final range 타임아웃으로 재배분 */

static uint32_t tmo_mclks_to_us(uint16_t mclks, uint8_t vcsel_pclks)
{
    uint32_t macro_ns = ((2304UL * vcsel_pclks * 1655UL) + 500UL) / 1000UL;
    return (((uint32_t)mclks * macro_ns) + 500UL) / 1000UL;
}

static uint32_t tmo_us_to_mclks(uint32_t us, uint8_t vcsel_pclks)
{
    uint32_t macro_ns = ((2304UL * vcsel_pclks * 1655UL) + 500UL) / 1000UL;
    return ((us * 1000UL) + (macro_ns / 2UL)) / macro_ns;
}

/* 칩 인코딩: 값 = LSByte × 2^MSByte + 1 */
static uint16_t tmo_encode(uint32_t mclks)
{
    uint32_t ls = mclks - 1UL;
    uint16_t ms = 0;
    if (mclks == 0UL) return 0;
    while (ls > 255UL) { ls >>= 1; ms++; }
    return (uint16_t)((ms << 8) | (ls & 0xFFUL));
}

static uint16_t tmo_decode(uint16_t v)
{
    return (uint16_t)(((v & 0x00FFU) << ((v >> 8) & 0x00FFU)) + 1U);
}

uint8_t VL53L0X_SetTimingBudget(VL53L0X *dev, uint32_t budget_us)
{
    /* ST 캘리브레이션 상수 [us] — 시퀀스 스텝별 고정 오버헤드 */
    const uint32_t kStart = 1320U, kEnd = 960U, kMsrc = 660U,
                   kTcc = 590U, kDss = 690U, kPre = 660U, kFinal = 550U;

    uint8_t seq, v;
    uint16_t enc;
    if (!rd8(dev, REG_SYSTEM_SEQUENCE_CONFIG, &seq)) return 0;
    uint8_t tcc   = (seq >> 4) & 0x01U;
    uint8_t dss   = (seq >> 3) & 0x01U;
    uint8_t msrc  = (seq >> 2) & 0x01U;
    uint8_t pre   = (seq >> 6) & 0x01U;
    uint8_t final = (seq >> 7) & 0x01U;
    if (!final) return 0;   /* final range 없는 구성엔 버짓 개념 없음 */

    if (!rd8(dev, REG_PRE_RANGE_VCSEL_PERIOD, &v)) return 0;
    uint8_t pre_vcsel = (uint8_t)((v + 1U) << 1);
    if (!rd8(dev, REG_MSRC_CONFIG_TIMEOUT_MACROP, &v)) return 0;
    uint32_t msrc_us = tmo_mclks_to_us((uint16_t)(v + 1U), pre_vcsel);
    if (!rd16(dev, REG_PRE_RANGE_TIMEOUT_HI, &enc)) return 0;
    uint16_t pre_mclks = tmo_decode(enc);
    uint32_t pre_us = tmo_mclks_to_us(pre_mclks, pre_vcsel);
    if (!rd8(dev, REG_FINAL_RANGE_VCSEL_PERIOD, &v)) return 0;
    uint8_t final_vcsel = (uint8_t)((v + 1U) << 1);

    uint32_t used = kStart + kEnd;
    if (tcc)      used += msrc_us + kTcc;
    if (dss)      used += 2UL * (msrc_us + kDss);
    else if (msrc) used += msrc_us + kMsrc;
    if (pre)      used += pre_us + kPre;
    used += kFinal;
    if (budget_us <= used) return 0;   /* 요청 버짓이 오버헤드보다 작음 — 재배분 불가 */

    uint32_t final_mclks = tmo_us_to_mclks(budget_us - used, final_vcsel);
    if (pre) final_mclks += pre_mclks;   /* 칩 규약: final 타임아웃 레지스터는 pre 몫 포함 */
    return wr16(dev, REG_FINAL_RANGE_TIMEOUT_HI, tmo_encode(final_mclks));
}

uint8_t VL53L0X_SetAddress(VL53L0X *dev, uint8_t new_addr_8bit)
{
    /* 레지스터엔 7-bit 값 기록. 성공 시에만 dev->addr 갱신 — 실패 시 기존 주소로 계속 통신 가능 */
    if (!wr8(dev, REG_I2C_SLAVE_DEVICE_ADDRESS, (uint8_t)((new_addr_8bit >> 1) & 0x7FU)))
        return 0;
    dev->addr = new_addr_8bit;
    return 1;
}

uint8_t VL53L0X_StartContinuous(VL53L0X *dev)
{
    uint8_t ok = 1;
    ok &= wr8(dev, 0x80, 0x01); ok &= wr8(dev, 0xFF, 0x01); ok &= wr8(dev, 0x00, 0x00);
    ok &= wr8(dev, 0x91, dev->stop_variable);
    ok &= wr8(dev, 0x00, 0x01); ok &= wr8(dev, 0xFF, 0x00); ok &= wr8(dev, 0x80, 0x00);
    ok &= wr8(dev, REG_SYSRANGE_START, 0x02);   /* back-to-back 연속측정 */
    return ok;
}

int8_t VL53L0X_PollRangeMM(VL53L0X *dev, uint16_t *out_mm)
{
    uint8_t s;
    if (!rd8(dev, REG_RESULT_INTERRUPT_STATUS, &s)) return -1;
    if ((s & 0x07U) == 0U) return 0;            /* 아직 새 샘플 없음 (버짓 ~33ms 진행 중) */

    uint8_t status;
    if (!rd8(dev, REG_RESULT_RANGE_STATUS, &status)) return -1;
    uint16_t mm;
    if (!rd16(dev, REG_RESULT_RANGE_MM, &mm)) return -1;
    if (!wr8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) return -1;

    /* 타깃이 언앰비규어스 사거리(~1.2m) 밖이면 칩은 8190이 아니라 wraparound된
     * "가짜 초근접 mm"(2~6cm)를 status=phase/signal fail과 함께 내보낸다.
     * 광장(곡선) 구간처럼 측벽이 사거리 밖일 때 이 값이 그대로 통과하면
     * 열린 쪽이 초근접 벽으로 뒤집혀 읽힌다 — status로 반드시 거른다.
     *   11 = valid만 실측으로 통과.
     *   7 = sigma fail: §5.18에서 통과시켰더니 개활 측면에서 0~37mm 쓰레기가
     *       그대로 새어나옴(§5.20, 20ms 버짓의 저신호 한계). 근접 실벽은 신호가
     *       강해 sigma fail이 뜨지 않으므로 out-of-range 처리가 안전하다.
     *   8/10 = min-range clip(초근접 실타깃, mm은 신뢰 불가 → 30mm 고정)
     *   그 외(phase/signal/HW fail) = 신뢰 타깃 없음 → out-of-range로 정규화 */
    uint8_t code = (uint8_t)((status >> 3) & 0x0FU);
    dev->last_code = code;
    if (code == 8U || code == 10U)
        mm = 30U;
    else if (code != 11U)
        mm = 8190U;

    *out_mm = mm;   /* out-of-range = 8190/8191 — 상한 캡은 호출측 */
    return 1;
}
