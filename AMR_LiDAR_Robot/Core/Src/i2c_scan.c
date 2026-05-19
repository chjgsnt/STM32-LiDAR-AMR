#include "i2c_scan.h"

#include "bringup_log.h"

#include "i2c.h"
#include "ssd1306.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C1_SCL_GPIO_PORT GPIOB
#define I2C1_SCL_PIN GPIO_PIN_8
#define I2C1_SDA_GPIO_PORT GPIOB
#define I2C1_SDA_PIN GPIO_PIN_9
#define MPU6500_I2C_ADDR_7BIT 0x68U
#define MPU6500_RAW_DATA_START_REG 0x3BU
#define MPU6500_PWR_MGMT_1_REG 0x6BU
#define MPU6500_WHO_AM_I_REG 0x75U
#define MPU6500_RAW_DATA_LEN 14U
#define MPU6500_GYRO_CALIBRATION_SAMPLES 200U
#define MPU6500_ACCEL_LSB_PER_G 16384.0f
#define MPU6500_GYRO_LSB_PER_DPS 131.0f
#define MPU6500_COMPLEMENTARY_ALPHA 0.95f
#define MPU6500_LOG_INTERVAL_MS 500U
#define MPU6500_PI 3.14159265358979323846f

#ifndef MPU6500_LOG_RAW
#define MPU6500_LOG_RAW 0
#endif

#ifndef MPU6500_LOG_UNITS
#define MPU6500_LOG_UNITS 0
#endif

#ifndef MPU6500_LOG_ACCEL_ANGLE
#define MPU6500_LOG_ACCEL_ANGLE 0
#endif

#ifndef MPU6500_LOG_FUSED_ANGLE
#define MPU6500_LOG_FUSED_ANGLE 1
#endif

#ifndef MPU6500_LOG_FILTER_DEBUG
#define MPU6500_LOG_FILTER_DEBUG 0
#endif

static int32_t gyro_bias_x = 0;
static int32_t gyro_bias_y = 0;
static int32_t gyro_bias_z = 0;
static float complementary_pitch_deg = 0.0f;
static float complementary_roll_deg = 0.0f;
static uint32_t complementary_last_ms = 0U;
static bool complementary_initialized = false;
static float complementary_last_dt_s = 0.0f;
static uint32_t mpu6500_last_log_ms = 0U;

static int16_t I2C_CombineInt16(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

static int32_t I2C_ScaleRawRounded(int32_t raw, int32_t multiplier, int32_t divisor)
{
    int32_t scaled = raw * multiplier;

    if (scaled < 0)
    {
        return -((-scaled + (divisor / 2)) / divisor);
    }

    return (scaled + (divisor / 2)) / divisor;
}

static const char *I2C_FixedSign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

static uint32_t I2C_FixedWhole(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;
    return (uint32_t)(abs_value / decimal_scale);
}

static uint32_t I2C_FixedFraction(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;
    return (uint32_t)(abs_value % decimal_scale);
}

static int32_t I2C_ScaleFloatRounded(float value, float multiplier)
{
    float scaled = value * multiplier;

    if (scaled < 0.0f)
    {
        return (int32_t)(scaled - 0.5f);
    }

    return (int32_t)(scaled + 0.5f);
}

static HAL_StatusTypeDef I2C_ReadMpu6500RawValues(int16_t *ax,
                                                  int16_t *ay,
                                                  int16_t *az,
                                                  int16_t *gx,
                                                  int16_t *gy,
                                                  int16_t *gz)
{
    uint8_t raw[MPU6500_RAW_DATA_LEN] = {0U};
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1,
                                             (uint16_t)(MPU6500_I2C_ADDR_7BIT << 1),
                                             MPU6500_RAW_DATA_START_REG,
                                             I2C_MEMADD_SIZE_8BIT,
                                             raw,
                                             MPU6500_RAW_DATA_LEN,
                                             100U);

    if (ret != HAL_OK)
    {
        return ret;
    }

    *ax = I2C_CombineInt16(raw[0], raw[1]);
    *ay = I2C_CombineInt16(raw[2], raw[3]);
    *az = I2C_CombineInt16(raw[4], raw[5]);
    *gx = I2C_CombineInt16(raw[8], raw[9]);
    *gy = I2C_CombineInt16(raw[10], raw[11]);
    *gz = I2C_CombineInt16(raw[12], raw[13]);

    return HAL_OK;
}

void I2C_ReadMpu6500WhoAmI(void)
{
    uint8_t who = 0U;

    LOG_INFO("MPU6500 WHO_AM_I read start.");

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1,
                                             (uint16_t)(MPU6500_I2C_ADDR_7BIT << 1),
                                             MPU6500_WHO_AM_I_REG,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &who,
                                             1U,
                                             100U);

    if (ret == HAL_OK)
    {
        LOG_INFO("MPU6500 WHO_AM_I = 0x%02X.", who);
    }
    else
    {
        LOG_INFO("MPU6500 WHO_AM_I read failed, error=0x%08lX.", (unsigned long)HAL_I2C_GetError(&hi2c1));
    }
}

void I2C_WakeMpu6500(void)
{
    uint8_t value = 0x00U;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1,
                                              (uint16_t)(MPU6500_I2C_ADDR_7BIT << 1),
                                              MPU6500_PWR_MGMT_1_REG,
                                              I2C_MEMADD_SIZE_8BIT,
                                              &value,
                                              1U,
                                              100U);

    if (ret != HAL_OK)
    {
        LOG_INFO("MPU6500 wake failed, error=0x%08lX.", (unsigned long)HAL_I2C_GetError(&hi2c1));
        return;
    }

    HAL_Delay(100U);
}

void I2C_CalibrateMpu6500Gyro(void)
{
    int32_t sum_gx = 0;
    int32_t sum_gy = 0;
    int32_t sum_gz = 0;

    LOG_INFO("MPU6500 gyro calibration start, keep sensor still.");

    for (uint16_t i = 0U; i < MPU6500_GYRO_CALIBRATION_SAMPLES; i++)
    {
        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;
        HAL_StatusTypeDef ret = I2C_ReadMpu6500RawValues(&ax, &ay, &az, &gx, &gy, &gz);

        if (ret != HAL_OK)
        {
            LOG_INFO("MPU6500 gyro calibration read failed, error=0x%08lX.", (unsigned long)HAL_I2C_GetError(&hi2c1));
            return;
        }

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        HAL_Delay(5U);
    }

    gyro_bias_x = I2C_ScaleRawRounded(sum_gx, 1, MPU6500_GYRO_CALIBRATION_SAMPLES);
    gyro_bias_y = I2C_ScaleRawRounded(sum_gy, 1, MPU6500_GYRO_CALIBRATION_SAMPLES);
    gyro_bias_z = I2C_ScaleRawRounded(sum_gz, 1, MPU6500_GYRO_CALIBRATION_SAMPLES);

    LOG_INFO("MPU6500 gyro bias raw: gx=%ld, gy=%ld, gz=%ld.",
             (long)gyro_bias_x,
             (long)gyro_bias_y,
             (long)gyro_bias_z);
}

void I2C_ReadMpu6500Raw(void)
{
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    HAL_StatusTypeDef ret = I2C_ReadMpu6500RawValues(&ax, &ay, &az, &gx, &gy, &gz);

    if (ret != HAL_OK)
    {
        uint32_t now_ms = HAL_GetTick();

        if ((now_ms - mpu6500_last_log_ms) >= MPU6500_LOG_INTERVAL_MS)
        {
            mpu6500_last_log_ms = now_ms;
            LOG_INFO("MPU6500 raw read failed, error=0x%08lX.", (unsigned long)HAL_I2C_GetError(&hi2c1));
        }

        return;
    }

#if MPU6500_LOG_UNITS
    int32_t ax_centi_g = I2C_ScaleRawRounded(ax, 100, 16384);
    int32_t ay_centi_g = I2C_ScaleRawRounded(ay, 100, 16384);
    int32_t az_centi_g = I2C_ScaleRawRounded(az, 100, 16384);
#endif
    float gx_dps = (float)((int32_t)gx - gyro_bias_x) / MPU6500_GYRO_LSB_PER_DPS;
    float gy_dps = (float)((int32_t)gy - gyro_bias_y) / MPU6500_GYRO_LSB_PER_DPS;
#if MPU6500_LOG_UNITS
    float gz_dps = (float)((int32_t)gz - gyro_bias_z) / MPU6500_GYRO_LSB_PER_DPS;
    int32_t gx_tenth_dps = I2C_ScaleFloatRounded(gx_dps, 10.0f);
    int32_t gy_tenth_dps = I2C_ScaleFloatRounded(gy_dps, 10.0f);
    int32_t gz_tenth_dps = I2C_ScaleFloatRounded(gz_dps, 10.0f);
#endif
    float ax_g = (float)ax / MPU6500_ACCEL_LSB_PER_G;
    float ay_g = (float)ay / MPU6500_ACCEL_LSB_PER_G;
    float az_g = (float)az / MPU6500_ACCEL_LSB_PER_G;
    float accel_roll_deg = atan2f(ay_g, az_g) * 180.0f / MPU6500_PI;
    float accel_pitch_deg = atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g))) * 180.0f / MPU6500_PI;
    uint32_t now_ms = HAL_GetTick();
    bool should_log = ((now_ms - mpu6500_last_log_ms) >= MPU6500_LOG_INTERVAL_MS);

    if (complementary_initialized == false)
    {
        complementary_pitch_deg = accel_pitch_deg;
        complementary_roll_deg = accel_roll_deg;
        complementary_last_ms = now_ms;
        complementary_initialized = true;
        complementary_last_dt_s = 0.0f;
    }
    else
    {
        float dt = (float)(now_ms - complementary_last_ms) / 1000.0f;
        complementary_last_ms = now_ms;

        if ((dt <= 0.0f) || (dt > 0.1f))
        {
            dt = 0.0f;
        }

        complementary_last_dt_s = dt;
        complementary_pitch_deg = (MPU6500_COMPLEMENTARY_ALPHA * (complementary_pitch_deg + (gy_dps * dt))) +
                                  ((1.0f - MPU6500_COMPLEMENTARY_ALPHA) * accel_pitch_deg);
        complementary_roll_deg = (MPU6500_COMPLEMENTARY_ALPHA * (complementary_roll_deg + (gx_dps * dt))) +
                                 ((1.0f - MPU6500_COMPLEMENTARY_ALPHA) * accel_roll_deg);
    }

    if (should_log == false)
    {
        return;
    }

    mpu6500_last_log_ms = now_ms;

#if MPU6500_LOG_ACCEL_ANGLE
    int32_t accel_pitch_tenth_deg = I2C_ScaleFloatRounded(accel_pitch_deg, 10.0f);
    int32_t accel_roll_tenth_deg = I2C_ScaleFloatRounded(accel_roll_deg, 10.0f);
#endif
#if MPU6500_LOG_FUSED_ANGLE
    int32_t fused_pitch_tenth_deg = I2C_ScaleFloatRounded(complementary_pitch_deg, 10.0f);
    int32_t fused_roll_tenth_deg = I2C_ScaleFloatRounded(complementary_roll_deg, 10.0f);
#endif
#if MPU6500_LOG_FILTER_DEBUG
    int32_t filter_dt_millis = I2C_ScaleFloatRounded(complementary_last_dt_s, 1000.0f);
#endif

#if MPU6500_LOG_RAW
    LOG_INFO("MPU6500 raw: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d.",
             (int)ax,
             (int)ay,
             (int)az,
             (int)gx,
             (int)gy,
             (int)gz);
#endif
#if MPU6500_LOG_UNITS
    LOG_INFO("MPU6500: ax=%s%lu.%02lug ay=%s%lu.%02lug az=%s%lu.%02lug gx=%s%lu.%01ludps gy=%s%lu.%01ludps gz=%s%lu.%01ludps",
             I2C_FixedSign(ax_centi_g),
             (unsigned long)I2C_FixedWhole(ax_centi_g, 100),
             (unsigned long)I2C_FixedFraction(ax_centi_g, 100),
             I2C_FixedSign(ay_centi_g),
             (unsigned long)I2C_FixedWhole(ay_centi_g, 100),
             (unsigned long)I2C_FixedFraction(ay_centi_g, 100),
             I2C_FixedSign(az_centi_g),
             (unsigned long)I2C_FixedWhole(az_centi_g, 100),
             (unsigned long)I2C_FixedFraction(az_centi_g, 100),
             I2C_FixedSign(gx_tenth_dps),
             (unsigned long)I2C_FixedWhole(gx_tenth_dps, 10),
             (unsigned long)I2C_FixedFraction(gx_tenth_dps, 10),
             I2C_FixedSign(gy_tenth_dps),
             (unsigned long)I2C_FixedWhole(gy_tenth_dps, 10),
             (unsigned long)I2C_FixedFraction(gy_tenth_dps, 10),
             I2C_FixedSign(gz_tenth_dps),
             (unsigned long)I2C_FixedWhole(gz_tenth_dps, 10),
             (unsigned long)I2C_FixedFraction(gz_tenth_dps, 10));
#endif
#if MPU6500_LOG_ACCEL_ANGLE
    LOG_INFO("MPU6500 angle accel: pitch=%s%lu.%01ludeg roll=%s%lu.%01ludeg",
             I2C_FixedSign(accel_pitch_tenth_deg),
             (unsigned long)I2C_FixedWhole(accel_pitch_tenth_deg, 10),
             (unsigned long)I2C_FixedFraction(accel_pitch_tenth_deg, 10),
             I2C_FixedSign(accel_roll_tenth_deg),
             (unsigned long)I2C_FixedWhole(accel_roll_tenth_deg, 10),
             (unsigned long)I2C_FixedFraction(accel_roll_tenth_deg, 10));
#endif
#if MPU6500_LOG_FUSED_ANGLE
    LOG_INFO("MPU6500 angle fused: pitch=%s%lu.%01ludeg roll=%s%lu.%01ludeg",
             I2C_FixedSign(fused_pitch_tenth_deg),
             (unsigned long)I2C_FixedWhole(fused_pitch_tenth_deg, 10),
             (unsigned long)I2C_FixedFraction(fused_pitch_tenth_deg, 10),
             I2C_FixedSign(fused_roll_tenth_deg),
             (unsigned long)I2C_FixedWhole(fused_roll_tenth_deg, 10),
             (unsigned long)I2C_FixedFraction(fused_roll_tenth_deg, 10));
#endif
#if MPU6500_LOG_FILTER_DEBUG
    LOG_INFO("MPU6500 filter: dt=%s%lu.%03lus alpha=0.95",
             I2C_FixedSign(filter_dt_millis),
             (unsigned long)I2C_FixedWhole(filter_dt_millis, 1000),
             (unsigned long)I2C_FixedFraction(filter_dt_millis, 1000));
#endif
}

static void I2C_RunOledDisplayTest(uint8_t addr)
{
    if (SSD1306_Init(&hi2c1, addr) == false)
    {
        LOG_INFO("OLED SSD1306 init failed at 0x%02X.", addr);
        return;
    }

    (void)SSD1306_Clear();
    (void)SSD1306_WriteString(0U, 0U, "OLED OK");
    (void)SSD1306_WriteString(0U, 2U, "I2C: 0x3C");

    if (SSD1306_UpdateScreen() == true)
    {
        LOG_INFO("OLED display test written at 0x%02X.", addr);
    }
    else
    {
        LOG_INFO("OLED display update failed at 0x%02X.", addr);
    }
}

void I2C_ScanBus(void)
{
    uint8_t found_count = 0U;
    bool oled_found = false;
    HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(&hi2c1);
    uint32_t i2c_error = HAL_I2C_GetError(&hi2c1);
    GPIO_PinState scl_level = HAL_GPIO_ReadPin(I2C1_SCL_GPIO_PORT, I2C1_SCL_PIN);
    GPIO_PinState sda_level = HAL_GPIO_ReadPin(I2C1_SDA_GPIO_PORT, I2C1_SDA_PIN);

    LOG_INFO("I2C scan started.");
    LOG_INFO("I2C1 state before scan: 0x%02X.", (unsigned int)i2c_state);
    LOG_INFO("I2C1 error before scan: 0x%08lX.", (unsigned long)i2c_error);
    LOG_INFO("I2C1 SCL pin: PB8.");
    LOG_INFO("I2C1 SDA pin: PB9.");
    LOG_INFO("I2C1 SCL level before scan: %u.", (unsigned int)scl_level);
    LOG_INFO("I2C1 SDA level before scan: %u.", (unsigned int)sda_level);

    for (uint8_t addr = 0x01U; addr <= 0x7FU; addr++)
    {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2U, 50U);

        if (status == HAL_OK)
        {
            LOG_INFO("I2C device found at 0x%02X.", addr);

            if ((addr == 0x3CU) || (addr == 0x3DU))
            {
                LOG_INFO("OLED found at 0x%02X.", addr);
                oled_found = true;

                if (addr == SSD1306_I2C_ADDR_7BIT)
                {
                    I2C_RunOledDisplayTest(addr);
                }
            }

            found_count++;
        }
        else if (status == HAL_BUSY)
        {
            LOG_INFO("I2C HAL_BUSY at 0x%02X, error=0x%08lX.", addr, (unsigned long)HAL_I2C_GetError(&hi2c1));
        }
        else if (status == HAL_TIMEOUT)
        {
            LOG_INFO("I2C HAL_TIMEOUT at 0x%02X, error=0x%08lX.", addr, (unsigned long)HAL_I2C_GetError(&hi2c1));
        }
        else if (status == HAL_ERROR)
        {
            LOG_INFO("I2C HAL_ERROR at 0x%02X, error=0x%08lX.", addr, (unsigned long)HAL_I2C_GetError(&hi2c1));
        }
        else
        {
            LOG_INFO("I2C status=%u at 0x%02X, error=0x%08lX.", (unsigned int)status, addr, (unsigned long)HAL_I2C_GetError(&hi2c1));
        }
    }

    if (found_count == 0U)
    {
        LOG_INFO("No I2C devices found.");
    }

    if (oled_found == false)
    {
        LOG_INFO("OLED not found.");
    }

    LOG_INFO("I2C scan finished.");
}
