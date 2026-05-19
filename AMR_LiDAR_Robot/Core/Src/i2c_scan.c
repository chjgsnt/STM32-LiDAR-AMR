#include "i2c_scan.h"

#include "bringup_log.h"

#include "i2c.h"
#include "ssd1306.h"

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

static int16_t I2C_CombineInt16(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

static int32_t I2C_ScaleRawRounded(int16_t raw, int32_t multiplier, int32_t divisor)
{
    int32_t scaled = (int32_t)raw * multiplier;

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

void I2C_ReadMpu6500Raw(void)
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
        LOG_INFO("MPU6500 raw read failed, error=0x%08lX.", (unsigned long)HAL_I2C_GetError(&hi2c1));
        return;
    }

    int16_t ax = I2C_CombineInt16(raw[0], raw[1]);
    int16_t ay = I2C_CombineInt16(raw[2], raw[3]);
    int16_t az = I2C_CombineInt16(raw[4], raw[5]);
    int16_t gx = I2C_CombineInt16(raw[8], raw[9]);
    int16_t gy = I2C_CombineInt16(raw[10], raw[11]);
    int16_t gz = I2C_CombineInt16(raw[12], raw[13]);
    int32_t ax_centi_g = I2C_ScaleRawRounded(ax, 100, 16384);
    int32_t ay_centi_g = I2C_ScaleRawRounded(ay, 100, 16384);
    int32_t az_centi_g = I2C_ScaleRawRounded(az, 100, 16384);
    int32_t gx_tenth_dps = I2C_ScaleRawRounded(gx, 10, 131);
    int32_t gy_tenth_dps = I2C_ScaleRawRounded(gy, 10, 131);
    int32_t gz_tenth_dps = I2C_ScaleRawRounded(gz, 10, 131);

    LOG_INFO("MPU6500 raw: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d.",
             (int)ax,
             (int)ay,
             (int)az,
             (int)gx,
             (int)gy,
             (int)gz);
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
