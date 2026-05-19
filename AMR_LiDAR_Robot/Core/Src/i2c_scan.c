#include "i2c_scan.h"

#include "bringup_log.h"

#include "i2c.h"

#include <stdbool.h>
#include <stdint.h>

#define I2C1_SCL_GPIO_PORT GPIOB
#define I2C1_SCL_PIN GPIO_PIN_8
#define I2C1_SDA_GPIO_PORT GPIOB
#define I2C1_SDA_PIN GPIO_PIN_9

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
