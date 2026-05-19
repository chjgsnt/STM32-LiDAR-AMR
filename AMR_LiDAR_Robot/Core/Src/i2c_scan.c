#include "i2c_scan.h"

#include "bringup_log.h"

#include "i2c.h"

#include <stdbool.h>
#include <stdint.h>

void I2C_ScanBus(void)
{
    uint8_t found_count = 0U;
    bool oled_found = false;
    LOG_INFO("I2C scan started.");

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
            LOG_INFO("I2C bus busy at 0x%02X.", addr);
        }
        else if (status == HAL_TIMEOUT)
        {
            LOG_INFO("I2C timeout at 0x%02X.", addr);
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
