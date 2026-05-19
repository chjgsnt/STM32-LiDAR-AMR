#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define SSD1306_I2C_ADDR_7BIT 0x3CU

bool SSD1306_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
bool SSD1306_Clear(void);
bool SSD1306_WriteString(uint8_t x, uint8_t page, const char *text);
bool SSD1306_UpdateScreen(void);

#endif /* SSD1306_H */
