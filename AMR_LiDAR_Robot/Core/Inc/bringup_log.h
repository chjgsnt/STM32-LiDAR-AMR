#ifndef BRINGUP_LOG_H
#define BRINGUP_LOG_H

#include "stm32f4xx_hal.h"

#include <stdio.h>

/*
 * Minimal timestamped log macro for bring-up.
 *
 * Output format:
 * [000123 ms] Message
 */
#define LOG_INFO(format, ...) \
    printf("[%06lu ms] " format "\r\n", (unsigned long)HAL_GetTick(), ##__VA_ARGS__)

#endif /* BRINGUP_LOG_H */
