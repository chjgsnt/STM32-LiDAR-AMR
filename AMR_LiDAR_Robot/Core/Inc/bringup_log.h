#ifndef BRINGUP_LOG_H
#define BRINGUP_LOG_H

#include "app_config.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>

void App_LogInit(void);
void App_LogPrintf(const char *format, ...);
void App_LogWriteString(const char *text);

/*
 * Minimal timestamped log macro for bring-up.
 *
 * Output format:
 * [000123 ms] Message
 */
#define APP_LOG(format, ...) \
    App_LogPrintf(format "\r\n", ##__VA_ARGS__)

#define APP_LOG_RAW(text) \
    App_LogWriteString(text)

#define LOG_INFO(format, ...) \
    APP_LOG("[%06lu ms] " format, (unsigned long)HAL_GetTick(), ##__VA_ARGS__)

#endif /* BRINGUP_LOG_H */
