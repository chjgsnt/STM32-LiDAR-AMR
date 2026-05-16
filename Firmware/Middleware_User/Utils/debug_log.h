#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>

/*
 * UART debug logging module.
 *
 * This module will provide lightweight serial logging helpers for runtime
 * diagnostics, sensor dumps, and test evidence collection.
 */

void DebugLog_Init(void);
void DebugLog_Print(const char *message);
void DebugLog_PrintBuffer(const uint8_t *data, uint16_t length);

#endif /* DEBUG_LOG_H */
