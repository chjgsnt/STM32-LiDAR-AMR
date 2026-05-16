#include "debug_log.h"

void DebugLog_Init(void)
{
    /* TODO: Attach debug logging to a UART after CubeMX setup. */
}

void DebugLog_Print(const char *message)
{
    (void)message;
    /* TODO: Send a null-terminated string to the debug UART. */
}

void DebugLog_PrintBuffer(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    /* TODO: Dump binary data for debugging and evidence recording. */
}
