#include "bringup_log.h"

#include "cmsis_os.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

static osMutexId_t app_log_mutex = NULL;

static const osMutexAttr_t app_log_mutex_attributes = {
    .name = "appLogMutex",
};

void App_LogInit(void)
{
    if (app_log_mutex == NULL)
    {
        app_log_mutex = osMutexNew(&app_log_mutex_attributes);
    }
}

static bool App_LogShouldUseMutex(void)
{
    return (app_log_mutex != NULL) && (osKernelGetState() == osKernelRunning);
}

void App_LogWriteString(const char *text)
{
    bool locked = false;

    if (text == NULL)
    {
        return;
    }

    if (App_LogShouldUseMutex())
    {
        locked = (osMutexAcquire(app_log_mutex, APP_LOG_MUTEX_TIMEOUT_MS) == osOK);
    }

    (void)printf("%s", text);

    if (locked)
    {
        (void)osMutexRelease(app_log_mutex);
    }
}

void App_LogPrintf(const char *format, ...)
{
    char line[APP_LOG_BUFFER_SIZE];
    va_list args;
    int written;

    if (format == NULL)
    {
        return;
    }

    va_start(args, format);
    written = vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    if (written < 0)
    {
        return;
    }

    App_LogWriteString(line);
}
