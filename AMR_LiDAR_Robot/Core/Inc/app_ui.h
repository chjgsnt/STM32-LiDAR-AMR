#ifndef APP_UI_H
#define APP_UI_H

#include <stdint.h>

#ifndef APP_TELEMETRY_AUTO_ENABLE
#define APP_TELEMETRY_AUTO_ENABLE 0
#endif

#ifndef APP_TELEMETRY_PERIOD_MS
#define APP_TELEMETRY_PERIOD_MS 1000U
#endif

#ifndef APP_UI_OLED_ENABLE
#define APP_UI_OLED_ENABLE 0
#endif

#ifndef APP_UI_SERIAL_FALLBACK_ENABLE
#define APP_UI_SERIAL_FALLBACK_ENABLE 1
#endif

#ifndef APP_UI_SERIAL_AUTO_PRINT_ENABLE
#define APP_UI_SERIAL_AUTO_PRINT_ENABLE 0
#endif

#ifndef APP_UI_SERIAL_PRINT_PERIOD_MS
#define APP_UI_SERIAL_PRINT_PERIOD_MS 2000U
#endif

#ifdef __cplusplus
extern "C" {
#endif

void App_UI_Init(void);
void App_UI_Update(void);

void AppUI_Init(void);
void AppUI_Update(void);
void AppUI_SetPage(uint8_t page);
void AppUI_NextPage(void);
void AppUI_PrintStatus(void);
uint16_t AppUI_GetSpeedLimit(void);
uint16_t AppUI_GetObstacleThreshold(void);
uint16_t AppUI_GetWallThreshold(void);
uint8_t AppUI_GetPage(void);
uint8_t AppUI_IsOledReady(void);
void AppTelemetry_Print(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UI_H */
