#ifndef APP_BUTTON_CONTROL_H
#define APP_BUTTON_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_ButtonControl_Init(void);
void App_ButtonControl_Update(void);
uint8_t App_ButtonControl_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUTTON_CONTROL_H */
