#ifndef APP_SERIAL_COMMAND_H
#define APP_SERIAL_COMMAND_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_SerialCommand_Init(void);
void App_SerialCommand_Task(void);
void App_SerialCommand_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERIAL_COMMAND_H */
