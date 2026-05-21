#ifndef APP_LIDAR_H
#define APP_LIDAR_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_Lidar_Init(void);
void App_Lidar_Task(void);
void App_Lidar_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_LIDAR_H */
