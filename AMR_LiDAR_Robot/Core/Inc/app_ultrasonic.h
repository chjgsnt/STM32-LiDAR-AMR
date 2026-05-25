#ifndef APP_ULTRASONIC_H
#define APP_ULTRASONIC_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default demo wiring:
 *   HC-SR04 TRIG: PC0
 *   HC-SR04 ECHO: PC1
 *
 * Confirm these pins in CubeMX before regenerating code. If your ultrasonic
 * module is wired elsewhere, update only these macros.
 */
#ifndef APP_ULTRASONIC_TRIG_GPIO_PORT
#define APP_ULTRASONIC_TRIG_GPIO_PORT GPIOC
#endif

#ifndef APP_ULTRASONIC_TRIG_GPIO_PIN
#define APP_ULTRASONIC_TRIG_GPIO_PIN GPIO_PIN_0
#endif

#ifndef APP_ULTRASONIC_TRIG_GPIO_CLK_ENABLE
#define APP_ULTRASONIC_TRIG_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#endif

#ifndef APP_ULTRASONIC_TRIG_PIN_NAME
#define APP_ULTRASONIC_TRIG_PIN_NAME "PC0"
#endif

#ifndef APP_ULTRASONIC_ECHO_GPIO_PORT
#define APP_ULTRASONIC_ECHO_GPIO_PORT GPIOC
#endif

#ifndef APP_ULTRASONIC_ECHO_GPIO_PIN
#define APP_ULTRASONIC_ECHO_GPIO_PIN GPIO_PIN_1
#endif

#ifndef APP_ULTRASONIC_ECHO_GPIO_CLK_ENABLE
#define APP_ULTRASONIC_ECHO_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#endif

#ifndef APP_ULTRASONIC_ECHO_PIN_NAME
#define APP_ULTRASONIC_ECHO_PIN_NAME "PC1"
#endif

#ifndef APP_ULTRASONIC_MAX_DISTANCE_CM
#define APP_ULTRASONIC_MAX_DISTANCE_CM 400U
#endif

#ifndef APP_ULTRASONIC_NO_ECHO_AS_MAX_CM
#define APP_ULTRASONIC_NO_ECHO_AS_MAX_CM 1U
#endif

#define APP_ULTRASONIC_INVALID_CM 0xFFFFU

void App_Ultrasonic_Init(void);
uint8_t App_Ultrasonic_ReadDistanceCm(uint16_t *distance_cm);

#ifdef __cplusplus
}
#endif

#endif /* APP_ULTRASONIC_H */
