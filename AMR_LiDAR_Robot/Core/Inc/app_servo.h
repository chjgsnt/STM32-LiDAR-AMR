#ifndef APP_SERVO_H
#define APP_SERVO_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default demo wiring:
 *   Servo signal: PA8 / TIM1_CH1 / AF1
 *
 * Confirm this pin in CubeMX before regenerating code. If your servo signal is
 * wired elsewhere, update these macros and the timer/channel setup together.
 */
#ifndef APP_SERVO_PWM_GPIO_PORT
#define APP_SERVO_PWM_GPIO_PORT GPIOA
#endif

#ifndef APP_SERVO_PWM_GPIO_PIN
#define APP_SERVO_PWM_GPIO_PIN GPIO_PIN_8
#endif

#ifndef APP_SERVO_PWM_GPIO_AF
#define APP_SERVO_PWM_GPIO_AF GPIO_AF1_TIM1
#endif

#ifndef APP_SERVO_PWM_GPIO_CLK_ENABLE
#define APP_SERVO_PWM_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#endif

#define APP_SERVO_PWM_TIMER_NAME "TIM1_CH1_PA8"

#ifndef APP_SERVO_PERIOD_US
#define APP_SERVO_PERIOD_US 20000U
#endif

#ifndef APP_SERVO_MIN_US
#define APP_SERVO_MIN_US 500U
#endif

#ifndef APP_SERVO_MAX_US
#define APP_SERVO_MAX_US 2500U
#endif

#ifndef APP_SERVO_CENTER_US
#define APP_SERVO_CENTER_US 1500U
#endif

#ifndef APP_SERVO_LEFT_US
#define APP_SERVO_LEFT_US 2100U
#endif

#ifndef APP_SERVO_RIGHT_US
#define APP_SERVO_RIGHT_US 900U
#endif

void App_Servo_Init(void);
void App_Servo_Center(void);
void App_Servo_Left(void);
void App_Servo_Right(void);
void App_Servo_SetPulseUs(uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVO_H */
