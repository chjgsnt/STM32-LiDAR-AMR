#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/*
 * Application configuration module.
 *
 * This header keeps project-wide constants and compile-time settings for the
 * autonomous mobile robot firmware.
 */

#define APP_CONFIG_PROJECT_NAME "Fully Embedded 2D-LiDAR Autonomous Mobile Robot"
#define APP_CONFIG_TARGET_BOARD "STM32 NUCLEO-F446RE"
#define APP_CONFIG_STRUCTURE_STEP (0U)

void AppConfig_LoadDefaults(void);

#endif /* APP_CONFIG_H */
