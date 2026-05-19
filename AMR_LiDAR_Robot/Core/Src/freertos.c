/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bringup_log.h"
#include "i2c_scan.h"
#include "mpu6500.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef LD2_Pin
#define BRINGUP_LED_PIN GPIO_PIN_5
#else
#define BRINGUP_LED_PIN LD2_Pin
#endif

#ifndef LD2_GPIO_Port
#define BRINGUP_LED_GPIO_PORT GPIOA
#else
#define BRINGUP_LED_GPIO_PORT LD2_GPIO_Port
#endif

#define IMU_SAMPLE_PERIOD_MS 10U
#define APP_IMU_LOG_INTERVAL_MS 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lidarTask */
osThreadId_t lidarTaskHandle;
const osThreadAttr_t lidarTask_attributes = {
  .name = "lidarTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static int32_t App_ScaleFloatRounded(float value, float multiplier);
static const char *App_FixedSign(int32_t value);
static uint32_t App_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t App_FixedFraction(int32_t value, int32_t decimal_scale);
static void App_LogImuStatus(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of lidarTask */
  lidarTaskHandle = osThreadNew(StartTask02, NULL, &lidarTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartTask03, NULL, &imuTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(StartTask04, NULL, &oledTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartTask05, NULL, &controlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  LOG_INFO("FreeRTOS started.");
  I2C_ScanBus();
  I2C_ReadMpu6500WhoAmI();
  I2C_WakeMpu6500();
  I2C_CalibrateMpu6500Gyro();

  /* Infinite loop */
  uint8_t led_tick = 0U;
  for(;;)
  {
    I2C_ReadMpu6500Raw();
    App_LogImuStatus();

    led_tick++;
    if (led_tick >= (1000U / IMU_SAMPLE_PERIOD_MS))
    {
      HAL_GPIO_TogglePin(BRINGUP_LED_GPIO_PORT, BRINGUP_LED_PIN);
      led_tick = 0U;
    }

    osDelay(IMU_SAMPLE_PERIOD_MS);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the lidarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static int32_t App_ScaleFloatRounded(float value, float multiplier)
{
  float scaled = value * multiplier;

  if (scaled < 0.0f)
  {
    return (int32_t)(scaled - 0.5f);
  }

  return (int32_t)(scaled + 0.5f);
}

static const char *App_FixedSign(int32_t value)
{
  return (value < 0) ? "-" : "";
}

static uint32_t App_FixedWhole(int32_t value, int32_t decimal_scale)
{
  int32_t abs_value = (value < 0) ? -value : value;
  return (uint32_t)(abs_value / decimal_scale);
}

static uint32_t App_FixedFraction(int32_t value, int32_t decimal_scale)
{
  int32_t abs_value = (value < 0) ? -value : value;
  return (uint32_t)(abs_value % decimal_scale);
}

static void App_LogImuStatus(void)
{
  static uint32_t last_log_ms = 0U;
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_log_ms) < APP_IMU_LOG_INTERVAL_MS)
  {
    return;
  }

  last_log_ms = now_ms;

  MPU6500_Data_t imu;

  if (MPU6500_GetLatest(&imu) == false)
  {
    LOG_INFO("APP IMU: ready=0");
    return;
  }

  int32_t pitch_tenth_deg = App_ScaleFloatRounded(imu.fused_pitch_deg, 10.0f);
  int32_t roll_tenth_deg = App_ScaleFloatRounded(imu.fused_roll_deg, 10.0f);

  LOG_INFO("APP IMU: ready=%u pitch=%s%lu.%01ludeg roll=%s%lu.%01ludeg",
           (unsigned int)imu.is_ready,
           App_FixedSign(pitch_tenth_deg),
           (unsigned long)App_FixedWhole(pitch_tenth_deg, 10),
           (unsigned long)App_FixedFraction(pitch_tenth_deg, 10),
           App_FixedSign(roll_tenth_deg),
           (unsigned long)App_FixedWhole(roll_tenth_deg, 10),
           (unsigned long)App_FixedFraction(roll_tenth_deg, 10));
}

/* USER CODE END Application */

