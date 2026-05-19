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
#include "i2c.h"
#include "i2c_scan.h"
#include "mpu6500.h"
#if APP_ENABLE_MOTOR_TEST && !APP_ENABLE_MOTOR_GPIO_STATIC_TEST
#include "motor_driver.h"
#include "tim.h"
#endif

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
#define MOTOR_TEST_DUTY 300
#define MOTOR_TEST_PRE_PHASE_STOP_MS 1000U
#define MOTOR_TEST_INITIAL_STOP_MS 2000U
#define MOTOR_TEST_RUN_MS 4000U
#define MOTOR_TEST_LOG_PERIOD_MS 500U
#define MOTOR_GPIO_STATIC_HOLD_MS 3000U
#define I2C_BASELINE_SCL_GPIO_PORT GPIOB
#define I2C_BASELINE_SCL_PIN GPIO_PIN_8
#define I2C_BASELINE_SDA_GPIO_PORT GPIOB
#define I2C_BASELINE_SDA_PIN GPIO_PIN_9

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
static void App_LogI2cBaseline(void);
#if APP_ENABLE_I2C_BUS_RECOVERY
static void App_RecoverI2cBus(void);
static void App_InitI2cRecoveryGpio(void);
static GPIO_PinState App_ReadI2cScl(void);
static GPIO_PinState App_ReadI2cSda(void);
#endif
#if APP_ENABLE_MOTOR_TEST && !APP_ENABLE_MOTOR_GPIO_STATIC_TEST
static void App_RunMotorBringupTest(void);
static void App_RunMotorPhase(const char *phase, int16_t duty, uint8_t motor_index);
static void App_LogMotorPwmStatus(const char *phase, int16_t duty);
static uint32_t App_GetMaxTim3Ccr(void);
#endif
#if APP_ENABLE_MOTOR_GPIO_STATIC_TEST
static void App_RunMotorGpioStaticTest(void);
static void App_InitMotorGpioStaticPins(void);
static void App_SetMotorGpioStaticPins(GPIO_PinState pa6_state,
                                       GPIO_PinState pa7_state,
                                       GPIO_PinState pb0_state,
                                       GPIO_PinState pb1_state);
static void App_LogMotorGpioStaticState(GPIO_PinState pa6_state,
                                        GPIO_PinState pa7_state,
                                        GPIO_PinState pb0_state,
                                        GPIO_PinState pb1_state);
#endif

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
  App_LogI2cBaseline();
#if APP_ENABLE_I2C_BUS_RECOVERY
  App_RecoverI2cBus();
#endif
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
#if APP_ENABLE_MOTOR_GPIO_STATIC_TEST
  App_RunMotorGpioStaticTest();
#elif APP_ENABLE_MOTOR_TEST
  App_RunMotorBringupTest();
#endif

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
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

static void App_LogI2cBaseline(void)
{
  printf("[I2C_BASELINE] debug start\r\n");
  printf("[I2C_BASELINE] PB8/SCL level=%u\r\n",
         (unsigned int)HAL_GPIO_ReadPin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN));
  printf("[I2C_BASELINE] PB9/SDA level=%u\r\n",
         (unsigned int)HAL_GPIO_ReadPin(I2C_BASELINE_SDA_GPIO_PORT, I2C_BASELINE_SDA_PIN));
  printf("[I2C_BASELINE] hi2c1.State=0x%02X\r\n", (unsigned int)hi2c1.State);
  printf("[I2C_BASELINE] hi2c1.ErrorCode=0x%08lX\r\n", (unsigned long)hi2c1.ErrorCode);
}

#if APP_ENABLE_I2C_BUS_RECOVERY
static void App_RecoverI2cBus(void)
{
  GPIO_PinState before_scl = App_ReadI2cScl();
  GPIO_PinState before_sda = App_ReadI2cSda();

  printf("[I2C_RECOVERY] before SCL=%u SDA=%u\r\n",
         (unsigned int)before_scl,
         (unsigned int)before_sda);

  (void)HAL_I2C_DeInit(&hi2c1);
  App_InitI2cRecoveryGpio();

  HAL_GPIO_WritePin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C_BASELINE_SDA_GPIO_PORT, I2C_BASELINE_SDA_PIN, GPIO_PIN_SET);
  osDelay(1);

  if (App_ReadI2cSda() == GPIO_PIN_RESET)
  {
    for (uint8_t i = 0U; i < 9U; i++)
    {
      HAL_GPIO_WritePin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN, GPIO_PIN_RESET);
      osDelay(1);
      HAL_GPIO_WritePin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN, GPIO_PIN_SET);
      osDelay(1);
    }
  }

  HAL_GPIO_WritePin(I2C_BASELINE_SDA_GPIO_PORT, I2C_BASELINE_SDA_PIN, GPIO_PIN_RESET);
  osDelay(1);
  HAL_GPIO_WritePin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(I2C_BASELINE_SDA_GPIO_PORT, I2C_BASELINE_SDA_PIN, GPIO_PIN_SET);
  osDelay(1);

  printf("[I2C_RECOVERY] after SCL=%u SDA=%u\r\n",
         (unsigned int)App_ReadI2cScl(),
         (unsigned int)App_ReadI2cSda());

  MX_I2C1_Init();
}

static void App_InitI2cRecoveryGpio(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = I2C_BASELINE_SCL_PIN | I2C_BASELINE_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static GPIO_PinState App_ReadI2cScl(void)
{
  return HAL_GPIO_ReadPin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN);
}

static GPIO_PinState App_ReadI2cSda(void)
{
  return HAL_GPIO_ReadPin(I2C_BASELINE_SDA_GPIO_PORT, I2C_BASELINE_SDA_PIN);
}
#endif

#if APP_ENABLE_MOTOR_TEST && !APP_ENABLE_MOTOR_GPIO_STATIC_TEST
static void App_RunMotorBringupTest(void)
{
  printf("[MOTOR] PWM test start\r\n");

  MotorDriver_Init();

  App_RunMotorPhase("A forward", MOTOR_TEST_DUTY, 0U);
  App_RunMotorPhase("A reverse", -MOTOR_TEST_DUTY, 0U);
  App_RunMotorPhase("B forward", MOTOR_TEST_DUTY, 1U);
  App_RunMotorPhase("B reverse", -MOTOR_TEST_DUTY, 1U);

  MotorDriver_StopAll();

  printf("[MOTOR] PWM test done\r\n");
}

static void App_RunMotorPhase(const char *phase, int16_t duty, uint8_t motor_index)
{
  MotorDriver_StopAll();
  osDelay(MOTOR_TEST_PRE_PHASE_STOP_MS);
  MotorDriver_ResetEncoders();

  if (motor_index == 0U)
  {
    MotorDriver_SetMotorA(duty);
  }
  else
  {
    MotorDriver_SetMotorB(duty);
  }

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < MOTOR_TEST_RUN_MS;
       elapsed_ms += MOTOR_TEST_LOG_PERIOD_MS)
  {
    osDelay(MOTOR_TEST_LOG_PERIOD_MS);
    App_LogMotorPwmStatus(phase, duty);
  }
}

static void App_LogMotorPwmStatus(const char *phase, int16_t duty)
{
  uint32_t period_counts = TIM3->ARR + 1U;
  uint32_t pwm_percent = 0U;

  if (period_counts > 0U)
  {
    pwm_percent = (App_GetMaxTim3Ccr() * 100U) / period_counts;
  }

  printf("[MOTOR] %s duty=%d ARR=%lu CCR1=%lu CCR2=%lu CCR3=%lu CCR4=%lu pwm=%lu%% encA=%ld encB=%ld\r\n",
         phase,
         (int)duty,
         (unsigned long)TIM3->ARR,
         (unsigned long)TIM3->CCR1,
         (unsigned long)TIM3->CCR2,
         (unsigned long)TIM3->CCR3,
         (unsigned long)TIM3->CCR4,
         (unsigned long)pwm_percent,
         (long)MotorDriver_GetEncoderA(),
         (long)MotorDriver_GetEncoderB());
}

static uint32_t App_GetMaxTim3Ccr(void)
{
  uint32_t max_ccr = TIM3->CCR1;

  if (TIM3->CCR2 > max_ccr)
  {
    max_ccr = TIM3->CCR2;
  }

  if (TIM3->CCR3 > max_ccr)
  {
    max_ccr = TIM3->CCR3;
  }

  if (TIM3->CCR4 > max_ccr)
  {
    max_ccr = TIM3->CCR4;
  }

  return max_ccr;
}
#endif

#if APP_ENABLE_MOTOR_GPIO_STATIC_TEST
static void App_RunMotorGpioStaticTest(void)
{
  printf("[MOTOR_GPIO] static test start\r\n");

  App_InitMotorGpioStaticPins();

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  printf("[MOTOR_GPIO] all low\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  osDelay(MOTOR_TEST_INITIAL_STOP_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  printf("[MOTOR_GPIO] PA6 high\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  printf("[MOTOR_GPIO] PA7 high\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  printf("[MOTOR_GPIO] PB0 high\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
  printf("[MOTOR_GPIO] PB1 high\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  printf("[MOTOR_GPIO] AIN1 and BIN1 high\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);
  printf("[MOTOR_GPIO] AIN2 and BIN2 high\r\n");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  printf("[MOTOR_GPIO] static test done\r\n");
}

static void App_InitMotorGpioStaticPins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void App_SetMotorGpioStaticPins(GPIO_PinState pa6_state,
                                       GPIO_PinState pa7_state,
                                       GPIO_PinState pb0_state,
                                       GPIO_PinState pb1_state)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, pa6_state);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, pa7_state);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, pb0_state);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, pb1_state);
}

static void App_LogMotorGpioStaticState(GPIO_PinState pa6_state,
                                        GPIO_PinState pa7_state,
                                        GPIO_PinState pb0_state,
                                        GPIO_PinState pb1_state)
{
  printf("[MOTOR_GPIO] PA6=%u PA7=%u PB0=%u PB1=%u\r\n",
         (pa6_state == GPIO_PIN_SET) ? 1U : 0U,
         (pa7_state == GPIO_PIN_SET) ? 1U : 0U,
         (pb0_state == GPIO_PIN_SET) ? 1U : 0U,
         (pb1_state == GPIO_PIN_SET) ? 1U : 0U);
}
#endif

/* USER CODE END Application */

