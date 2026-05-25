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
#if APP_ENABLE_ENCODER_BRINGUP_TEST
#include "app_encoder_bringup.h"
#endif
#include "app_lidar.h"
#if APP_ENABLE_LIDAR_OBSTACLE_AVOIDANCE_TEST
#include "app_lidar_obstacle_avoidance.h"
#endif
#include "app_lidar_stop_check.h"
#if APP_ENABLE_MOTOR_FORCED_SPIN_CHECK_TEST
#include "app_motor_forced_spin_check.h"
#endif
#if APP_ENABLE_NO_SERVO_OBSTACLE_TEST
#include "app_no_servo_obstacle.h"
#endif
#if APP_ENABLE_SERVO_SCAN_OBSTACLE_TEST
#include "app_servo_scan_obstacle.h"
#endif
#if !APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE
#include "app_obstacle.h"
#include "app_obstacle_motor.h"
#endif
#include "bringup_log.h"
#include "i2c.h"
#include "i2c_scan.h"
#include "mpu6500.h"
#if APP_ENABLE_CHASSIS_OPENLOOP_TEST || APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST || APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST || APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST || APP_ENABLE_WHEEL_SPEED_PI_TEST || APP_ENABLE_HEADING_HOLD_TEST
#include "chassis.h"
#endif
#if (APP_ENABLE_MOTOR_TEST && !APP_ENABLE_MOTOR_GPIO_STATIC_TEST) || APP_ENABLE_CHASSIS_OPENLOOP_TEST || APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST || APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST || APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST || APP_ENABLE_WHEEL_SPEED_PI_TEST || APP_ENABLE_HEADING_HOLD_TEST
#include "motor_driver.h"
#endif
#if APP_ENABLE_MOTOR_TEST && !APP_ENABLE_MOTOR_GPIO_STATIC_TEST
#include "tim.h"
#endif
#if APP_ENABLE_WHEEL_SPEED_PI_TEST || APP_ENABLE_HEADING_HOLD_TEST
#include "wheel_speed_controller.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if APP_ENABLE_CHASSIS_OPENLOOP_TEST
typedef void (*App_ChassisCommand_t)(int16_t duty);
#endif

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
/*
 * 300 is suitable for lifted-wheel direction verification.
 * 500 is selected for ground open-loop traction test.
 */
#define CHASSIS_TEST_DUTY CHASSIS_GROUND_TEST_DUTY
#define CHASSIS_TEST_INITIAL_STOP_MS 2000U
#define CHASSIS_TEST_BETWEEN_STOP_MS 1000U
#define CHASSIS_TEST_RUN_MS 3000U
#define CHASSIS_TEST_LOG_PERIOD_MS 500U
#define CHASSIS_CAL_DUTY 300
#define CHASSIS_CAL_INITIAL_STOP_MS 2000U
#define CHASSIS_CAL_BETWEEN_STOP_MS 1000U
#define CHASSIS_CAL_RUN_MS 3000U
#define CHASSIS_CAL_LOG_PERIOD_MS 500U
#define CHASSIS_GROUND_INITIAL_STOP_MS 2000U
#define CHASSIS_GROUND_BETWEEN_STOP_MS 1000U
#define CHASSIS_GROUND_RUN_MS 1500U
#define CHASSIS_GROUND_LOG_PERIOD_MS 500U
#define CHASSIS_BALANCE_BASE_DUTY CHASSIS_GROUND_TEST_DUTY
#define CHASSIS_BALANCE_INITIAL_STOP_MS 2000U
#define CHASSIS_BALANCE_BETWEEN_STOP_MS 2000U
#define CHASSIS_BALANCE_RUN_MS 5000U
#define CHASSIS_BALANCE_LOG_PERIOD_MS 200U
#define CHASSIS_BALANCE_KP 1
#ifndef CHASSIS_LEFT_TRIM
#define CHASSIS_LEFT_TRIM (-10)
#endif
#ifndef CHASSIS_RIGHT_TRIM
#define CHASSIS_RIGHT_TRIM 10
#endif
#define CHASSIS_BALANCE_CORRECTION_LIMIT 100
#define CHASSIS_BALANCE_DUTY_MIN 300
#define CHASSIS_BALANCE_DUTY_MAX CHASSIS_MAX_OPENLOOP_DUTY
#define WHEEL_SPEED_PI_INITIAL_STOP_MS 2000U
#define WHEEL_SPEED_PI_BETWEEN_STOP_MS 2000U
#define WHEEL_SPEED_PI_RUN_MS 6000U
#define WHEEL_SPEED_PI_SAMPLE_PERIOD_MS 200U
#define WHEEL_SPEED_PI_TARGET_TICKS_PER_SAMPLE 800
#define WHEEL_SPEED_PI_FEEDFORWARD_DUTY CHASSIS_GROUND_TEST_DUTY
#define WHEEL_SPEED_PI_KP_NUM 100
#define WHEEL_SPEED_PI_KI_NUM 5
#define WHEEL_SPEED_PI_GAIN_DEN 100
#define WHEEL_SPEED_PI_INTEGRAL_LIMIT 2000
#define WHEEL_SPEED_PI_DUTY_MIN 300
#define WHEEL_SPEED_PI_DUTY_MAX CHASSIS_MAX_OPENLOOP_DUTY
#define HEADING_HOLD_INITIAL_STOP_MS 2000U
#define HEADING_HOLD_BETWEEN_STOP_MS 2000U
#define HEADING_HOLD_RUN_MS 6000U
#define HEADING_HOLD_SAMPLE_PERIOD_MS WHEEL_SPEED_PI_SAMPLE_PERIOD_MS
#define HEADING_HOLD_BASE_TARGET_TICKS_PER_SAMPLE WHEEL_SPEED_PI_TARGET_TICKS_PER_SAMPLE
#define HEADING_K_FORWARD 2
#define HEADING_K_BACKWARD 2
#define HEADING_CORR_LIMIT_FORWARD 50
#define HEADING_CORR_LIMIT_BACKWARD 50
#define HEADING_HOLD_TARGET_MIN_TICKS 600
#define HEADING_HOLD_TARGET_MAX_TICKS 900
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
#if !APP_ENABLE_MOTOR_FORCED_SPIN_CHECK_TEST
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
#endif
static const char *App_GetActiveModeName(void);
static const char *App_GetActiveTestName(void);
#if APP_ENABLE_MOTOR_TEST && !APP_ENABLE_MOTOR_GPIO_STATIC_TEST
static void App_RunMotorBringupTest(void);
static void App_RunMotorPhase(const char *phase, int16_t duty, uint8_t motor_index);
static void App_LogMotorPwmStatus(const char *phase, int16_t duty);
static uint32_t App_GetMaxTim3Ccr(void);
#endif
#if APP_ENABLE_CHASSIS_OPENLOOP_TEST
static void App_RunChassisOpenLoopTest(void);
static void App_RunChassisPhase(const char *action,
                                int16_t duty,
                                int16_t left_duty,
                                int16_t right_duty,
                                App_ChassisCommand_t command);
static void App_LogChassisStatus(const char *action, int16_t left_duty, int16_t right_duty);
#endif
#if APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST
static void App_RunChassisDirectionCalTest(void);
static void App_RunChassisDirectionCalPhase(const char *action, int16_t left_duty, int16_t right_duty);
static void App_LogChassisDirectionCalStatus(const char *action, int16_t left_duty, int16_t right_duty);
#endif
#if APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST
static void App_RunChassisGroundTractionTest(void);
static void App_RunChassisGroundTractionPhase(int16_t duty);
static void App_LogChassisGroundTractionStatus(int16_t duty,
                                               int32_t enc_a,
                                               int32_t enc_b,
                                               int32_t delta_a,
                                               int32_t delta_b);
#endif
#if APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST
static void App_RunChassisSpeedBalanceTest(void);
static void App_RunChassisSpeedBalancePhase(const char *direction_name,
                                            int8_t direction,
                                            int16_t base_duty);
static int32_t App_AbsI32(int32_t value);
static int16_t App_LimitInt16(int32_t value, int16_t min_value, int16_t max_value);
static void App_LogChassisSpeedBalanceStatus(const char *direction_name,
                                             int16_t base_duty,
                                             int16_t left_duty,
                                             int16_t right_duty,
                                             int32_t enc_a,
                                             int32_t enc_b,
                                             int32_t delta_a,
                                             int32_t delta_b,
                                             int32_t err);
#endif
#if APP_ENABLE_WHEEL_SPEED_PI_TEST
static void App_RunWheelSpeedPiTest(void);
static void App_RunWheelSpeedPiPhase(const char *direction_name,
                                     int8_t direction,
                                     const WheelSpeedController_Config_t *config);
static void App_LogWheelSpeedPiStatus(const char *direction_name,
                                      int32_t target_ticks_per_sample,
                                      int16_t left_duty,
                                      int16_t right_duty,
                                      int32_t enc_a,
                                      int32_t enc_b,
                                      int32_t delta_a,
                                      int32_t delta_b,
                                      const WheelSpeedController_State_t *left_state,
                                      const WheelSpeedController_State_t *right_state);
#endif
#if APP_ENABLE_HEADING_HOLD_TEST
static void App_RunHeadingHoldTest(void);
static void App_RunHeadingHoldPhase(const char *direction_name,
                                    int8_t direction,
                                    float target_yaw_deg,
                                    float *yaw_deg,
                                    uint32_t *last_imu_update_ms,
                                    const WheelSpeedController_Config_t *base_config);
static float App_UpdateHeadingYawDeg(float yaw_deg, uint32_t *last_imu_update_ms);
static int32_t App_LimitI32(int32_t value, int32_t min_value, int32_t max_value);
static void App_LogHeadingHoldStatus(const char *direction_name,
                                     float target_yaw_deg,
                                     float yaw_deg,
                                     float yaw_error_deg,
                                     int32_t k_heading,
                                     int32_t correction_limit_ticks,
                                     int32_t heading_correction_ticks,
                                     int32_t left_target_ticks,
                                     int32_t right_target_ticks,
                                     int32_t delta_a,
                                     int32_t delta_b,
                                     int16_t left_duty,
                                     int16_t right_duty);
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
  App_LogInit();
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
#if APP_ENABLE_MOTOR_FORCED_SPIN_CHECK_TEST
  APP_LOG("[APP] MotorForcedSpinCheck: default task idle, skipping I2C/IMU bring-up");

  for(;;)
  {
    HAL_GPIO_TogglePin(BRINGUP_LED_GPIO_PORT, BRINGUP_LED_PIN);
    osDelay(1000);
  }
#else
#if APP_ENABLE_I2C_BASELINE_DEBUG
  App_LogI2cBaseline();
#endif
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
#endif
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
#if APP_ENABLE_LIDAR_BRINGUP_TEST
  App_Lidar_Init();
#if APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE
  App_LidarStopCheck_Init();
#elif APP_ENABLE_LIDAR_OBSTACLE_AVOIDANCE_TEST
  APP_LOG("[APP] LiDARObstacleAvoidance: lidar parser task active");
#else
  App_Obstacle_Init();
  App_ObstacleMotor_Init();
#endif
#endif

  /* Infinite loop */
  for(;;)
  {
#if APP_ENABLE_LIDAR_BRINGUP_TEST
    App_Lidar_Task();
#if APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE
    App_LidarStopCheck_Task();
#elif APP_ENABLE_LIDAR_OBSTACLE_AVOIDANCE_TEST
    /* LiDARObstacleAvoidance consumes the published LiDAR status in controlTask. */
#else
#if !APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
    App_Obstacle_Task();
#endif
    App_ObstacleMotor_Task();
#endif
    osDelay(50);
#else
    osDelay(1000);
#endif
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
  APP_LOG("[APP] active_mode=%s", App_GetActiveModeName());
  APP_LOG("[APP] active_test=%s", App_GetActiveTestName());

#if APP_ENABLE_MOTOR_FORCED_SPIN_CHECK_TEST
  App_MotorForcedSpinCheck_Run();
#elif APP_ENABLE_MOTOR_GPIO_STATIC_TEST
  App_RunMotorGpioStaticTest();
#elif APP_ENABLE_MOTOR_TEST
  App_RunMotorBringupTest();
#elif APP_ENABLE_CHASSIS_OPENLOOP_TEST
  App_RunChassisOpenLoopTest();
#elif APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST
  App_RunChassisDirectionCalTest();
#elif APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST
  App_RunChassisGroundTractionTest();
#elif APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST
  App_RunChassisSpeedBalanceTest();
#elif APP_ENABLE_WHEEL_SPEED_PI_TEST
  App_RunWheelSpeedPiTest();
#elif APP_ENABLE_HEADING_HOLD_TEST
  App_RunHeadingHoldTest();
#elif APP_ENABLE_ENCODER_BRINGUP_TEST
  APP_LOG("[APP] EncoderBringup: encoder count/delta logging enabled");
  App_EncoderBringup_Init();
  App_EncoderBringup_Task(argument);
#elif APP_ENABLE_LIDAR_OBSTACLE_AVOIDANCE_TEST
  APP_LOG("[APP] lidar obstacle avoidance enabled");
  App_LidarObstacleAvoidance_Init();
  for(;;)
  {
    App_LidarObstacleAvoidance_Task();
    osDelay(50);
  }
#elif APP_ENABLE_NO_SERVO_OBSTACLE_TEST
  APP_LOG("[APP] NoServoObstacleAvoidance: ultrasonic-only backup/turn obstacle avoidance enabled");
  App_NoServoObstacle_Init();
  for(;;)
  {
    App_NoServoObstacle_Task();
    osDelay(50);
  }
#elif APP_ENABLE_SERVO_SCAN_OBSTACLE_TEST
  APP_LOG("[APP] ServoScanObstacle: ultrasonic servo scan obstacle avoidance enabled");
  App_ServoScanObstacle_Init();
  for(;;)
  {
    App_ServoScanObstacle_Task();
    osDelay(50);
  }
#elif APP_TEST_MODE_ENABLE_SENSOR_BRINGUP
  APP_LOG("[APP] sensor bring-up safe default: no motor output, no servo PWM, no ultrasonic obstacle task");
#elif APP_ENABLE_LIDAR_BRINGUP_TEST
#if APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE
  APP_LOG("[APP] LiDAR obstacle stop check: front-only forward/stop, obstacle state machine disabled");
#elif APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
#if APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE
  APP_LOG("[APP] IMU heading assist lifted-wheel-test: fixed FORWARD_SLOW, motor output enabled, ground=0");
#elif APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ACTIVE
  APP_LOG("[APP] IMU heading assist lifted-wheel-test: fixed FORWARD_SLOW, apply=0, motor output disabled, ground=0");
#else
  APP_LOG("[APP] IMU heading assist dry-run mode: fixed FORWARD_SLOW dry-run, motor output disabled");
#endif
#elif APP_LIDAR_OBSTACLE_GROUND_TEST_ENABLE
  APP_LOG("[APP] LiDAR obstacle ground-test mode: guarded low-speed motor output enabled");
#else
  APP_LOG("[APP] LiDAR obstacle dry-run mode: motor output disabled");
#endif
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
#if !APP_ENABLE_MOTOR_FORCED_SPIN_CHECK_TEST
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
  APP_LOG("[I2C_BASELINE] debug start");
  APP_LOG("[I2C_BASELINE] PB8/SCL level=%u",
          (unsigned int)HAL_GPIO_ReadPin(I2C_BASELINE_SCL_GPIO_PORT, I2C_BASELINE_SCL_PIN));
  APP_LOG("[I2C_BASELINE] PB9/SDA level=%u",
          (unsigned int)HAL_GPIO_ReadPin(I2C_BASELINE_SDA_GPIO_PORT, I2C_BASELINE_SDA_PIN));
  APP_LOG("[I2C_BASELINE] hi2c1.State=0x%02X", (unsigned int)hi2c1.State);
  APP_LOG("[I2C_BASELINE] hi2c1.ErrorCode=0x%08lX", (unsigned long)hi2c1.ErrorCode);
}
#endif

static const char *App_GetActiveModeName(void)
{
#if APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_DRY_RUN
  return "LiDAR obstacle dry-run";
#elif APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_GROUND_TEST
  return "LiDAR obstacle ground-test";
#elif APP_ACTIVE_MODE == APP_MODE_IMU_HEADING_ASSIST_DRY_RUN
  return "IMU heading assist dry-run";
#elif APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_STOP_CHECK
  return "LidarObstacleStopCheck";
#elif APP_ACTIVE_MODE == APP_MODE_MOTOR_FORCED_SPIN_CHECK
  return "MotorForcedSpinCheck";
#elif APP_ACTIVE_MODE == APP_MODE_SERVO_SCAN_OBSTACLE
  return "ServoScanObstacle";
#elif APP_ACTIVE_MODE == APP_MODE_NO_SERVO_OBSTACLE
  return "NoServoObstacleAvoidance";
#elif APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_AVOIDANCE
  return "LiDARObstacleAvoidance";
#elif APP_ACTIVE_MODE == APP_MODE_ENCODER_BRINGUP
  return "EncoderBringup";
#elif APP_ACTIVE_MODE == APP_MODE_MOTOR_TEST
  return "motor test";
#elif APP_ACTIVE_MODE == APP_MODE_IMU_TEST
  return "IMU test";
#elif APP_ACTIVE_MODE == APP_MODE_SENSOR_BRINGUP
  return "sensor bring-up";
#else
  return "unknown";
#endif
}

static const char *App_GetActiveTestName(void)
{
#if APP_ACTIVE_TEST == APP_TEST_NONE
  return "none";
#elif APP_ACTIVE_TEST == APP_TEST_MOTOR_PWM
  return "motor PWM";
#elif APP_ACTIVE_TEST == APP_TEST_CHASSIS_OPENLOOP
  return "chassis open-loop";
#elif APP_ACTIVE_TEST == APP_TEST_CHASSIS_SPEED_BALANCE
  return "chassis speed balance";
#elif APP_ACTIVE_TEST == APP_TEST_WHEEL_SPEED_PI
  return "wheel speed PI";
#elif APP_ACTIVE_TEST == APP_TEST_HEADING_HOLD
  return "heading hold";
#elif APP_ACTIVE_TEST == APP_TEST_LIDAR_BRINGUP
  return "LiDAR bring-up";
#elif APP_ACTIVE_TEST == APP_TEST_LIDAR_STOP_CHECK
  return "LidarObstacleStopCheck";
#elif APP_ACTIVE_TEST == APP_TEST_MOTOR_FORCED_SPIN_CHECK
  return "MotorForcedSpinCheck";
#elif APP_ACTIVE_TEST == APP_TEST_SERVO_SCAN_OBSTACLE
  return "ServoScanObstacle";
#elif APP_ACTIVE_TEST == APP_TEST_NO_SERVO_OBSTACLE
  return "NoServoObstacleAvoidance";
#elif APP_ACTIVE_TEST == APP_TEST_LIDAR_OBSTACLE_AVOIDANCE
  return "LiDARObstacleAvoidance";
#elif APP_ACTIVE_TEST == APP_TEST_ENCODER_BRINGUP
  return "EncoderBringup";
#else
  return "unknown";
#endif
}

#if APP_ENABLE_I2C_BUS_RECOVERY && !APP_ENABLE_MOTOR_FORCED_SPIN_CHECK_TEST
static void App_RecoverI2cBus(void)
{
  GPIO_PinState before_scl = App_ReadI2cScl();
  GPIO_PinState before_sda = App_ReadI2cSda();

  APP_LOG("[I2C_RECOVERY] before SCL=%u SDA=%u",
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

  APP_LOG("[I2C_RECOVERY] after SCL=%u SDA=%u",
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
  APP_LOG("[MOTOR] PWM test start");

  MotorDriver_Init();

  App_RunMotorPhase("A forward", MOTOR_TEST_DUTY, 0U);
  App_RunMotorPhase("A reverse", -MOTOR_TEST_DUTY, 0U);
  App_RunMotorPhase("B forward", MOTOR_TEST_DUTY, 1U);
  App_RunMotorPhase("B reverse", -MOTOR_TEST_DUTY, 1U);

  MotorDriver_StopAll();

  APP_LOG("[MOTOR] PWM test done");
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

  APP_LOG("[MOTOR] %s duty=%d ARR=%lu CCR1=%lu CCR2=%lu CCR3=%lu CCR4=%lu pwm=%lu%% encA=%ld encB=%ld",
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

#if APP_ENABLE_CHASSIS_OPENLOOP_TEST
static void App_RunChassisOpenLoopTest(void)
{
  APP_LOG("[CHASSIS] open-loop test start");

  Chassis_Init();
  Chassis_Stop();
  osDelay(CHASSIS_TEST_INITIAL_STOP_MS);

  App_RunChassisPhase("Forward",
                      CHASSIS_TEST_DUTY,
                      CHASSIS_TEST_DUTY,
                      CHASSIS_TEST_DUTY,
                      Chassis_Forward);
  Chassis_Stop();
  osDelay(CHASSIS_TEST_BETWEEN_STOP_MS);

  App_RunChassisPhase("Backward",
                      CHASSIS_TEST_DUTY,
                      -CHASSIS_TEST_DUTY,
                      -CHASSIS_TEST_DUTY,
                      Chassis_Backward);
  Chassis_Stop();
  osDelay(CHASSIS_TEST_BETWEEN_STOP_MS);

  App_RunChassisPhase("TurnLeft",
                      CHASSIS_TEST_DUTY,
                      -CHASSIS_TEST_DUTY,
                      CHASSIS_TEST_DUTY,
                      Chassis_TurnLeft);
  Chassis_Stop();
  osDelay(CHASSIS_TEST_BETWEEN_STOP_MS);

  App_RunChassisPhase("TurnRight",
                      CHASSIS_TEST_DUTY,
                      CHASSIS_TEST_DUTY,
                      -CHASSIS_TEST_DUTY,
                      Chassis_TurnRight);
  Chassis_Stop();

  APP_LOG("[CHASSIS] open-loop test done");
}

static void App_RunChassisPhase(const char *action,
                                int16_t duty,
                                int16_t left_duty,
                                int16_t right_duty,
                                App_ChassisCommand_t command)
{
  MotorDriver_ResetEncoders();
  command(duty);

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < CHASSIS_TEST_RUN_MS;
       elapsed_ms += CHASSIS_TEST_LOG_PERIOD_MS)
  {
    osDelay(CHASSIS_TEST_LOG_PERIOD_MS);
    App_LogChassisStatus(action, left_duty, right_duty);
  }
}

static void App_LogChassisStatus(const char *action, int16_t left_duty, int16_t right_duty)
{
  APP_LOG("[CHASSIS] action=%s left_duty=%d right_duty=%d encA=%ld encB=%ld",
          action,
          (int)left_duty,
          (int)right_duty,
          (long)MotorDriver_GetEncoderA(),
          (long)MotorDriver_GetEncoderB());
}
#endif

#if APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST
static void App_RunChassisDirectionCalTest(void)
{
  Chassis_Init();
  Chassis_Stop();
  osDelay(CHASSIS_CAL_INITIAL_STOP_MS);

  App_RunChassisDirectionCalPhase("Left+300", CHASSIS_CAL_DUTY, 0);
  Chassis_Stop();
  osDelay(CHASSIS_CAL_BETWEEN_STOP_MS);

  App_RunChassisDirectionCalPhase("Left-300", -CHASSIS_CAL_DUTY, 0);
  Chassis_Stop();
  osDelay(CHASSIS_CAL_BETWEEN_STOP_MS);

  App_RunChassisDirectionCalPhase("Right+300", 0, CHASSIS_CAL_DUTY);
  Chassis_Stop();
  osDelay(CHASSIS_CAL_BETWEEN_STOP_MS);

  App_RunChassisDirectionCalPhase("Right-300", 0, -CHASSIS_CAL_DUTY);
  Chassis_Stop();

  APP_LOG("[CHASSIS_CAL] done");
}

static void App_RunChassisDirectionCalPhase(const char *action, int16_t left_duty, int16_t right_duty)
{
  MotorDriver_ResetEncoders();
  Chassis_SetRaw(left_duty, right_duty);

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < CHASSIS_CAL_RUN_MS;
       elapsed_ms += CHASSIS_CAL_LOG_PERIOD_MS)
  {
    osDelay(CHASSIS_CAL_LOG_PERIOD_MS);
    App_LogChassisDirectionCalStatus(action, left_duty, right_duty);
  }
}

static void App_LogChassisDirectionCalStatus(const char *action, int16_t left_duty, int16_t right_duty)
{
  APP_LOG("[CHASSIS_CAL] action=%s left_duty=%d right_duty=%d encA=%ld encB=%ld",
          action,
          (int)left_duty,
          (int)right_duty,
          (long)MotorDriver_GetEncoderA(),
          (long)MotorDriver_GetEncoderB());
}
#endif

#if APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST
static void App_RunChassisGroundTractionTest(void)
{
  APP_LOG("[CHASSIS_GROUND] traction test start");

  Chassis_Init();
  Chassis_Stop();
  osDelay(CHASSIS_GROUND_INITIAL_STOP_MS);

  App_RunChassisGroundTractionPhase(CHASSIS_AIR_TEST_DUTY);
  Chassis_Stop();
  osDelay(CHASSIS_GROUND_BETWEEN_STOP_MS);

  App_RunChassisGroundTractionPhase(400);
  Chassis_Stop();
  osDelay(CHASSIS_GROUND_BETWEEN_STOP_MS);

  App_RunChassisGroundTractionPhase(CHASSIS_GROUND_TEST_DUTY);
  Chassis_Stop();
  osDelay(CHASSIS_GROUND_BETWEEN_STOP_MS);

  App_RunChassisGroundTractionPhase(CHASSIS_MAX_OPENLOOP_DUTY);
  Chassis_Stop();

  APP_LOG("[CHASSIS_GROUND] traction test done");
}

static void App_RunChassisGroundTractionPhase(int16_t duty)
{
  MotorDriver_ResetEncoders();

  int32_t previous_enc_a = MotorDriver_GetEncoderA();
  int32_t previous_enc_b = MotorDriver_GetEncoderB();

  Chassis_Forward(duty);

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < CHASSIS_GROUND_RUN_MS;
       elapsed_ms += CHASSIS_GROUND_LOG_PERIOD_MS)
  {
    osDelay(CHASSIS_GROUND_LOG_PERIOD_MS);

    int32_t enc_a = MotorDriver_GetEncoderA();
    int32_t enc_b = MotorDriver_GetEncoderB();
    int32_t delta_a = enc_a - previous_enc_a;
    int32_t delta_b = enc_b - previous_enc_b;

    App_LogChassisGroundTractionStatus(duty, enc_a, enc_b, delta_a, delta_b);

    previous_enc_a = enc_a;
    previous_enc_b = enc_b;
  }
}

static void App_LogChassisGroundTractionStatus(int16_t duty,
                                               int32_t enc_a,
                                               int32_t enc_b,
                                               int32_t delta_a,
                                               int32_t delta_b)
{
  APP_LOG("[CHASSIS_GROUND] duty=%d encA=%ld encB=%ld deltaA=%ld deltaB=%ld",
          (int)duty,
          (long)enc_a,
          (long)enc_b,
          (long)delta_a,
          (long)delta_b);
}
#endif

#if APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST
static void App_RunChassisSpeedBalanceTest(void)
{
  APP_LOG("[BALANCE] speed balance test start");

  Chassis_Init();
  Chassis_Stop();
  osDelay(CHASSIS_BALANCE_INITIAL_STOP_MS);

  MotorDriver_ResetEncoders();
  App_RunChassisSpeedBalancePhase("forward", 1, CHASSIS_BALANCE_BASE_DUTY);
  Chassis_Stop();
  osDelay(CHASSIS_BALANCE_BETWEEN_STOP_MS);

  MotorDriver_ResetEncoders();
  App_RunChassisSpeedBalancePhase("backward", -1, CHASSIS_BALANCE_BASE_DUTY);
  Chassis_Stop();

  APP_LOG("[BALANCE] speed balance test done");
}

static void App_RunChassisSpeedBalancePhase(const char *direction_name,
                                            int8_t direction,
                                            int16_t base_duty)
{
  int32_t previous_enc_a = MotorDriver_GetEncoderA();
  int32_t previous_enc_b = MotorDriver_GetEncoderB();
  int32_t left_target = (int32_t)base_duty + CHASSIS_LEFT_TRIM;
  int32_t right_target = (int32_t)base_duty + CHASSIS_RIGHT_TRIM;
  int16_t left_mag = App_LimitInt16(left_target, CHASSIS_BALANCE_DUTY_MIN, CHASSIS_BALANCE_DUTY_MAX);
  int16_t right_mag = App_LimitInt16(right_target, CHASSIS_BALANCE_DUTY_MIN, CHASSIS_BALANCE_DUTY_MAX);
  int16_t left_cmd = (direction > 0) ? left_mag : (int16_t)-left_mag;
  int16_t right_cmd = (direction > 0) ? right_mag : (int16_t)-right_mag;

  Chassis_SetRaw(left_cmd, right_cmd);

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < CHASSIS_BALANCE_RUN_MS;
       elapsed_ms += CHASSIS_BALANCE_LOG_PERIOD_MS)
  {
    osDelay(CHASSIS_BALANCE_LOG_PERIOD_MS);

    int32_t enc_a = MotorDriver_GetEncoderA();
    int32_t enc_b = MotorDriver_GetEncoderB();
    int32_t delta_a = enc_a - previous_enc_a;
    int32_t delta_b = enc_b - previous_enc_b;
    int32_t speed_left = App_AbsI32(delta_a);
    int32_t speed_right = App_AbsI32(delta_b);
    int32_t err = speed_left - speed_right;
    int16_t correction = App_LimitInt16((int32_t)CHASSIS_BALANCE_KP * err,
                                        -CHASSIS_BALANCE_CORRECTION_LIMIT,
                                        CHASSIS_BALANCE_CORRECTION_LIMIT);
    left_target = (int32_t)base_duty - correction;
    right_target = (int32_t)base_duty + correction;
    left_target += CHASSIS_LEFT_TRIM;
    right_target += CHASSIS_RIGHT_TRIM;

    left_mag = App_LimitInt16(left_target, CHASSIS_BALANCE_DUTY_MIN, CHASSIS_BALANCE_DUTY_MAX);
    right_mag = App_LimitInt16(right_target, CHASSIS_BALANCE_DUTY_MIN, CHASSIS_BALANCE_DUTY_MAX);

    left_cmd = (direction > 0) ? left_mag : (int16_t)-left_mag;
    right_cmd = (direction > 0) ? right_mag : (int16_t)-right_mag;
    Chassis_SetRaw(left_cmd, right_cmd);

    App_LogChassisSpeedBalanceStatus(direction_name,
                                     base_duty,
                                     left_cmd,
                                     right_cmd,
                                     enc_a,
                                     enc_b,
                                     delta_a,
                                     delta_b,
                                     err);

    previous_enc_a = enc_a;
    previous_enc_b = enc_b;
  }
}

static int32_t App_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int16_t App_LimitInt16(int32_t value, int16_t min_value, int16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return (int16_t)value;
}

static void App_LogChassisSpeedBalanceStatus(const char *direction_name,
                                             int16_t base_duty,
                                             int16_t left_duty,
                                             int16_t right_duty,
                                             int32_t enc_a,
                                             int32_t enc_b,
                                             int32_t delta_a,
                                             int32_t delta_b,
                                             int32_t err)
{
  char log_line[192];

  (void)snprintf(log_line,
                 sizeof(log_line),
                 "[BALANCE] dir=%s base=%d left_trim=%d right_trim=%d left_duty=%d right_duty=%d encA=%ld encB=%ld dA=%ld dB=%ld err=%ld\r\n",
                 direction_name,
                 (int)base_duty,
                 (int)CHASSIS_LEFT_TRIM,
                 (int)CHASSIS_RIGHT_TRIM,
                 (int)left_duty,
                 (int)right_duty,
                 (long)enc_a,
                 (long)enc_b,
                 (long)delta_a,
                 (long)delta_b,
                 (long)err);
  APP_LOG_RAW(log_line);
}
#endif

#if APP_ENABLE_WHEEL_SPEED_PI_TEST
static void App_RunWheelSpeedPiTest(void)
{
  static const WheelSpeedController_Config_t wheel_speed_pi_config = {
    .target_ticks_per_sample = WHEEL_SPEED_PI_TARGET_TICKS_PER_SAMPLE,
    .feedforward_duty = WHEEL_SPEED_PI_FEEDFORWARD_DUTY,
    .kp_num = WHEEL_SPEED_PI_KP_NUM,
    .ki_num = WHEEL_SPEED_PI_KI_NUM,
    .gain_den = WHEEL_SPEED_PI_GAIN_DEN,
    .integral_limit = WHEEL_SPEED_PI_INTEGRAL_LIMIT,
    .duty_min = WHEEL_SPEED_PI_DUTY_MIN,
    .duty_max = WHEEL_SPEED_PI_DUTY_MAX,
  };

  APP_LOG("[PI] wheel speed PI test start");

  Chassis_Init();
  Chassis_Stop();
  osDelay(WHEEL_SPEED_PI_INITIAL_STOP_MS);

  MotorDriver_ResetEncoders();
  App_RunWheelSpeedPiPhase("forward", 1, &wheel_speed_pi_config);
  Chassis_Stop();
  osDelay(WHEEL_SPEED_PI_BETWEEN_STOP_MS);

  MotorDriver_ResetEncoders();
  App_RunWheelSpeedPiPhase("backward", -1, &wheel_speed_pi_config);
  Chassis_Stop();

  APP_LOG("[PI] wheel speed PI test done");
}

static void App_RunWheelSpeedPiPhase(const char *direction_name,
                                     int8_t direction,
                                     const WheelSpeedController_Config_t *config)
{
  WheelSpeedController_State_t left_state;
  WheelSpeedController_State_t right_state;

  WheelSpeedController_Reset(&left_state);
  WheelSpeedController_Reset(&right_state);

  int32_t previous_enc_a = MotorDriver_GetEncoderA();
  int32_t previous_enc_b = MotorDriver_GetEncoderB();
  int16_t left_mag = config->feedforward_duty;
  int16_t right_mag = config->feedforward_duty;
  int16_t left_cmd = (direction > 0) ? left_mag : (int16_t)-left_mag;
  int16_t right_cmd = (direction > 0) ? right_mag : (int16_t)-right_mag;

  Chassis_SetRaw(left_cmd, right_cmd);

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < WHEEL_SPEED_PI_RUN_MS;
       elapsed_ms += WHEEL_SPEED_PI_SAMPLE_PERIOD_MS)
  {
    osDelay(WHEEL_SPEED_PI_SAMPLE_PERIOD_MS);

    int32_t enc_a = MotorDriver_GetEncoderA();
    int32_t enc_b = MotorDriver_GetEncoderB();
    int32_t delta_a = enc_a - previous_enc_a;
    int32_t delta_b = enc_b - previous_enc_b;
    int32_t speed_left = (delta_a < 0) ? -delta_a : delta_a;
    int32_t speed_right = (delta_b < 0) ? -delta_b : delta_b;

    left_mag = WheelSpeedController_Update(&left_state, config, speed_left);
    right_mag = WheelSpeedController_Update(&right_state, config, speed_right);
    left_cmd = (direction > 0) ? left_mag : (int16_t)-left_mag;
    right_cmd = (direction > 0) ? right_mag : (int16_t)-right_mag;

    Chassis_SetRaw(left_cmd, right_cmd);
    App_LogWheelSpeedPiStatus(direction_name,
                              config->target_ticks_per_sample,
                              left_cmd,
                              right_cmd,
                              enc_a,
                              enc_b,
                              delta_a,
                              delta_b,
                              &left_state,
                              &right_state);

    previous_enc_a = enc_a;
    previous_enc_b = enc_b;
  }
}

static void App_LogWheelSpeedPiStatus(const char *direction_name,
                                      int32_t target_ticks_per_sample,
                                      int16_t left_duty,
                                      int16_t right_duty,
                                      int32_t enc_a,
                                      int32_t enc_b,
                                      int32_t delta_a,
                                      int32_t delta_b,
                                      const WheelSpeedController_State_t *left_state,
                                      const WheelSpeedController_State_t *right_state)
{
  char log_line[256];

  (void)snprintf(log_line,
                 sizeof(log_line),
                 "[PI] dir=%s target=%ld left_duty=%d right_duty=%d encA=%ld encB=%ld dA=%ld dB=%ld errL=%ld errR=%ld intL=%ld intR=%ld\r\n",
                 direction_name,
                 (long)target_ticks_per_sample,
                 (int)left_duty,
                 (int)right_duty,
                 (long)enc_a,
                 (long)enc_b,
                 (long)delta_a,
                 (long)delta_b,
                 (long)left_state->error,
                 (long)right_state->error,
                 (long)left_state->integral,
                 (long)right_state->integral);
  APP_LOG_RAW(log_line);
}
#endif

#if APP_ENABLE_HEADING_HOLD_TEST
static void App_RunHeadingHoldTest(void)
{
  static const WheelSpeedController_Config_t heading_speed_pi_config = {
    .target_ticks_per_sample = HEADING_HOLD_BASE_TARGET_TICKS_PER_SAMPLE,
    .feedforward_duty = WHEEL_SPEED_PI_FEEDFORWARD_DUTY,
    .kp_num = WHEEL_SPEED_PI_KP_NUM,
    .ki_num = WHEEL_SPEED_PI_KI_NUM,
    .gain_den = WHEEL_SPEED_PI_GAIN_DEN,
    .integral_limit = WHEEL_SPEED_PI_INTEGRAL_LIMIT,
    .duty_min = WHEEL_SPEED_PI_DUTY_MIN,
    .duty_max = WHEEL_SPEED_PI_DUTY_MAX,
  };

  float yaw_deg = 0.0f;
  uint32_t last_imu_update_ms = 0U;

  APP_LOG("[HEADING] heading hold test start");

  Chassis_Init();
  Chassis_Stop();
  osDelay(HEADING_HOLD_INITIAL_STOP_MS);

  last_imu_update_ms = 0U;
  yaw_deg = App_UpdateHeadingYawDeg(yaw_deg, &last_imu_update_ms);
  float target_yaw_deg = yaw_deg;

  MotorDriver_ResetEncoders();
  App_RunHeadingHoldPhase("forward",
                          1,
                          target_yaw_deg,
                          &yaw_deg,
                          &last_imu_update_ms,
                          &heading_speed_pi_config);
  Chassis_Stop();
  MotorDriver_StopAll();
  osDelay(HEADING_HOLD_BETWEEN_STOP_MS);

  last_imu_update_ms = 0U;
  yaw_deg = App_UpdateHeadingYawDeg(yaw_deg, &last_imu_update_ms);
  target_yaw_deg = yaw_deg;

  MotorDriver_ResetEncoders();
  App_RunHeadingHoldPhase("backward",
                          -1,
                          target_yaw_deg,
                          &yaw_deg,
                          &last_imu_update_ms,
                          &heading_speed_pi_config);
  Chassis_Stop();
  MotorDriver_StopAll();

  APP_LOG("[HEADING] heading hold test done");
}

static void App_RunHeadingHoldPhase(const char *direction_name,
                                    int8_t direction,
                                    float target_yaw_deg,
                                    float *yaw_deg,
                                    uint32_t *last_imu_update_ms,
                                    const WheelSpeedController_Config_t *base_config)
{
  WheelSpeedController_Config_t left_config = *base_config;
  WheelSpeedController_Config_t right_config = *base_config;
  WheelSpeedController_State_t left_state;
  WheelSpeedController_State_t right_state;

  WheelSpeedController_Reset(&left_state);
  WheelSpeedController_Reset(&right_state);

  int32_t previous_enc_a = MotorDriver_GetEncoderA();
  int32_t previous_enc_b = MotorDriver_GetEncoderB();
  int16_t left_mag = base_config->feedforward_duty;
  int16_t right_mag = base_config->feedforward_duty;
  int16_t left_cmd = (direction > 0) ? left_mag : (int16_t)-left_mag;
  int16_t right_cmd = (direction > 0) ? right_mag : (int16_t)-right_mag;
  int32_t k_heading = (direction > 0) ? HEADING_K_FORWARD : HEADING_K_BACKWARD;
  int32_t correction_limit_ticks = (direction > 0) ? HEADING_CORR_LIMIT_FORWARD : HEADING_CORR_LIMIT_BACKWARD;

  Chassis_SetRaw(left_cmd, right_cmd);

  for (uint32_t elapsed_ms = 0U;
       elapsed_ms < HEADING_HOLD_RUN_MS;
       elapsed_ms += HEADING_HOLD_SAMPLE_PERIOD_MS)
  {
    osDelay(HEADING_HOLD_SAMPLE_PERIOD_MS);

    *yaw_deg = App_UpdateHeadingYawDeg(*yaw_deg, last_imu_update_ms);

    int32_t enc_a = MotorDriver_GetEncoderA();
    int32_t enc_b = MotorDriver_GetEncoderB();
    int32_t delta_a = enc_a - previous_enc_a;
    int32_t delta_b = enc_b - previous_enc_b;
    int32_t speed_left = (delta_a < 0) ? -delta_a : delta_a;
    int32_t speed_right = (delta_b < 0) ? -delta_b : delta_b;
    float yaw_error_deg = *yaw_deg - target_yaw_deg;
    int32_t yaw_error_tenth_deg = App_ScaleFloatRounded(yaw_error_deg, 10.0f);
    int32_t heading_correction_ticks = (k_heading * yaw_error_tenth_deg) / 10;

    heading_correction_ticks = App_LimitI32(heading_correction_ticks,
                                            -correction_limit_ticks,
                                            correction_limit_ticks);

    int32_t left_target_ticks = App_LimitI32(HEADING_HOLD_BASE_TARGET_TICKS_PER_SAMPLE + heading_correction_ticks,
                                            HEADING_HOLD_TARGET_MIN_TICKS,
                                            HEADING_HOLD_TARGET_MAX_TICKS);
    int32_t right_target_ticks = App_LimitI32(HEADING_HOLD_BASE_TARGET_TICKS_PER_SAMPLE - heading_correction_ticks,
                                             HEADING_HOLD_TARGET_MIN_TICKS,
                                             HEADING_HOLD_TARGET_MAX_TICKS);

    left_config.target_ticks_per_sample = left_target_ticks;
    right_config.target_ticks_per_sample = right_target_ticks;

    left_mag = WheelSpeedController_Update(&left_state, &left_config, speed_left);
    right_mag = WheelSpeedController_Update(&right_state, &right_config, speed_right);
    left_cmd = (direction > 0) ? left_mag : (int16_t)-left_mag;
    right_cmd = (direction > 0) ? right_mag : (int16_t)-right_mag;

    Chassis_SetRaw(left_cmd, right_cmd);
    App_LogHeadingHoldStatus(direction_name,
                             target_yaw_deg,
                             *yaw_deg,
                             yaw_error_deg,
                             k_heading,
                             correction_limit_ticks,
                             heading_correction_ticks,
                             left_target_ticks,
                             right_target_ticks,
                             delta_a,
                             delta_b,
                             left_cmd,
                             right_cmd);

    previous_enc_a = enc_a;
    previous_enc_b = enc_b;
  }
}

static float App_UpdateHeadingYawDeg(float yaw_deg, uint32_t *last_imu_update_ms)
{
  MPU6500_Data_t imu;

  if ((last_imu_update_ms == 0) || (MPU6500_GetLatest(&imu) == false))
  {
    return yaw_deg;
  }

  if (*last_imu_update_ms == 0U)
  {
    *last_imu_update_ms = imu.last_update_ms;
    return yaw_deg;
  }

  uint32_t dt_ms = imu.last_update_ms - *last_imu_update_ms;
  *last_imu_update_ms = imu.last_update_ms;

  if ((dt_ms == 0U) || (dt_ms > 500U))
  {
    return yaw_deg;
  }

  return yaw_deg + (imu.gz_dps * ((float)dt_ms / 1000.0f));
}

static int32_t App_LimitI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static void App_LogHeadingHoldStatus(const char *direction_name,
                                     float target_yaw_deg,
                                     float yaw_deg,
                                     float yaw_error_deg,
                                     int32_t k_heading,
                                     int32_t correction_limit_ticks,
                                     int32_t heading_correction_ticks,
                                     int32_t left_target_ticks,
                                     int32_t right_target_ticks,
                                     int32_t delta_a,
                                     int32_t delta_b,
                                     int16_t left_duty,
                                     int16_t right_duty)
{
  char log_line[384];
  int32_t target_yaw_tenth_deg = App_ScaleFloatRounded(target_yaw_deg, 10.0f);
  int32_t yaw_tenth_deg = App_ScaleFloatRounded(yaw_deg, 10.0f);
  int32_t yaw_error_tenth_deg = App_ScaleFloatRounded(yaw_error_deg, 10.0f);

  (void)snprintf(log_line,
                 sizeof(log_line),
                 "[HEADING] dir=%s target_yaw=%s%lu.%01lu yaw=%s%lu.%01lu yaw_error_current_minus_target=%s%lu.%01lu k_heading=%ld corr_limit=%ld heading_corr=%ld left_target=%ld right_target=%ld dA=%ld dB=%ld left_duty=%d right_duty=%d\r\n",
                 direction_name,
                 App_FixedSign(target_yaw_tenth_deg),
                 (unsigned long)App_FixedWhole(target_yaw_tenth_deg, 10),
                 (unsigned long)App_FixedFraction(target_yaw_tenth_deg, 10),
                 App_FixedSign(yaw_tenth_deg),
                 (unsigned long)App_FixedWhole(yaw_tenth_deg, 10),
                 (unsigned long)App_FixedFraction(yaw_tenth_deg, 10),
                 App_FixedSign(yaw_error_tenth_deg),
                 (unsigned long)App_FixedWhole(yaw_error_tenth_deg, 10),
                 (unsigned long)App_FixedFraction(yaw_error_tenth_deg, 10),
                 (long)k_heading,
                 (long)correction_limit_ticks,
                 (long)heading_correction_ticks,
                 (long)left_target_ticks,
                 (long)right_target_ticks,
                 (long)delta_a,
                 (long)delta_b,
                 (int)left_duty,
                 (int)right_duty);
  APP_LOG_RAW(log_line);
}
#endif

#if APP_ENABLE_MOTOR_GPIO_STATIC_TEST
static void App_RunMotorGpioStaticTest(void)
{
  APP_LOG("[MOTOR_GPIO] static test start");

  App_InitMotorGpioStaticPins();

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  APP_LOG("[MOTOR_GPIO] all low");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  osDelay(MOTOR_TEST_INITIAL_STOP_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  APP_LOG("[MOTOR_GPIO] PA6 high");
  App_LogMotorGpioStaticState(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  APP_LOG("[MOTOR_GPIO] PA7 high");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  APP_LOG("[MOTOR_GPIO] PB0 high");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
  APP_LOG("[MOTOR_GPIO] PB1 high");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  APP_LOG("[MOTOR_GPIO] AIN1 and BIN1 high");
  App_LogMotorGpioStaticState(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);
  APP_LOG("[MOTOR_GPIO] AIN2 and BIN2 high");
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);
  osDelay(MOTOR_GPIO_STATIC_HOLD_MS);

  App_SetMotorGpioStaticPins(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  App_LogMotorGpioStaticState(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  APP_LOG("[MOTOR_GPIO] static test done");
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
  APP_LOG("[MOTOR_GPIO] PA6=%u PA7=%u PB0=%u PB1=%u",
          (pa6_state == GPIO_PIN_SET) ? 1U : 0U,
          (pa7_state == GPIO_PIN_SET) ? 1U : 0U,
          (pb0_state == GPIO_PIN_SET) ? 1U : 0U,
          (pb1_state == GPIO_PIN_SET) ? 1U : 0U);
}
#endif

/* USER CODE END Application */

