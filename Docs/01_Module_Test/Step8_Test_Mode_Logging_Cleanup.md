# Step 8 Test Mode and Logging Cleanup

Target: STM32 NUCLEO-F446RE / STM32F446RE

## Why Step8

Step3 through Step7 created several useful hardware bring-up tests:

- Step3 Motor Driver Bring-up
- Step4 Chassis Open-Loop Test
- Step5 Encoder Speed Balance Test
- Step6 Wheel Speed PI Test
- Step7 Heading Hold Test

Those tests should remain available, but only one should run at a time. Step8 centralizes the test mode selection and reduces noisy logs so future LiDAR and navigation work starts from a cleaner baseline.

Step8 does not add a new control algorithm and does not delete the existing Step3 through Step7 test code.

## APP_ACTIVE_TEST

Test mode selection is centralized in:

```text
AMR_LiDAR_Robot/Core/Inc/app_config.h
```

The default active test is:

```c
#define APP_ACTIVE_TEST APP_TEST_HEADING_HOLD
```

To select a different test, change `APP_ACTIVE_TEST` or override it from the build system. The legacy `APP_ENABLE_*` switches are still supported for compatibility, but a compile-time check rejects configurations where more than one test is enabled.

## Test Mode IDs

```c
#define APP_TEST_NONE                  0
#define APP_TEST_MOTOR_PWM             1
#define APP_TEST_CHASSIS_OPENLOOP      2
#define APP_TEST_CHASSIS_SPEED_BALANCE 3
#define APP_TEST_WHEEL_SPEED_PI        4
#define APP_TEST_HEADING_HOLD          5
```

## I2C Scan Verbosity

The I2C scan is quiet by default:

```c
#define APP_I2C_SCAN_VERBOSE 0
```

With `APP_I2C_SCAN_VERBOSE=0`, the scan logs only the important lines:

- scan start
- SCL/SDA levels
- found device addresses
- final scan result
- scan finished

To debug every address probe, set:

```c
#define APP_I2C_SCAN_VERBOSE 1
```

Verbose mode restores per-address `HAL_BUSY`, `HAL_TIMEOUT`, `HAL_ERROR`, and other detailed scan logs.

## I2C Baseline Debug

I2C baseline and recovery logs remain available:

```c
#define APP_ENABLE_I2C_BASELINE_DEBUG 1
#define APP_ENABLE_I2C_BUS_RECOVERY 1
```

`APP_ENABLE_I2C_BASELINE_DEBUG=1` keeps the boot-time SCL/SDA, I2C state, and error-code baseline logs. It can be set to `0` later when the hardware baseline is stable enough.

## Logging Rule

Step8 adds `APP_LOG(...)` and keeps `LOG_INFO(...)` as the timestamped compatibility macro.

Logging rules:

- Build each log line in one call.
- Use `APP_LOG(...)`, `LOG_INFO(...)`, or `APP_LOG_RAW(...)` instead of splitting one logical line across multiple prints.
- Long test rows can still use local `snprintf` into a buffer, then `APP_LOG_RAW(buffer)`.
- In FreeRTOS, the log layer creates a UART log mutex before tasks are started, so IMU and control-task log lines are less likely to interleave.

## Known Limitations

- The mutex protects logs that go through `APP_LOG`, `LOG_INFO`, or `APP_LOG_RAW`; direct low-level `printf` calls outside this layer should be avoided.
- The default active mode is still a bring-up test mode, not the final robot application.
- Step7 backward heading hold still needs tuning.
- Future LiDAR bring-up may need its own log category or rate limiting.
