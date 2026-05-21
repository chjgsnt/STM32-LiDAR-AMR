# Progress Report

Last updated: 2026-05-22

## Bring-Up Summary

- Step 0: Baseline project structure created.
- Step 1: STM32CubeMX configuration checked for core bring-up peripherals and middleware.
- Step 2: Minimal hardware bring-up verified with USART2 logging, FreeRTOS startup, I2C1 scan, and basic I2C module checks.
- Step 3: AT8236 motor driver, Motor A/B PWM forward/reverse, and TIM2/TIM4 encoder counting verified.
- Step 4: Chassis open-loop test completed.
- Step 5: Encoder speed balance test completed.
- Step 6: Wheel speed PI closed-loop test completed.
- Step 7: Heading hold straight-line test has a saved baseline.
- Step 8: Test mode and logging cleanup is in progress.
- Step 9: RPLIDAR C1 UART4 bring-up wiring and CubeMX configuration are recorded.
- LiDAR obstacle avoidance integration status is recorded in `Docs/02_System_Integration/Obstacle_Avoidance_Status.md`.
- `APP_ACTIVE_MODE` now selects the main runtime mode. The default mode is LiDAR obstacle dry-run, so default firmware does not drive the wheels.

## Step4 Chassis Open-Loop Test

### Test Objective

- Verify chassis-level open-loop motion above the tested AT8236 motor driver.
- Confirm Motor A/B PWM forward/reverse control still works through the chassis abstraction.
- Confirm TIM2/TIM4 encoder counts update during chassis movement.
- Confirm the MPU6500 IMU remains ready during motion.
- Verify chassis `Forward`, `Backward`, `TurnLeft`, and `TurnRight` actions.

### Key Implementation

- Added and used the chassis open-loop motion layer on top of the verified motor driver path.
- Mapped Motor A as the left wheel and Motor B as the right wheel.
- Used signed wheel commands to correct physical wheel direction.
- Ran the chassis sequence without encoder speed balance or PID control.

### Test Results

- `MPU6500 IMU ready=1` remained valid during the test.
- AT8236 motor driver operation was confirmed.
- Motor A/B PWM forward/reverse operation was confirmed.
- TIM2/TIM4 encoder counting was confirmed.
- Chassis `Forward`, `Backward`, `TurnLeft`, and `TurnRight` actions were verified.
- Step4 chassis open-loop test is complete.

### Final Direction Configuration

```c
CHASSIS_LEFT_SIGN = -1
CHASSIS_RIGHT_SIGN = 1
```

### Ground Duty Selection

- Lifted-wheel direction checks can run at lower duty values.
- Ground open-loop testing selected `duty=500` as the current reliable test value.

### Known Limitation

- Open-loop `Forward` / `Backward` motion is not straight enough because the left and right wheel speeds are not yet balanced.
- This is expected before encoder speed balance and speed PID are added.

### Next Stage Plan

- Step5 Encoder Speed Balance Test is complete.
- Proceed toward closed-loop wheel speed PID for straighter chassis motion.
- Optionally clean up UART logging with a log mutex / serialized logging path.

## Step5 Encoder Speed Balance Test

### Test Objective

- Estimate left and right wheel speed from TIM2/TIM4 encoder count deltas.
- Verify a first-pass P-only left/right wheel speed balance above the chassis open-loop layer.
- Confirm the MPU6500 IMU remains ready during the balance test.
- Improve straight-line `Forward` / `Backward` behavior before adding full speed PID.

### Key Implementation

- Used `base_duty=500` for the forward/backward balance test.
- Applied static duty trims:

```c
CHASSIS_LEFT_TRIM = -10
CHASSIS_RIGHT_TRIM = 10
```

- Used P-only correction from the left/right encoder delta error.
- Kept the final chassis direction configuration unchanged:

```c
CHASSIS_LEFT_SIGN = -1
CHASSIS_RIGHT_SIGN = 1
```

### Test Results

- Encoder speed balance test passed.
- P-only balance can reduce left/right wheel speed difference.
- During the second half of forward motion, `err` can usually be reduced to about `+/-10 ticks/sample`.
- Backward motion can run, but `err` fluctuates slightly more than forward motion.
- `MPU6500 IMU ready=1` remained valid during the test.

### Known Issue

- UART logs may interleave when IMU and BALANCE logs print at the same time.

### Next Stage Plan

- Add closed-loop wheel speed PID for more stable forward/backward motion.
- Continue optimizing backward speed balance in the PID stage.
- Optionally add a log mutex or serialized logging path to avoid interleaved UART messages.

## Step6 Wheel Speed PI Closed-Loop Test

### Test Objective

- Verify independent PI speed control for the left and right wheels.
- Use encoder delta magnitude as wheel speed feedback.
- Confirm Forward PI and Backward PI can complete with the chassis on the ground.
- Confirm the MPU6500 IMU and I2C baseline remain healthy during the PI test.

### Key Implementation

- Used `target_ticks_per_sample=800`.
- Used `feedforward_duty=500`.
- Limited PI output duty magnitude to `300..600`.
- Ran each direction with a `200 ms` control period.
- Calculated speed feedback from `abs(deltaA)` and `abs(deltaB)`.

### Test Results

- Wheel speed PI closed-loop test passed.
- Forward PI and Backward PI both ran to completion.
- During startup, duty briefly reached the upper limit `600`.
- After the startup transient, duty stabilized at about `470..500`.
- In the later part of both forward and backward runs, encoder deltas were mostly stable at about `810..830 ticks/sample`, close to `target=800`.
- `APP IMU: ready=1` remained valid during the PI test.
- I2C baseline remained normal: `SCL=1`, `SDA=1`.

### Known Limitations

- Actual wheel speed is still slightly higher than the target; later tuning can reduce `feedforward_duty` or `Ki`.
- UART logs may occasionally interleave with IMU logs; a log mutex can be added later.

### Next Stage Plan

- Tune PI parameters for tighter speed tracking.
- Or add IMU yaw heading hold above the wheel speed PI layer.

## Step7 Heading Hold Straight-Line Test

### Test Objective

- Add a short-duration heading hold layer above the Step6 wheel speed PI controller.
- Estimate relative yaw by integrating MPU6500 gyro-Z rate.
- Bias left and right wheel speed targets to reduce yaw drift.
- Confirm the MPU6500 IMU and I2C baseline remain healthy during heading hold motion.

### Key Implementation

- Used gyro-Z yaw integration from the MPU6500.
- Used `yaw_error = current_yaw - target_yaw`.
- Used explicit left/right target correction:

```text
heading_correction = K_heading * yaw_error
left_target = base_target + heading_correction
right_target = base_target - heading_correction
```

- Reused the Step6 wheel speed PI controller without changing the PI parameters.
- Kept `target_ticks_per_sample=800`.
- Current conservative heading parameters:

```c
HEADING_K_FORWARD = 2
HEADING_K_BACKWARD = 2
HEADING_CORR_LIMIT_FORWARD = 50
HEADING_CORR_LIMIT_BACKWARD = 50
```

### Test Results

- Step7 heading hold test now has a saved baseline.
- Forward heading hold is clearly improved; final yaw was about `7.1 deg`.
- Backward heading hold can run, but final yaw error was larger at about `+7 deg` relative to the backward target.
- `APP IMU: ready=1` remained valid during the test.
- I2C baseline remained normal.

### Known Limitations

- Backward heading hold still needs tuning.
- Gyro-only yaw integration is relative and will drift over longer runs.
- UART logs may occasionally interleave with IMU or control logs.

### Next Stage Plan

- Tune backward heading correction.
- Add odometry-yaw fusion above the wheel speed PI layer.

## Step8 Test Mode and Logging Cleanup

### Objective

- Centralize Step3 through Step7 bring-up test selection.
- Keep existing motor, chassis, encoder balance, wheel PI, and heading hold tests available.
- Make the default configuration enable only one active test mode.
- Reduce noisy I2C scan output before LiDAR and navigation integration work.
- Improve UART log consistency so IMU and control-task lines are less likely to interleave.

### Key Implementation

- Added `Core/Inc/app_config.h` as the central test-mode configuration header.
- Added `APP_ACTIVE_TEST` with mode IDs:

```c
APP_TEST_NONE                  0
APP_TEST_MOTOR_PWM             1
APP_TEST_CHASSIS_OPENLOOP      2
APP_TEST_CHASSIS_SPEED_BALANCE 3
APP_TEST_WHEEL_SPEED_PI        4
APP_TEST_HEADING_HOLD          5
```

- Kept the legacy `APP_ENABLE_*` switches for compatibility, with a compile-time check that rejects multiple active tests.
- Added `APP_LOG(...)`, `APP_LOG_RAW(...)`, and timestamped `LOG_INFO(...)` on top of a single-line buffered log function.
- Added a FreeRTOS log mutex initialized before tasks are created.
- Added `APP_I2C_SCAN_VERBOSE`, default `0`, to suppress per-address `HAL_ERROR` noise during normal boot scans.
- Kept I2C baseline and recovery logs available through `APP_ENABLE_I2C_BASELINE_DEBUG` and `APP_ENABLE_I2C_BUS_RECOVERY`.

### Known Limitations

- Direct `printf` calls outside `APP_LOG`, `LOG_INFO`, or `APP_LOG_RAW` should be avoided.
- Step7 backward heading hold still needs tuning.
- LiDAR UART4 bring-up is prepared; raw UART reception and navigation integration are not completed yet.

### Next Stage Plan

- Proceed to raw UART4 reception byte-count verification for RPLIDAR C1.
- Continue backward heading correction tuning or add odometry-yaw fusion when returning to chassis control.

## Step9 RPLIDAR C1 UART4 Bring-up Preparation

### Objective

- Record RPLIDAR C1 TTL UART wiring on `UART4`.
- Record the CubeMX UART4 settings for `460800` baud, `8N1`.
- Keep USART1 reserved for debug logging.
- Keep validated motor-related pins unchanged.
- Prepare only raw UART bring-up; do not mark distance parsing or obstacle avoidance as completed.

### Key Configuration

| Item | Value |
| --- | --- |
| LiDAR model | SLAMTEC RPLIDAR C1 |
| Interface | TTL UART |
| MCU | STM32F446RETx |
| UART | `UART4` |
| Baud / format | `460800`, `8N1` |
| UART4 TX | `PC10` |
| UART4 RX | `PC11` |

### Wiring

| RPLIDAR C1 wire | Connection |
| --- | --- |
| Red `VCC` | `5V` |
| Black `GND` | `GND` |
| Yellow `TX` | `PC11` / `UART4_RX` |
| Green `RX` | `PC10` / `UART4_TX` |

TX/RX must be crossed: LiDAR `TX` -> MCU `RX`, and LiDAR `RX` -> MCU `TX`.

### Current Status / Next Step

- UART4 has been configured for RPLIDAR C1.
- Hardware wiring has been completed.
- Next step is to add raw UART reception code and verify rx byte count at 460800 baud.

## LiDAR Obstacle Avoidance Integration Status

### Objective

- Record the current unified runtime mode selector.
- Record the current LiDAR obstacle state machine behavior.
- Record corner escape and IMU heading assist dry-run progress.
- Keep the default safety rule visible in the project documentation.

### Current Status

- `APP_ACTIVE_MODE` is the central runtime selector.
- Default mode is `APP_MODE_LIDAR_OBSTACLE_DRY_RUN`, with motor output disabled.
- `APP_MODE_LIDAR_OBSTACLE_GROUND_TEST` is available for supervised short ground tests.
- `APP_MODE_IMU_HEADING_ASSIST_DRY_RUN` is available for fixed `FORWARD_SLOW` heading-correction logging.
- The obstacle state machine includes `FORWARD`, `TURN`, `BACKUP`, `BLOCKED`, `CORNER_BACKUP`, and `CORNER_TURN` behavior.
- `front_wide`, `obs_front`, escape lock, timeout/reselect, and corner escape recovery have been added.
- IMU heading assist currently defaults to `apply=0`, so correction is logged but not applied to motor output.

### Known Limitations

- Wall corners can still cause spinning or repeated in-place turning.
- IMU heading assist should stay log-only until lifted-wheel testing verifies correction sign and left/right output behavior.

Detailed notes and commands are in `Docs/02_System_Integration/Obstacle_Avoidance_Status.md`.
