# Final Report: STM32 LiDAR AMR Demo

## Project Summary

This project is an STM32F446xx based AMR / smart path-following car built on FreeRTOS. The current final demo path is a LiDAR based obstacle avoidance mode named `LiDARObstacleAvoidance`.

The final default mode does not depend on a servo or ultrasonic module. Servo and ultrasonic based demos are kept in the codebase as optional expansion demos, but they are not required for the current hardware demonstration.

Current verified final behavior:

- Default `active_mode` is `LiDARObstacleAvoidance`.
- LiDAR UART4 RX starts successfully at `baud=460800`.
- LiDAR scan descriptor `A5 5A 05 00 00 40 81` is recognized.
- Scan response type `0x81` is detected.
- The LiDAR parser reports `valid_points`, `nearest`, `front`, `front_wide`, `left`, and `right` fields.
- The final state machine can execute:
  `START_DELAY -> WAIT_LIDAR -> DRIVE_FORWARD -> OBSTACLE_STOP -> BACKUP -> TURN -> RECOVER -> DRIVE_FORWARD`.
- A front obstacle can trigger:
  `obstacle detected front_min_mm=xxx`, stop, backup, turn, and `recover forward`.

## Actual Hardware Configuration

The project hardware used for the final demo is:

- MCU: STM32F446xx.
- RTOS: FreeRTOS.
- Drive base: left/right motor chassis.
- IMU: MPU6500 on I2C, address `0x68`.
- IMU identity: `WHO_AM_I = 0x70`.
- IMU status: pitch and roll logs are available.
- LiDAR: connected through UART4.
- LiDAR baud rate: `460800`.
- Current hardware has no servo installed.
- Current hardware has no ultrasonic module installed.
- Digital Logic Analyzer is used only as a debugging tool. It is not part of the sensor or actuator stack.

Because the current robot does not include a servo or ultrasonic sensor, the final mainline cannot depend on `ServoScanObstacle` or `NoServoObstacleAvoidance`. Those modules can remain as extension demos or fallback experiments, but they should not be documented as mandatory final-demo hardware.

## Bring-Up And Debug Process

### Toolchain And Build

The project is built with the STM32CubeCLT / CMake / Ninja / `arm-none-eabi` toolchain flow.

The normal Debug build command is:

```powershell
cmake --build --preset Debug --clean-first
```

The normal flashing command is:

```powershell
STM32_Programmer_CLI -c port=SWD -w build\Debug\AMR_LiDAR_Robot.elf -v -rst
```

This flow was used throughout bring-up to verify code changes and flash the STM32 target.

### I2C Baseline And Recovery

The I2C baseline test was used to confirm the bus before relying on the IMU.

Verified points:

- I2C1 SCL pin: `PB8`.
- I2C1 SDA pin: `PB9`.
- PB8/SCL and PB9/SDA levels were checked during baseline logging.
- I2C scan found MPU6500 at address `0x68`.
- MPU6500 `WHO_AM_I` returned `0x70`.
- I2C recovery support is present in the bring-up flow.

OLED was used earlier as one possible I2C verification target. In the current scan result the OLED status is `oled=0`, and the final mainline does not depend on OLED output.

Relevant code areas:

- `Core/Src/freertos.c`
- `Core/Src/i2c_scan.c`
- `Core/Inc/mpu6500.h`
- `Core/Src/ssd1306.c`

### IMU Bring-Up

The MPU6500 IMU bring-up included:

- Wake-up and `WHO_AM_I` verification.
- Gyro calibration.
- Raw accelerometer / gyro reading.
- Pitch and roll reporting through serial logs.

Representative IMU log format from the code:

```text
APP IMU: ready=1 pitch=...deg roll=...deg
MPU6500 angle accel: pitch=...deg roll=...deg
MPU6500 angle fused: pitch=...deg roll=...deg
```

The IMU is currently used as a verified onboard sensor and debug source. It is a good future candidate for yaw / gyro assisted turning, but the current final obstacle avoidance state machine is still reactive and LiDAR-front-distance based.

### Motor Bring-Up

The motor bring-up completed basic left/right motor control and chassis-level commands.

Verified work:

- Basic PWM motor output.
- Stop command.
- Forward command.
- Backup command.
- In-place turn command.
- Left/right duty trim experiments.

The current forward calibration for the final LiDAR demo is approximately:

```text
left duty  = 498
right duty = 500
```

Earlier tests showed that left/right wheel start behavior and duty mismatch could cause the robot to drift. The current `498/500` forward duty was chosen as a practical calibrated value for the final demo.

Relevant code areas:

- `Core/Src/motor_driver.c`
- `Core/Src/chassis.c`
- `Core/Src/app_motor_forced_spin_check.c`
- `Core/Src/app_lidar_obstacle_avoidance.c`

### Early Obstacle Experiments

Early obstacle experiments included basic stop / resume behavior and servo / ultrasonic oriented demo work.

However, the actual final hardware currently has:

- no servo,
- no ultrasonic module.

Therefore, `ServoScanObstacle` and `NoServoObstacleAvoidance` are not suitable as the final default mode. They are retained only as optional expansion demos or backup implementations.

### LiDAR Bring-Up

LiDAR bring-up reused the existing UART4 based parser instead of replacing the low-level driver.

Verified LiDAR bring-up events:

- `APP LiDAR: uart4 rx started, baud=460800`
- STOP command sent before scan startup.
- SCAN command sent.
- Descriptor observed:
  `A5 5A 05 00 00 40 81`
- Descriptor data type:
  `0x81`
- Scan response type `0x81` detected.
- `ready=1` after descriptor detection.
- `valid_points` increases while scan data is received.
- Parser reports `nearest`, `front`, `front_wide`, `left`, and `right`.

The parser extracts front-sector distance through `front_min_mm` and marks it valid with `front_valid`. The current parser also reports `front_wide_min_mm`, `left_min_mm`, and `right_min_mm`, which are useful for future direction selection.

Relevant code areas:

- `Core/Src/app_lidar.c`
- `Core/Inc/app_lidar.h`

### LiDAR Obstacle Stop Check

Before the final state-machine demo, the project first validated a simpler LiDAR stop-check path:

- Read LiDAR front sector.
- Detect front obstacle distance.
- Stop motor output when the front obstacle was too close.
- Resume or hold based on hysteresis / clear logic during earlier experiments.

This phase proved that LiDAR front distance could control motor safety behavior.

Relevant code areas:

- `Core/Src/app_lidar_stop_check.c`
- `Core/Inc/app_lidar_stop_check.h`

### Final LiDARObstacleAvoidance Demo

The final demo is implemented as `LiDARObstacleAvoidance`.

It uses LiDAR front-sector data and a conservative state machine:

```text
START_DELAY
WAIT_LIDAR
DRIVE_FORWARD
OBSTACLE_STOP
BACKUP
TURN
RECOVER
```

Obstacle behavior:

- Wait for startup delay.
- Wait until LiDAR front data is valid.
- Drive forward with calibrated motor duty.
- If `front_min_mm < LIDAR_OBS_STOP_MM`, stop.
- Backup for a fixed time.
- Turn for a fixed time.
- Recover and return to forward driving if LiDAR data is valid.

Safety behavior:

- If LiDAR front data is missing for too long, the robot enters `WAIT_LIDAR`.
- A single invalid front reading is tolerated to reduce state jitter.
- Long-term no-data behavior remains fail-safe: the robot stops instead of continuing forward.

Relevant code areas:

- `Core/Src/app_lidar_obstacle_avoidance.c`
- `Core/Inc/app_lidar_obstacle_avoidance.h`
- `Core/Src/freertos.c`
- `Core/Inc/app_test_config.h`

## Current Final Function Architecture

### Motor Control Module

The motor control layer drives the left and right motors. It supports:

- forward output,
- stop,
- backup,
- in-place turn.

The final demo calls chassis-level commands and does not require changes to `motor_driver.c`.

Main files:

- `Core/Src/motor_driver.c`
- `Core/Src/chassis.c`
- `Core/Inc/chassis.h`

### IMU Module

The IMU module reads MPU6500 data and provides debug orientation information.

Current status:

- I2C address verified at `0x68`.
- `WHO_AM_I = 0x70`.
- Gyro calibration flow is present.
- Pitch and roll serial logs are available.

Main files:

- `Core/Src/i2c_scan.c`
- `Core/Inc/mpu6500.h`
- `Core/Src/freertos.c`

### LiDAR Module

The LiDAR module receives scan data through UART4 and parses the scan response.

Current status:

- UART4 RX starts at `460800`.
- Descriptor and scan response type `0x81` are detected.
- `valid_points` increases during scanning.
- Front-sector minimum distance is exposed as `front_min_mm`.

Main files:

- `Core/Src/app_lidar.c`
- `Core/Inc/app_lidar.h`

### Decision / Control State Machine

The final decision logic is the `LiDARObstacleAvoidance` state machine.

States:

- `START_DELAY`
- `WAIT_LIDAR`
- `DRIVE_FORWARD`
- `OBSTACLE_STOP`
- `BACKUP`
- `TURN`
- `RECOVER`

The state machine uses `front_min_mm` and LiDAR validity information to decide when to drive, stop, backup, turn, and recover.

Main files:

- `Core/Src/app_lidar_obstacle_avoidance.c`
- `Core/Inc/app_lidar_obstacle_avoidance.h`

## Current LiDARObstacleAvoidance Parameters

These values are taken from the current `Core/Src/app_lidar_obstacle_avoidance.c`.

| Parameter | Current value | Meaning |
| --- | ---: | --- |
| `LIDAR_OBS_START_DELAY_MS` | `3000` ms | Initial delay before LiDAR-controlled movement |
| `LIDAR_OBS_FRONT_MIN_MM` | `120` mm | Filters out too-close invalid / unreliable front points |
| `LIDAR_OBS_STOP_MM` | `400` mm | Front obstacle trigger distance |
| `LIDAR_OBS_FRONT_TIMEOUT_MS` | `600` ms | Maximum allowed age of valid front LiDAR data |
| `LIDAR_OBS_FRONT_INVALID_LIMIT` | `3` samples | Consecutive invalid front samples tolerated before WAIT_LIDAR |
| `LIDAR_OBS_BACKUP_MS` | `450` ms | Backup duration after obstacle stop |
| `LIDAR_OBS_TURN_MS` | `1000` ms | Turn duration after backup |
| `LIDAR_OBS_RECOVER_MS` | `200` ms | Recovery pause before driving forward again |
| `LIDAR_OBS_FORWARD_LEFT_DUTY` | `498` | Left motor forward duty |
| `LIDAR_OBS_FORWARD_RIGHT_DUTY` | `500` | Right motor forward duty |
| `LIDAR_OBS_BACKUP_DUTY` | `320` | Backup duty |
| `LIDAR_OBS_TURN_DUTY` | `360` | In-place turn duty |

Tuning notes:

- Increase `LIDAR_OBS_STOP_MM` to stop earlier.
- Decrease `LIDAR_OBS_STOP_MM` only after controlled ground testing.
- Increase `LIDAR_OBS_TURN_MS` if the robot recovers and immediately sees the same obstacle again.
- Adjust `LIDAR_OBS_FORWARD_LEFT_DUTY` and `LIDAR_OBS_FORWARD_RIGHT_DUTY` for straight-line calibration.
- Increase `LIDAR_OBS_FRONT_INVALID_LIMIT` only if front readings are noisy but LiDAR data is otherwise healthy.

## Representative Test Log / Serial Verification

The following is a condensed representative log set. It records the important verification events without including every repeated motor or LiDAR status line.

```text
[APP] active_mode=LiDARObstacleAvoidance
[APP] active_test=LiDARObstacleAvoidance
[APP] LiDARObstacleAvoidance: lidar parser task active
[APP] lidar obstacle avoidance enabled

APP LiDAR: uart4 rx started, baud=460800
APP LiDAR: send STOP cmd
APP LiDAR: send SCAN cmd
APP LiDAR: descriptor=A5 5A 05 00 00 40 81 data_type=0x81
APP LiDAR: scan response type 0x81 detected
APP LiDAR: ready=1 rx=... type=0x81 valid=... nearest=... front=... front_wide=... left=... right=...

APP LIDAR_OBS: state=START_DELAY reason=init
APP LIDAR_OBS: state=WAIT_LIDAR reason=start_delay_done
APP LIDAR_OBS: state=DRIVE_FORWARD reason=lidar_ready
APP LIDAR_OBS: front_min_mm=xxx valid_points=xxx

APP LIDAR_OBS: obstacle detected front_min_mm=xxx valid_points=xxx
APP LIDAR_OBS: state=OBSTACLE_STOP reason=obstacle_detected
APP LIDAR_OBS: state=BACKUP reason=stop_done
APP LIDAR_OBS: state=TURN reason=backup_done
APP LIDAR_OBS: state=RECOVER reason=turn_done
APP LIDAR_OBS: recover forward
APP LIDAR_OBS: state=DRIVE_FORWARD reason=recover_forward
```

If the front sector temporarily has no valid point, the current code tolerates several invalid samples. If front data remains invalid or times out, the expected safety behavior is:

```text
APP LIDAR_OBS: lidar_no_valid_front_distance invalid_count=3 front_age_ms=...
APP LIDAR_OBS: state=WAIT_LIDAR reason=lidar_no_valid_front_distance
```

## Why The Final Version Uses LiDAR Instead Of Ultrasonic/Servo

The final version uses LiDAR because it matches both the actual hardware and the project theme.

Key reasons:

- The current project hardware does not include a servo.
- The current project hardware does not include an ultrasonic module.
- `ServoScanObstacle` depends on servo plus ultrasonic hardware.
- `NoServoObstacleAvoidance` depends on ultrasonic hardware.
- LiDAR is physically connected and already outputs valid scan data.
- LiDAR UART4 RX, descriptor parsing, scan response type `0x81`, and zone distance extraction have been verified.
- A LiDAR based demo better matches the AMR / intelligent path-following robot goal.

Conclusion:

- `LiDARObstacleAvoidance` is the final default mainline demo mode.
- Servo / ultrasonic code is retained only as optional expansion demo code or backup experiment code.
- The final report should not present servo or ultrasonic hardware as required for the current final demo.

## Test Method

### Build

Run from the project root:

```powershell
cmake --build --preset Debug --clean-first
```

Expected result:

- Build completes.
- `build\Debug\AMR_LiDAR_Robot.elf` is generated.

### Flash

Run from the project root:

```powershell
STM32_Programmer_CLI -c port=SWD -w build\Debug\AMR_LiDAR_Robot.elf -v -rst
```

Expected result:

- Programming succeeds.
- Verify succeeds.
- Target resets.

### Serial Observation

Check these items on the serial log:

- `active_mode=LiDARObstacleAvoidance`.
- `active_test=LiDARObstacleAvoidance`.
- `APP LiDAR: uart4 rx started, baud=460800`.
- `APP LiDAR: scan response type 0x81 detected`.
- `ready=1`.
- `valid_points` increases.
- `front_min_mm` appears and changes reasonably when objects move in front of the robot.
- The LiDAR obstacle state machine reaches `DRIVE_FORWARD`.

### Lifted-Wheel Test

Before ground testing, lift the wheels off the table or ground.

Check:

- Startup remains stopped during `START_DELAY`.
- The robot enters `WAIT_LIDAR` until LiDAR front data is valid.
- The robot enters `DRIVE_FORWARD` when front data is valid.
- Placing an obstacle in front of the LiDAR triggers `OBSTACLE_STOP`.
- The state machine proceeds through `BACKUP`, `TURN`, and `RECOVER`.

### Ground Test

Use a low-risk obstacle such as a cardboard box or backpack.

Recommended precautions:

- Do not use a wall for the first ground test.
- Keep a hand ready to disconnect power.
- Start with a short open area.
- Watch both the robot motion and the serial log.

Pass criteria:

- With no obstacle, the robot drives forward.
- A front obstacle around 30-40 cm can trigger stop behavior.
- The robot stops, backs up, turns, recovers, and resumes forward movement.
- The robot does not stay in `WAIT_LIDAR` for a long time when LiDAR scan data is healthy.

## Known Issues And Future Improvements

Known issues:

- `MOTOR setA/setB` logs are currently frequent and may hide key LiDAR / state-machine logs. Future work should add log rate limiting or duty-change-only printing.
- The LiDAR front sector can occasionally have no valid points. The current code adds `LIDAR_OBS_FRONT_INVALID_LIMIT=3` and `LIDAR_OBS_FRONT_TIMEOUT_MS=600` as a safety and jitter-reduction mechanism.
- After recovery, if `front_min_mm` is still below the stop threshold, the robot may trigger obstacle avoidance again. This is safe behavior, but it can be improved by tuning `LIDAR_OBS_TURN_MS` or using a better turn decision.
- The current final demo is reactive obstacle avoidance. It is not SLAM, global navigation, or full path planning.

Future improvements:

- Compare left and right LiDAR sectors and choose the more open turn direction.
- Use IMU yaw / gyro integration for fixed-angle turns.
- Add a simple local occupancy grid or local path scoring.
- Rate-limit motor logs.
- Move key demo parameters into a dedicated documented configuration block.
- Record a ground demo video or photo set for final presentation evidence.
- Add a concise test log archive after each stable demo run.

## Current Final Conclusion

The project has reached a stable final-demo stage based on `LiDARObstacleAvoidance`.

The main achievement is that the robot can use LiDAR front-sector distance to detect an obstacle, stop, backup, turn, recover, and continue driving without relying on unavailable servo or ultrasonic hardware.

This is a conservative and demonstrable AMR obstacle avoidance baseline. It is intentionally not documented as SLAM or full autonomous navigation. The next natural step is to improve the turn decision by using LiDAR left/right sector comparison and, later, IMU-assisted turn angle control.
