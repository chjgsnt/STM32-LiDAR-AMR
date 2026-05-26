# AMR LiDAR Robot

## Project Overview

This project is an STM32 LiDAR AMR / smart path-following robot demo. The current final default demo mode is `LiDARObstacleAvoidance`.

The robot uses LiDAR front-distance detection, a conservative control state machine, and left/right motor control to perform reactive obstacle avoidance. It is not documented as SLAM, mapping, global navigation, or full autonomous path planning.

## Current Hardware

- MCU: STM32F446xx
- RTOS: FreeRTOS
- Drive base: differential motor chassis with left/right motor control
- IMU: MPU6500 over I2C, address `0x68`, `WHO_AM_I=0x70`
- LiDAR: UART4, `baud=460800`
- Current final version does not depend on a servo
- Current final version does not depend on an ultrasonic module
- Digital Logic Analyzer is a debugging tool, not an onboard sensor or actuator

Servo and ultrasonic demo code may exist in the repository as optional expansion paths, but they are not required by the final default demo.

## Current Default Mode

Default runtime mode:

```text
active_mode=LiDARObstacleAvoidance
```

State machine:

```text
START_DELAY -> WAIT_LIDAR -> DRIVE_FORWARD -> OBSTACLE_STOP -> BACKUP -> TURN -> RECOVER
```

State meanings:

- `START_DELAY`: keep motors stopped during startup.
- `WAIT_LIDAR`: wait for valid LiDAR front-sector data.
- `DRIVE_FORWARD`: drive forward using calibrated left/right duty.
- `OBSTACLE_STOP`: stop when a front obstacle is detected.
- `BACKUP`: reverse briefly to create space.
- `TURN`: turn in place for a fixed time.
- `RECOVER`: pause/recover, then return to forward driving when LiDAR data is valid.

## Build And Flash

Build Debug firmware:

```powershell
cmake --build --preset Debug --clean-first
```

Flash with STM32 Programmer CLI:

```powershell
STM32_Programmer_CLI -c port=SWD -w build\Debug\AMR_LiDAR_Robot.elf -v -rst
```

## Expected Serial Logs

Representative logs for the final demo:

```text
[APP] active_mode=LiDARObstacleAvoidance
APP LiDAR: uart4 rx started, baud=460800
APP LiDAR: scan response type 0x81 detected
APP LIDAR_OBS: state=DRIVE_FORWARD reason=lidar_ready
APP LIDAR_OBS: front_min_mm=xxx valid_points=xxx
APP LIDAR_OBS: obstacle detected front_min_mm=xxx
APP LIDAR_OBS: state=OBSTACLE_STOP
APP LIDAR_OBS: state=BACKUP
APP LIDAR_OBS: state=TURN
APP LIDAR_OBS: state=RECOVER
```

## Documentation

See [Docs/Final_Report.md](Docs/Final_Report.md) for the full project report.
See [Docs/Demo_Guide.md](Docs/Demo_Guide.md) for the final offline demo procedure.

The final report records:

- hardware bring-up,
- LiDAR and IMU verification,
- representative serial logs,
- design tradeoffs,
- current parameters,
- known limitations,
- future improvement ideas.

## Current Demo Features

- LiDAR obstacle avoidance
- AMR state machine
- Differential-drive odometry pose and velocity telemetry
- OLED status UI
- USER BUTTON offline control
- Serial telemetry and commands
- Safety fault handling
- Minimal recorded-action return-to-start

## Odometry And Fault Handling

Odometry uses differential-drive kinematics from left/right encoder deltas to
estimate `x`, `y`, `theta`, linear velocity, and angular velocity. Calibration
constants are centralized in `Core/Inc/app_odometry.h`.

`app_fault` provides a latched Fault Manager for safe-stop behavior. LiDAR RX
timeout and encoder stall faults stop motor outputs and require an explicit
button or serial reset before motion can resume.

Scan matching, occupancy-grid mapping, and full planner recovery are not yet
implemented. They are future extensions on top of the current odometry pose and
LiDAR telemetry.

## Known Limitations

- Current avoidance is reactive LiDAR obstacle avoidance, not SLAM.
- `MOTOR setA/setB` logs are frequent and can hide key state-machine logs.
- The LiDAR front sector may occasionally have no valid point; the current demo includes invalid-count and timeout safety handling.
- If `front_min_mm` is still below the stop threshold after recovery, obstacle avoidance will trigger again. This is expected safe behavior.

## Future Work

- Compare left/right LiDAR sectors and choose the more open direction.
- Use IMU yaw / gyro data for fixed-angle turns.
- Rate-limit motor logs.
- Add ground demo video/photos and a formal acceptance test table.
