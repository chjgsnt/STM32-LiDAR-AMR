# Fully Embedded 2D-LiDAR Autonomous Mobile Robot

## Hardware Platform

- MCU board: STM32 NUCLEO-F446RE
- Sensor target: RPLidar C1 2D LiDAR
- IMU target: MPU6500
- Display target: SSD1306 OLED
- Actuation target: DC motors with encoders

## Development Toolchain

- STM32CubeMX
- VSCode with STM32CubeIDE extension
- STM32CubeCLT

## Project Progress / Bring-Up Status

- Step 0 created the baseline project file structure for later firmware, documentation, tools, and tests.
- Step 1 created the first STM32CubeMX configuration for core bring-up peripherals and middleware.
- Step 2 verified the generated configuration on hardware with USART2 logging, FreeRTOS startup, I2C1 scanning, and basic I2C module bring-up.
- MPU6500 IMU bring-up is verified on I2C1 at address `0x68`: `WHO_AM_I = 0x70`, raw accelerometer/gyroscope data, `g` / `dps` conversion, gyro bias calibration, pitch/roll, complementary-filter fused pitch/roll, and App-layer readout are working.
- Step 3 verified the AT8236 motor driver path, Motor A/B PWM forward/reverse output, and TIM2/TIM4 encoder counting.
- Step 4 Chassis Open-Loop Test is complete. The chassis open-loop path has been verified with `MPU6500 IMU ready=1`, AT8236 motor driver output, Motor A/B PWM forward/reverse, TIM2/TIM4 encoder counting, and chassis `Forward`, `Backward`, `TurnLeft`, and `TurnRight` actions.
- Step 5 Encoder Speed Balance Test is complete. With `base_duty=500`, `CHASSIS_LEFT_TRIM=-10`, and `CHASSIS_RIGHT_TRIM=10`, P-only balance reduces left/right wheel speed difference. During the second half of forward motion, `err` can usually be reduced to about `+/-10 ticks/sample`. Backward motion can run, but `err` fluctuates slightly more and will be optimized in the later PID stage. `MPU6500 IMU ready=1` remained valid during the test.
- Final chassis direction configuration: `CHASSIS_LEFT_SIGN = -1`, `CHASSIS_RIGHT_SIGN = 1`.
- Current speed balance test duty: `500`.
- Current known issue: UART logs may interleave when IMU and BALANCE logs print at the same time.

## Planned Firmware Modules

- LiDAR data reception and RPLidar C1 packet parsing
- MPU6500 IMU interface
- SSD1306 OLED display interface
- Motor control
- Encoder feedback
- PID control
- Occupancy grid mapping
- Navigation and path planning
- UART debug logging
- FreeRTOS application tasks
- Application configuration

## Evidence Recording

Use the `Docs/` folders to keep project evidence organized:

- `Docs/01_Module_Test/`: module bring-up notes, including MPU6500 IMU validation
- `Docs/Progress_Report.md`: project bring-up progress report
- `Docs/02_Debug_Logs/`: serial logs and runtime traces
- `Docs/03_OLED_Photos/`: OLED display photos during tests
- `Docs/04_Logic_Analyzer/`: UART, I2C, PWM, and encoder captures
- `Docs/05_LiDAR_Raw_Data/`: raw LiDAR frames and parsing notes
- `Docs/06_PID_Tuning/`: PID tuning tables and test observations
- `Docs/07_Map_Output/`: occupancy grid snapshots and exported map data
- `Docs/08_Benchmark/`: timing, CPU load, memory, and latency measurements
- `Docs/09_Git_Records/`: milestone notes and change records

## Directory Overview

- `Firmware/App/`: application-level entry points and FreeRTOS task placeholders
- `Firmware/Drivers_User/`: user-maintained hardware driver modules
- `Firmware/Middleware_User/`: user-maintained control, mapping, navigation, and utility modules
- `Firmware/Config/`: application configuration headers
- `Docs/`: design notes, test evidence, debug records, and experiment outputs
- `Tools/`: host-side scripts and serial helper tools
- `Tests/`: unit and hardware test placeholders

## Next Step

Proceed to closed-loop wheel speed PID, or clean up UART logging with a log mutex / serialized logging path.
