# Obstacle Avoidance Status

Date: 2026-05-22

Target: STM32 NUCLEO-F446RE / STM32F446RETx

Scope:

- Record the current LiDAR obstacle avoidance integration status.
- Record the unified `APP_ACTIVE_MODE` runtime selector.
- Record the current corner escape and IMU heading assist dry-run behavior.
- Keep the default safety rule visible: default firmware must not drive the wheels.

## Current Run Modes

Runtime mode selection is centralized in:

```text
AMR_LiDAR_Robot/Core/Inc/app_test_config.h
```

The current mode IDs are:

```c
#define APP_MODE_LIDAR_OBSTACLE_DRY_RUN     0
#define APP_MODE_LIDAR_OBSTACLE_GROUND_TEST 1
#define APP_MODE_MOTOR_TEST                 2
#define APP_MODE_IMU_TEST                   3
#define APP_MODE_SENSOR_BRINGUP             4
#define APP_MODE_IMU_HEADING_ASSIST_DRY_RUN 5
```

The three modes relevant to obstacle avoidance are:

| Mode | Purpose | Wheel output |
| --- | --- | --- |
| `APP_MODE_LIDAR_OBSTACLE_DRY_RUN` | Run LiDAR obstacle decision and motor-command logging without wheel output. | Disabled, `enabled=0`, `ground=0`. |
| `APP_MODE_LIDAR_OBSTACLE_GROUND_TEST` | Supervised short ground test for the LiDAR obstacle state machine. | Enabled, `enabled=1`, `ground=1`. |
| `APP_MODE_IMU_HEADING_ASSIST_DRY_RUN` | Fixed `FORWARD_SLOW` heading assist test that logs target/current/error/correction. | Disabled by default, `enabled=0`, `ground=0`. |

The default is:

```c
#define APP_ACTIVE_MODE APP_MODE_LIDAR_OBSTACLE_DRY_RUN
```

## How to Switch Modes

Change only `APP_ACTIVE_MODE` in `app_test_config.h`, or override it from the build system.

Examples:

```c
#define APP_ACTIVE_MODE APP_MODE_LIDAR_OBSTACLE_DRY_RUN
```

```c
#define APP_ACTIVE_MODE APP_MODE_LIDAR_OBSTACLE_GROUND_TEST
```

```c
#define APP_ACTIVE_MODE APP_MODE_IMU_HEADING_ASSIST_DRY_RUN
```

Do not directly edit legacy lower-level switches such as `APP_OBSTACLE_MOTOR_ENABLE` or `APP_OBSTACLE_GROUND_TEST_ENABLE`. They are derived from `APP_ACTIVE_MODE`.

## Default Safety Strategy

Default firmware is intentionally safe:

- `APP_ACTIVE_MODE` defaults to `APP_MODE_LIDAR_OBSTACLE_DRY_RUN`.
- `APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR` defaults to `0`.
- `APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE` defaults to `0`.
- The default obstacle mode logs decisions and preview motor commands, but does not drive the wheels.

Only `APP_MODE_LIDAR_OBSTACLE_GROUND_TEST` should allow ground-test motor output. The IMU heading assist apply path must first be checked with lifted wheels before considering any ground use.

## LiDAR Obstacle State Machine

The LiDAR obstacle motor layer now contains these ground states:

- `FORWARD`
- `CAUTION`
- `TURN_LEFT`
- `TURN_RIGHT`
- `BACKUP`
- `BLOCKED`
- `CORNER_BACKUP`
- `CORNER_TURN`
- `STOP`

The state machine uses the obstacle decision and LiDAR distance summaries to choose motion. Recent additions include:

- `front_wide`: a wider front obstacle check to reduce narrow-angle blind decisions.
- `obs_front`: selected obstacle-front distance used by the ground state machine.
- front clear/block history to avoid reacting to one noisy sample.
- turn hold and turn switch margin to reduce rapid left/right oscillation.
- near-wall backup and escape behavior.
- escape lock, timeout, and reselect logic.
- corner escape recovery with backup and turn phases.

The dry-run path prints the selected action and command preview. The ground-test path applies the command only under the explicit ground-test mode.

## Corner Escape

Corner escape is intended to recover when the robot repeatedly approaches a near front obstacle or gets trapped in a local turn/backup pattern.

At a high level:

- detect repeated close-front or backup recovery behavior.
- enter `CORNER_BACKUP`.
- then enter `CORNER_TURN` toward the more open side when possible.
- apply a cooldown so the robot does not immediately re-enter the same recovery path.

Current limitation: wall corners can still cause the robot to spin or turn in place. The recovery reduces some stuck cases, but it is not a complete local-planning solution yet.

## IMU Heading Assist Dry-Run

`APP_MODE_IMU_HEADING_ASSIST_DRY_RUN` is a dedicated heading correction verification mode.

Current behavior:

- fixed action: `FORWARD_SLOW`
- default motor output: disabled
- `ground=0`
- captures target yaw from the current integrated gyro-Z heading
- logs target/current/error/correction
- correction is limited by `APP_IMU_HEADING_CORRECTION_MAX`
- `APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR` defaults to `0`

Representative logs:

```text
APP IMU HEADING TEST: lifted_wheel=0 apply=0
APP IMU HEADING TEST: target=... current=... error=... corr=...
APP OBS MOTOR: enabled=0 ground=0 heading_corr=... heading_apply=0 left=480 right=520 ... dry-run
```

With `apply=0`, correction is printed only and is not applied to the left/right motor command.

## Lifted-Wheel Apply Check

Before any ground use of IMU heading correction, use a lifted-wheel test.

Temporary test configuration:

```c
#define APP_ACTIVE_MODE APP_MODE_IMU_HEADING_ASSIST_DRY_RUN
#define APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE 1
#define APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR 1
```

Expected behavior:

- `enabled=1`
- `ground=0`
- logs include `lifted-wheel-test`
- `FORWARD_SLOW` base command is `left=480`, `right=520`
- applied command uses `left = left - corr`, `right = right + corr`
- command is clamped by the existing motor duty limit

Representative log:

```text
APP IMU HEADING TEST: lifted_wheel=1 apply=1
APP OBS MOTOR: enabled=1 ground=0 heading_corr=... heading_apply=1 left=... right=... ... lifted-wheel-test
```

After the test, restore the default safe configuration before committing or continuing normal work.

## Verified Results

Current verified status:

- `APP_ACTIVE_MODE` centralizes runtime/test mode selection.
- Default mode is `APP_MODE_LIDAR_OBSTACLE_DRY_RUN`.
- Default configuration does not drive the wheels.
- `APP_MODE_LIDAR_OBSTACLE_GROUND_TEST` is available for supervised short ground tests.
- `APP_MODE_IMU_HEADING_ASSIST_DRY_RUN` is available for fixed `FORWARD_SLOW` heading correction logs.
- LiDAR obstacle logs include state, reason, decision, distances, action, command, and heading correction fields.
- `front_wide` and `obs_front` have been added to the obstacle decision context.
- escape lock, timeout, and reselect behavior have been added.
- corner escape recovery has been added.
- IMU heading assist dry-run reports changing target/current/error/correction when the robot body is manually rotated.
- heading correction is clamped to the configured limit.

## Known Limitations

- Wall corners can still cause spinning or repeated in-place turning.
- IMU heading assist defaults to log-only mode and does not apply correction to the motors.
- `APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR=1` should not be used for ground driving until the lifted-wheel test verifies the correction sign and left/right output behavior.
- Gyro-only heading is relative and can drift over time.
- The current obstacle avoidance is reactive; it is not a full mapper or path planner.

## Common Test Commands

Build:

```powershell
cd "C:\Users\29525\Desktop\CodingDay\Microprocessor Systems Design\AMR_LiDAR_Robot"
cmake --preset Debug
cmake --build --preset Debug
```

Program through ST-LINK:

```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "build\Debug\AMR_LiDAR_Robot.elf" -rst
```

Check active mode in source:

```powershell
findstr /n "APP_ACTIVE_MODE APP_MODE_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR" "Core\Inc\app_test_config.h"
```

Check active mode from the repository root:

```powershell
findstr /n "APP_ACTIVE_MODE APP_MODE_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR" "AMR_LiDAR_Robot\Core\Inc\app_test_config.h"
```

## Safety Notes

- Keep wheels lifted for any first test after changing motor-output-related macros.
- Do not commit a ground-test active mode as the default.
- Do not enable IMU heading assist apply for ground driving until lifted-wheel logs confirm the correction direction.
- Keep LiDAR UART4 and USART1 debug logging assignments unchanged during obstacle testing.
