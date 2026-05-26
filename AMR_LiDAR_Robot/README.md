# AMR LiDAR Robot

STM32 NUCLEO-F446RE + FreeRTOS 2D LiDAR AMR project.

The final stable demo prioritises reliable LiDAR obstacle avoidance, safe stop,
serial telemetry, and engineering validation evidence. It does not claim full
autonomous Start-to-Exit-to-Return maze navigation.

## 1. Project Summary

The current final demo build runs:

```text
active_mode=LiDARObstacleAvoidance
```

The main demonstrated capability is LiDAR-based reactive obstacle avoidance:

```text
front detection -> stop/hold -> backup -> turn -> recover
```

The robot receives LiDAR data, extracts front distance, decides local avoidance
actions, drives the chassis through the existing motor layer, and reports state
through serial telemetry. Fault handling and USER_ESTOP are included to keep the
demo recoverable during live testing.

Odometry, mapping, and exploration modules exist in the codebase, but final demo
navigation does not depend on live odometry-map integration. Odometry pose
integration is frozen by default for final demo stability.

## 2. Final Demo Features

Enabled and demonstrated in the final demo:

- LiDAR front-distance detection through UART4.
- LiDAR obstacle avoidance with stop / backup / turn / recover behavior.
- AMR state machine with IDLE, EXPLORE, AVOID, RETURN, FAULT, and ESTOP states.
- Serial command control: `status`, `tel`, `start`, `stop`, `estop`,
  `reset_fault`, `enc_dbg`, `map`, and `exp`.
- PC13 long-press USER_ESTOP.
- PC13 long-press fault/ESTOP recovery: clear fault and reset odometry/map.
- Serial telemetry as the primary UI evidence.
- Fault manager and safe stop behavior.
- Encoder/odometry diagnostic framework through `enc_dbg`, `odom_dbg`, and
  `odo_freeze`.
- Encoder one-wheel revolution calibration recorded in documentation.

Implemented as diagnostic/framework components, but not used for final live
navigation:

- Differential-drive odometry pose integration.
- Lightweight 5x5 cell map.
- DFS/frontier-style exploration skeleton.
- Recorded-action return/path framework.

Disabled or not implemented in the final demo:

- OLED hardware display is disabled by default due to pin/resource constraints.
- Bluetooth is not implemented.
- Full SLAM, ICP, A*, DWA, and live Start-to-Exit-to-Return navigation are not
  claimed.

## 3. Hardware Used

- MCU board: STM32 NUCLEO-F446RE.
- RTOS: FreeRTOS.
- LiDAR: UART4 at 460800 baud.
- Debug serial: USART2 virtual COM port at 115200 baud, 8N1.
- Drive base: differential chassis with left and right motors.
- Encoders: left and right wheel encoder feedback.
- USER BUTTON: PC13 on the NUCLEO board.
- IMU support exists in the project, but the final demo is LiDAR-obstacle focused.
- OLED/SSD1306 hardware display is disabled by default.
- Bluetooth hardware/software path is not implemented.

Servo and ultrasonic demo code may exist in the repository as optional expansion
paths, but they are not required by the final default demo.

## 4. How to Build

Build the Debug firmware from the project root:

```powershell
cmake --build --preset Debug --clean-first
```

Expected output:

```text
build\Debug\AMR_LiDAR_Robot.elf
```

The project uses the STM32CubeCLT / CMake / Ninja / `arm-none-eabi` toolchain
flow.

## 5. How to Flash / Run

Flash with STM32 Programmer CLI:

```powershell
STM32_Programmer_CLI -c port=SWD -w build\Debug\AMR_LiDAR_Robot.elf -v -rst
```

Open the debug COM port:

```text
115200 baud, 8N1
```

After reset, check that the serial log reports the active mode as
`LiDARObstacleAvoidance`, and run:

```text
status
tel
```

The final demo should show `odo_frozen=1`.

## 6. Serial Commands

Core commands:

| Command | Purpose |
| --- | --- |
| `status` | Print detailed system, LiDAR, motor, odometry, map, and explorer status |
| `tel` | Print one compact telemetry line |
| `start` | Enter LiDAR obstacle-avoidance mode |
| `stop` | Stop the chassis and return to IDLE |
| `estop` | Enter ESTOP and disable motor outputs |
| `reset_fault` | Clear FAULT/ESTOP when safe |

Diagnostics and framework commands:

| Command | Purpose |
| --- | --- |
| `enc_dbg` | Print encoder counts, signed deltas, wheel-distance estimates, and calibration values |
| `odom_dbg` | Print odometry diagnostic information |
| `odo_freeze 1` | Freeze odometry pose integration |
| `odo_freeze 0` | Temporarily enable odometry pose integration for experiments |
| `odom_reset` / `odo_reset` | Reset odometry and map state |
| `map` / `grid` | Print the 5x5 software map view |
| `map_reset` | Reset the software map |
| `exp` | Print explorer framework status |
| `explore` | Start explorer framework state |
| `return` | Start return/path framework state |
| `ui` | Print serial UI status |
| `page 0/1/2` | Switch serial UI page state |

## 7. Demo Procedure

1. Build and flash the firmware.
2. Open USART2 serial at 115200 baud, 8N1.
3. Send `status`.
4. Confirm:
   - `state=IDLE`
   - `fault=NONE`
   - `lidar ready=1`
   - front distance is valid
   - `odo_frozen=1`
   - motor PWM is zero
5. Place the robot on the floor with enough space for backup and turn.
6. Send `start`.
7. Place an obstacle in front of the LiDAR.
8. Observe obstacle response:
   - front obstacle detection
   - stop/hold reason
   - backup
   - turn
   - recover
9. Send `enc_dbg` to show encoder calibration/diagnostic evidence.
10. Send `tel` to show compact telemetry.
11. Send `stop` and confirm the robot returns to IDLE and PWM is zero.
12. Long-press PC13 for about 2 seconds to trigger USER_ESTOP.
13. In FAULT/ESTOP, long-press PC13 again to clear fault and reset odometry/map.
14. Send `status` again to confirm safe IDLE state.

Expected evidence includes logs such as:

```text
[APP] active_mode=LiDARObstacleAvoidance
ODO: init ... ticks_per_rev=1694 ... freeze=1
[STATUS] state=IDLE fault=NONE
[STATUS] lidar ready=1 front=...
[STATUS] pose ... odo_frozen=1
TEL: mode=IDLE fault=NONE odo_frozen=1 ...
[CMD_RX] line=start
[AMR] state IDLE -> EXPLORE reason=serial_start
[OBS] hold reason=front_blocked front=... stop=...
APP LIDAR_OBS: state=BACKUP reason=obstacle_detected
APP LIDAR_OBS: state=TURN reason=backup_done ...
[CMD_RX] line=enc_dbg
ODO_DBG: frozen=1 cntL=... cntR=...
[BTN] action=estop
FAULT: code=USER_ESTOP
FAULT: safe stop
```

## 8. Documentation Links

- [Docs/Demo_Guide.md](Docs/Demo_Guide.md): final stable demo procedure.
- [Docs/Final_Report.md](Docs/Final_Report.md): final technical documentation draft.

## 9. Current Limitations

- Final demo is reactive LiDAR obstacle avoidance, not full autonomous maze
  solving.
- Live odometry integration is frozen by default:
  `APP_ODO_FREEZE_DEFAULT=1`.
- `status` and `tel` should show `odo_frozen=1` in the final demo.
- Map/explorer modules are software framework/skeleton components, not final
  live Start-to-Exit-to-Return navigation.
- OLED display is disabled by default due to pin/resource constraints.
- Bluetooth is not implemented.
- Software ESTOP is implemented, but it is not a hardware latched power cut.
- Full scan matching, ICP, A*, DWA, and robust global navigation remain future
  work.

## 10. Engineering Notes

Odometry and encoder calibration:

- Encoder diagnostic showed left forward tick sign should be positive.
- Odometry uses `ODOM_LEFT_TICK_SIGN=+1` and `ODOM_RIGHT_TICK_SIGN=+1`.
- One-wheel revolution calibration measured:
  - left wheel approximately 1738 ticks/rev,
  - right wheel approximately 1650 ticks/rev,
  - average approximately 1694 ticks/rev.
- Final odometry parameter:
  `ODO_ENCODER_TICKS_PER_REV=1694`.
- `ODO_GEAR_RATIO` remains `1.0`.

Final odometry trade-off:

- Ground testing showed left/right travel mismatch and pose instability during
  live motion.
- To keep the final demo stable, odometry integration is frozen by default.
- Encoder sampling and diagnostics still run while frozen.
- `odo_freeze 0` can temporarily enable pose integration for controlled
  experiments.
- While odometry is frozen, map updates are skipped to avoid corrupting the 5x5
  software map.

UI and telemetry:

- Serial telemetry is the primary UI evidence path.
- OLED support is kept out of the final enabled demo because hardware
  resources are prioritised for LiDAR, encoders, motor control, fault handling,
  and benchmark stability.

Safety:

- Fault handling is latched and requires explicit recovery.
- LiDAR timeout and encoder stall checks are part of the safety layer.
- PC13 long press triggers USER_ESTOP.
- PC13 long press in FAULT/ESTOP clears fault and resets odometry/map.

