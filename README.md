# AMR LiDAR Robot - EBU6475 Microprocessor Systems Design

## 1. Project Summary

This repository contains an STM32 NUCLEO-F446RE + FreeRTOS 2D LiDAR AMR project
for EBU6475 Microprocessor Systems Design.

The main project folder is:

```text
AMR_LiDAR_Robot/
```

The final stable demo prioritises reliable LiDAR obstacle avoidance, safe stop,
and serial telemetry. It is a stable LiDAR obstacle avoidance and safety demo,
not a full autonomous Start-to-Exit-to-Return benchmark solution.

## 2. Final Stable Demo Status

Final demo configuration:

- Active mode: `LiDARObstacleAvoidance`
- Main behavior: LiDAR-based obstacle avoidance with stop / backup / turn /
  recover actions
- UI evidence: serial telemetry through USART2
- OLED hardware display: disabled by default due to pin/resource constraints
- Bluetooth: not implemented
- Odometry integration: frozen by default for final demo stability
- Odometry macro: `APP_ODO_FREEZE_DEFAULT=1`
- Telemetry/status should show: `odo_frozen=1`
- Encoder diagnostics remain available through `enc_dbg`, `odom_dbg`, and
  `odo_freeze`

Encoder calibration result:

| Wheel test | Result |
| --- | ---: |
| Left wheel one revolution | approximately 1738 ticks |
| Right wheel one revolution | approximately 1650 ticks |
| Average calibration value | approximately 1694 ticks/rev |
| Final odometry setting | `ODO_ENCODER_TICKS_PER_REV=1694` |

The lightweight 5x5 map and exploration framework exist as software
framework/skeleton components. They are not used as final live navigation, and
the final demo does not claim full live Start-to-Exit-to-Return autonomous
benchmark navigation.

## 3. Key Features

Enabled in the final demo:

- LiDAR front-distance detection through UART4.
- Obstacle avoidance: stop / backup / turn / recover.
- AMR state machine with safe IDLE, EXPLORE/AVOID, FAULT, and ESTOP behavior.
- Fault manager and safe stop.
- Serial telemetry and command interface.
- PC13 long-press USER_ESTOP.
- PC13 long-press in FAULT/ESTOP clears fault and resets odometry/map.
- Encoder diagnostic and calibration evidence through serial commands.

Implemented as framework or diagnostics:

- Differential-drive odometry module.
- Encoder/odometry diagnostic framework.
- 5x5 cell-level map module.
- DFS/frontier-style exploration skeleton.
- Serial UI fallback and telemetry pages.

Not implemented or not enabled in the final demo:

- Bluetooth telemetry/control.
- OLED hardware display.
- Full SLAM / ICP.
- Full A* / DWA autonomous maze navigation.
- Full live Start-to-Exit-to-Return benchmark autonomy.

## 4. Repository Structure

```text
.
├── AMR_LiDAR_Robot/              # Main STM32Cube/CMake firmware project
│   ├── Core/                     # STM32 application, HAL glue, FreeRTOS tasks
│   ├── Drivers/                  # STM32 HAL drivers
│   ├── Middlewares/              # FreeRTOS and middleware
│   ├── Docs/                     # Demo guide and final report
│   └── README.md                 # Detailed project README
├── Docs/                         # Earlier evidence/progress folders
├── Firmware/                     # Early firmware planning/placeholders
├── Tests/                        # Test placeholders
├── Tools/                        # Host-side tools/placeholders
└── README.md                     # GitHub repository landing page
```

Most current code and documentation live under `AMR_LiDAR_Robot/`.

## 5. Quick Demo Commands

Open the USART2 debug COM port at 115200 baud, 8N1.

Core commands:

| Command | Purpose |
| --- | --- |
| `status` | Print detailed system state |
| `tel` | Print compact telemetry |
| `start` | Start LiDAR obstacle avoidance |
| `stop` | Stop and return to IDLE |
| `estop` | Enter ESTOP and safe stop |
| `reset_fault` | Clear FAULT/ESTOP when safe |

Diagnostics/framework commands:

| Command | Purpose |
| --- | --- |
| `enc_dbg` | Print encoder diagnostic/calibration data |
| `odom_dbg` | Print odometry diagnostic data |
| `odo_freeze 1` | Freeze odometry integration |
| `odo_freeze 0` | Temporarily enable odometry integration for experiments |
| `map` / `grid` | Print software map view |
| `exp` | Print explorer framework status |
| `script_exit` | Start experimental time-based Start-to-Exit script |
| `script_return` | Start experimental time-based Exit-to-Start script |
| `script_auto` | Start experimental reactive maze mode |
| `script_return_auto` | Start experimental reactive return attempt |
| `script_stop` | Stop experimental benchmark script and safe stop |
| `script_auto_stop` | Alias for stopping the active experimental script |
| `script_status` | Print experimental script state |

Typical demo flow:

```text
status
tel
start
enc_dbg
tel
stop
```

PC13 user button:

- Long press: trigger `USER_ESTOP`.
- Long press again while in FAULT/ESTOP: clear fault and reset odometry/map.
- On `exp/benchmark-script`, short press 1 starts `script_auto`; short press 2
  stops auto and arms return; short press 3 starts `script_return_auto`; short
  press 4 stops return and resets the button flow to auto.
- Exit detection is manual by button press. `script_return_auto` starts with a
  timed 180-degree turn-around, then continues reactive return exploration; it
  is not guaranteed map-based return-to-start.

## 6. Documentation Links

- [Detailed project README](AMR_LiDAR_Robot/README.md)
- [Demo Guide](AMR_LiDAR_Robot/Docs/Demo_Guide.md)
- [Final Technical Report Draft](AMR_LiDAR_Robot/Docs/Final_Report.md)

## 7. Current Limitations

- The final demo is reactive LiDAR obstacle avoidance, not full benchmark
  autonomy.
- Live odometry integration is frozen by default:
  `APP_ODO_FREEZE_DEFAULT=1`.
- The map/explorer modules are software framework/skeleton components, not
  verified live navigation.
- Experimental `script_exit` / `script_return` commands are time-based benchmark
  helpers, and `script_auto` is a simple reactive exploration fallback; they
  are not full autonomous navigation, SLAM, or A*.
- OLED hardware display is disabled by default due to pin/resource constraints.
- Bluetooth is not implemented.
- The software ESTOP path is implemented, but it is not a hardware latched
  power cut.
- Full scan matching, ICP, A*, DWA, and robust global maze navigation remain
  future work.
