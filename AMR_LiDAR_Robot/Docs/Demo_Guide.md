# AMR LiDAR Robot Demo Guide

This guide describes the final stable demo build for the STM32 NUCLEO-F446RE
LiDAR AMR. The demo focuses on the embedded Sense-Think-Act pipeline: LiDAR
front-distance sensing, local obstacle avoidance, closed-loop motor actuation,
fault handling, safe stop, serial telemetry, and encoder calibration evidence.

The final demo does not claim full autonomous Start-to-Exit-to-Return benchmark
completion. The map and explorer modules are retained as software framework
and diagnostic groundwork, but live navigation is not driven by odometry mapping
in the stable demo build.

## 1. Hardware Setup

- MCU board: STM32 NUCLEO-F446RE.
- Active application mode: `LiDARObstacleAvoidance`.
- 2D LiDAR: connected to UART4 at 460800 baud.
- PC debug serial: USART2 virtual COM port, 115200 baud, 8N1.
- Differential-drive chassis with left and right wheel encoders.
- Motor PWM and encoder capture are configured by the existing STM32Cube setup.
- USER BUTTON: PC13 on the NUCLEO board.
- OLED/SSD1306 display: disabled by default because of pin/resource constraints.
- UI evidence is provided through compact serial telemetry instead of OLED.

## 2. Serial Command List

Core demo commands:

- `status`: print system, LiDAR, motor, odometry, path, map, and explorer status.
- `tel`: print one compact telemetry line.
- `start`: enter exploration/obstacle-avoidance mode.
- `stop`: stop the chassis and return to IDLE.
- `estop`: enter ESTOP and disable motor outputs.
- `reset_fault`: clear FAULT/ESTOP state when safe.

Odometry and encoder diagnostics:

- `enc_dbg`: print encoder counts, deltas, wheel-distance estimates, and odometry calibration parameters.
- `odom_dbg`: same diagnostic path as `enc_dbg`.
- `odom_reset` or `odo_reset`: reset odometry pose and map state.
- `odo_freeze 1`: freeze odometry pose integration.
- `odo_freeze 0`: temporarily enable odometry pose integration for experiments.

Map/explorer framework commands:

- `map` or `grid`: print the 5x5 software map view.
- `map_reset`: clear the software map.
- `exp`: print explorer framework status.
- `explore`: start the explorer software framework.
- `return`: start the recorded-action/software return framework.

UI fallback commands:

- `ui`: print current serial UI status.
- `page 0`, `page 1`, `page 2`: switch serial UI page state.

Experimental benchmark script commands:

- `script_exit`: start a short time-based Start-to-Exit script.
- `script_return`: start a short time-based Exit-to-Start script.
- `script_stop`: stop the script and safe stop.
- `script_status`: print script state, step index, action, elapsed, and remaining time.
- `script_reset`: reset script state.

The benchmark script mode is experimental and time-based. It is provided to help
tune a fixed 5x5 maze run, but it is not claimed as full autonomous SLAM,
mapping, or Start-to-Exit-to-Return navigation.

## 3. Pre-Demo Checklist

1. Build and flash the final `Debug` firmware.
2. Open the USART2 COM port at 115200 baud, 8N1.
3. Confirm boot logs show the active mode as LiDAR obstacle avoidance.
4. Run `status` and check:
   - `state=IDLE`
   - `lidar ready=1`
   - front distance is valid and plausible
   - `odo_frozen=1`
   - motor PWM is zero
   - fault is `NONE`
5. Run `tel` and confirm compact telemetry prints `odo_frozen=1`.
6. Keep the robot on the floor with enough space for backup/turn/recover.
7. Keep one hand near PC13 for long-press ESTOP.

## 4. Final Demo Sequence

1. Power on the robot and wait for stable LiDAR data.
2. Send `status`.
3. Point out that odometry integration is frozen by default:
   - `APP_ODO_FREEZE_DEFAULT=1`
   - `status`/`tel` should show `odo_frozen=1`
   - encoder diagnostics still work through `enc_dbg` / `odom_dbg`
4. Place an obstacle in front of the LiDAR and observe the front-distance value.
5. Send `start`.
6. Demonstrate obstacle avoidance:
   - front obstacle detection
   - stop/hold reason when blocked
   - backup
   - turn
   - recover/drive-forward behavior when clear
7. Send `enc_dbg` to show calibrated encoder evidence while pose integration remains frozen.
8. Send `tel` to show concise system telemetry.
9. Send `stop` and confirm motor PWM returns to zero and state returns to IDLE.
10. Long-press PC13 for about 2 seconds to trigger USER_ESTOP.
11. Confirm ESTOP safe stop in logs.
12. While in ESTOP or FAULT, long-press PC13 again to clear fault and reset odometry/map.
13. Send `status` to confirm the system is back in a safe IDLE state.

## 5. Expected Log Evidence

Typical serial evidence:

```text
[APP] active_mode=LiDARObstacleAvoidance
ODO: init ... ticks_per_rev=1694 ... freeze=1
[STATUS] state=IDLE fault=NONE
[STATUS] lidar ready=1 front=...
[STATUS] pose ... odo_frozen=1
TEL: mode=IDLE fault=NONE odo_frozen=1 ...
[CMD_RX] line=start
[AMR] state IDLE -> EXPLORE reason=serial_start
APP LIDAR_OBS: start called ...
[OBS] hold reason=front_blocked front=... stop=...
APP LIDAR_OBS: state=BACKUP reason=obstacle_detected
APP LIDAR_OBS: state=TURN reason=backup_done ...
APP LIDAR_OBS: state=RECOVER ...
[CMD_RX] line=enc_dbg
ODO_DBG: frozen=1 cntL=... cntR=...
ODO_DBG: total rawL=... rawR=... signedL=... signedR=...
[CMD_RX] line=stop
[AMR] state EXPLORE -> IDLE reason=serial_stop
[BTN] long_press
[BTN] action=estop
FAULT: code=USER_ESTOP
FAULT: safe stop
[ESTOP] motor outputs disabled
[BTN] action=clear_fault_reset_odom
FAULT: cleared
[ODOM] reset
```

## 6. Recovery Procedure

If the robot is moving unexpectedly:

1. Long-press PC13 for about 2 seconds.
2. Confirm logs show USER_ESTOP / ESTOP and motor outputs disabled.
3. Move the robot to a safe position.
4. Long-press PC13 again while in ESTOP/FAULT to clear fault and reset odometry/map.
5. Send `status` and confirm:
   - `state=IDLE`
   - `fault=NONE`
   - PWM is zero
   - `odo_frozen=1`
6. If serial control is available, `estop`, `stop`, and `reset_fault` can also be used.

If LiDAR is not ready:

1. Do not send `start`.
2. Check LiDAR power and UART4 wiring.
3. Confirm `APP LiDAR` or `status` shows receive activity and valid front distance.
4. Power-cycle only after the chassis is physically safe.

If encoder/odometry values look unreasonable:

1. Keep `odo_frozen=1` for the final demo.
2. Use `enc_dbg` / `odom_dbg` for evidence and calibration diagnostics.
3. Use `odo_freeze 0` only for controlled experiments, then restore `odo_freeze 1`.

## 7. Known Limitations

- Odometry pose integration is frozen by default for the final demo to avoid wheel-slip and left/right distance mismatch corrupting live pose/map state.
- Encoder calibration is available and one-revolution tick calibration has been performed, but live pose navigation is not used for final demo decisions.
- OLED output is disabled because of pin/resource constraints; serial telemetry is the UI evidence path.
- The 5x5 map and explorer modules are software frameworks/skeletons for future work.
- `script_exit` and `script_return` are experimental time-based benchmark helpers and must not be presented as full autonomous navigation.
- The final demo does not perform full SLAM, ICP, A*, DWA, or full autonomous Start-to-Exit-to-Return benchmark navigation.
- Return-to-start/path features are prototype support logic and should not be presented as a completed benchmark return solution.
- Obstacle avoidance performance depends on floor friction, wheel slip, obstacle placement, and LiDAR visibility.

## 8. Experimental Benchmark Script Mode

The experimental script mode is intended for controlled 5x5 maze tuning on the
`exp/benchmark-script` branch. It does not replace the stable `start` obstacle
avoidance command.

Default `script_exit` steps:

1. `FORWARD`, duty `520`, duration `900 ms`.
2. `TURN_RIGHT`, duty `420`, duration `550 ms`.
3. `FORWARD`, duty `520`, duration `900 ms`.
4. `STOP`, duration `200 ms`.

Default `script_return` steps:

1. `TURN_RIGHT`, duty `420`, duration `1750 ms`.
2. `FORWARD`, duty `520`, duration `900 ms`.
3. `TURN_LEFT`, duty `420`, duration `550 ms`.
4. `FORWARD`, duty `520`, duration `900 ms`.
5. `STOP`, duration `200 ms`.

PC13 button behavior on the `exp/benchmark-script` branch:

- Short press in IDLE alternates `script_exit` and `script_return`.
- Short press while a script is running stops the script and safe-stops.
- Short press in non-IDLE non-script states requests stop.
- Short press in FAULT/ESTOP is ignored; use long press to recover.
- Long press while running still triggers USER_ESTOP.
- Long press in FAULT/ESTOP clears fault and resets odometry/map.

Safety behavior:

- Scripts only start from `AMR_STATE_IDLE` with no active fault.
- Forward steps stop and wait if LiDAR front distance is below `250 mm`.
- Waiting steps resume by advancing to the next script step after the front
  distance is above `450 mm`.
- Fault, ESTOP, or `script_stop` stops the script immediately.
- Use `script_status` to inspect state and timing.
