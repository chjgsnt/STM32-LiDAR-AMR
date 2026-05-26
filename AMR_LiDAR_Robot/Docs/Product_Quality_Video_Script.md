# Product Quality Video Script

## EBU6475 Microprocessor Systems Design - AMR LiDAR Robot

Target length: maximum 7 minutes.

Final demo message: this is a stable LiDAR obstacle avoidance, safe-stop, and
serial telemetry demo. It is not presented as full autonomous
Start-to-Exit-to-Return benchmark navigation.

## Video Timeline Plan

| Time | Screen / Camera Content | Narration Points | Serial Command(s) | Expected Logs / Evidence |
| --- | --- | --- | --- | --- |
| 0:00-1:00 | Team title slide, robot on bench/floor, close-up of NUCLEO board and LiDAR | Introduce project goal: STM32 FreeRTOS AMR with embedded Sense-Think-Act pipeline. Introduce team responsibilities with placeholders: `[Member A]` firmware/FreeRTOS, `[Member B]` LiDAR and safety, `[Member C]` motor/encoder validation, `[Member D]` documentation/video. State this video shows the final stable demo build. | None | None required |
| 1:00-2:00 | Architecture slide or code/file tree. Show LiDAR, encoders, PC13 button, motor driver | Explain Sense -> Think -> Act. Sense: 2D LiDAR, wheel encoders, IMU status. Think: obstacle logic, fault manager, odometry diagnostics, map/explorer framework. Act: chassis commands, motor control, safe stop. Mention FreeRTOS task separation and command processing outside ISR. | Optional: `status` | `[STATUS] state=IDLE fault=NONE`, `lidar ready=1`, `odo_frozen=1` |
| 2:00-3:30 | Live robot demo. Show serial terminal and robot. Place obstacle in front. | Show final active mode is `LiDARObstacleAvoidance`. Show serial `status`/`tel`. Send `start`. Demonstrate LiDAR front distance causing stop/backup/turn/recover. Send `stop`. Explain `odo_frozen=1` is intentional for stable final demo because live odometry was unstable with wheel mismatch/slip. | `status`, `tel`, `start`, `stop` | `[APP] active_mode=LiDARObstacleAvoidance`, `[CMD_RX] line=start`, `[AMR] state IDLE -> EXPLORE`, `[OBS] hold reason=front_blocked ...`, `APP LIDAR_OBS: state=BACKUP`, `state=TURN`, `state=RECOVER`, `[AMR] state ... -> IDLE reason=serial_stop` |
| 3:30-4:30 | Safety demo. Camera on robot and PC13. Terminal visible. Long press PC13. | Demonstrate USER_ESTOP. Long press PC13 to trigger software ESTOP. Confirm fault, PWM zero, safe stop. Long press again while in FAULT/ESTOP to clear fault and reset odometry/map. | Optional before/after: `status` | `[BTN] long_press`, `FAULT: code=USER_ESTOP`, `FAULT: safe stop`, `[ESTOP] motor outputs disabled`, `[BTN] action=clear_fault_reset_odom`, `FAULT: cleared`, `[ODOM] reset`, `[STATUS] state=IDLE fault=NONE` |
| 4:30-5:30 | Validation evidence. Show terminal with `tel`, `status`, `enc_dbg`. Show wheel/encoder calibration note or table. | Show telemetry and diagnostics. Explain encoder one-revolution calibration: left approximately 1738 ticks, right approximately 1650 ticks, average approximately 1694 ticks/rev. Final odometry uses `ODO_ENCODER_TICKS_PER_REV=1694`. Explain odometry freeze was an engineering decision after wheel asymmetry/slip caused pose instability. | `tel`, `status`, `enc_dbg` or `odom_dbg` | `TEL: ... odo_frozen=1 ...`, `ODO_DBG: frozen=1 cntL=... cntR=...`, `ODO_DBG: total rawL=... rawR=...`, `ticks_rev=1694` |
| 5:30-6:30 | Code walkthrough in IDE. Show main modules in file explorer. | Highlight modules: `app_lidar_obstacle_avoidance`, `app_fault`, `app_safety`, `app_odometry`, `app_map`, `app_explorer`, `app_ui`/serial telemetry, and `freertos`. Emphasise task separation, fault-safe behavior, diagnostic interfaces, and minimal-risk integration through existing chassis/motor APIs. | None | Code view evidence only |
| 6:30-7:00 | Closing slide with limitations and future work. Robot in safe IDLE. | State limitations clearly: no full Start-to-Exit-to-Return benchmark autonomy; no full scan matching/ICP; no full A*/DWA; Bluetooth not implemented; OLED disabled; software ESTOP is not a hardware latched E-stop. Future work: enable live odometry after wheel balancing/slip compensation, improve planner, add hardware E-stop and low-voltage protection. | `status` optional | `[STATUS] state=IDLE fault=NONE`, `pwmL=0 pwmR=0`, `odo_frozen=1` |

## Narrator Script

### 0:00-1:00 Team Introduction and Responsibilities

"This is our EBU6475 Microprocessor Systems Design project: an STM32
NUCLEO-F446RE based 2D LiDAR autonomous mobile robot. The project goal is to
demonstrate an embedded Sense-Think-Act pipeline on real hardware using LiDAR
perception, motor actuation, safety handling, and serial telemetry.

For team responsibilities, `[Member A]` focused on FreeRTOS integration and
application structure, `[Member B]` focused on LiDAR perception and obstacle
logic, `[Member C]` focused on motor, encoder, and odometry validation, and
`[Member D]` focused on documentation, testing evidence, and video preparation.

This video demonstrates the final stable demo build. The focus is reliable
LiDAR obstacle avoidance, safe stop behavior, and telemetry evidence."

### 1:00-2:00 System Architecture

"The system is organised around Sense, Think, and Act.

On the Sense side, the robot uses a 2D LiDAR connected through UART4 at 460800
baud. It also reads wheel encoders for diagnostic and calibration evidence, and
the IMU status is available as part of the wider embedded platform.

On the Think side, FreeRTOS separates periodic application work. The main logic
includes the LiDAR obstacle avoidance state machine, the AMR state machine, a
latched fault manager, safety checks, odometry diagnostics, and software
frameworks for map and explorer development.

On the Act side, the robot uses chassis-level commands and the motor driver to
stop, drive, back up, turn, recover, and safe stop. Serial telemetry is kept
low-rate and command processing is handled outside the UART interrupt context,
so debug output does not block the real-time control path."

### 2:00-3:30 Product Demonstration

"We first check the system using the `status` and `tel` commands. The active
mode is `LiDARObstacleAvoidance`, LiDAR is ready, and the final demo shows
`odo_frozen=1`. This means odometry pose integration is intentionally frozen for
demo stability, while encoder diagnostics still remain available.

Now we send `start`. The robot enters the obstacle avoidance mode. When the
LiDAR front distance detects an obstacle inside the threshold, the robot stops,
backs up, turns, and then recovers. This demonstrates the final Sense-Think-Act
loop: LiDAR senses the obstacle, the application logic selects a safe local
action, and the chassis executes the motor command.

After the motion demonstration, we send `stop`. The robot returns to IDLE and
the motor PWM command is zero."

### 3:30-4:30 Safety Demonstration

"Next we demonstrate safety behavior using the PC13 user button. A long press
triggers `USER_ESTOP`. The firmware enters ESTOP, disables motor outputs, and
reports the safe stop through serial logs.

While the robot is in ESTOP or fault state, another long press clears the fault
and resets odometry and map state. This gives a simple repeatable recovery path
for the final demo."

### 4:30-5:30 Validation and Telemetry Evidence

"The final demo uses serial telemetry as the primary user interface because the
OLED display is disabled due to pin and resource constraints. The `tel` command
gives a compact validation line, and `status` gives a more detailed system
state.

For encoder validation, we use `enc_dbg` or `odom_dbg`. During calibration, one
wheel revolution measured approximately 1738 ticks on the left wheel and 1650
ticks on the right wheel. The average is approximately 1694 ticks per wheel
revolution, so the final odometry parameter is
`ODO_ENCODER_TICKS_PER_REV=1694`.

Live odometry integration is frozen by default with `APP_ODO_FREEZE_DEFAULT=1`.
This was a deliberate engineering trade-off. Ground testing showed wheel
asymmetry and slip could make the pose jump, so the final video prioritises
reliable LiDAR obstacle avoidance and safe stop behavior while keeping encoder
diagnostics available."

### 5:30-6:30 Code Architecture Walkthrough

"The main firmware modules are organised by responsibility.

`app_lidar_obstacle_avoidance` contains the final local avoidance behavior:
front detection, stop, backup, turn, and recover.

`app_fault` and `app_safety` provide latched fault handling, LiDAR timeout
monitoring, encoder stall detection, and safe stop integration.

`app_odometry` provides encoder diagnostics, calibration parameters, optional
pose integration, freeze control, and sanity guards.

`app_map` and `app_explorer` provide a lightweight 5x5 cell map and exploration
skeleton. These are framework components for future work and are not used as
final live benchmark navigation.

`app_ui` and the serial command module provide telemetry, status output, and
debug commands. FreeRTOS keeps these application functions separated into
periodic task contexts."

### 6:30-7:00 Limitations and Reflection

"To be technically accurate, this final demo does not claim full autonomous
Start-to-Exit-to-Return benchmark completion. It also does not include full scan
matching or ICP, full A* or DWA navigation, verified Bluetooth, an enabled OLED
display, or a hardware latched emergency stop.

The main engineering result is a stable embedded LiDAR obstacle avoidance and
safety demonstration with useful telemetry and calibration evidence. Future work
would focus on wheel balancing and slip compensation, re-enabling live odometry,
implementing robust cell navigation and frontier planning, adding Bluetooth if
resources allow, and adding a hardware E-stop and low-voltage protection."

## Demo Command Sequence

Recommended terminal sequence:

```text
status
tel
start
enc_dbg
tel
stop
status
```

Safety sequence:

```text
Long press PC13 for USER_ESTOP
status
Long press PC13 again to clear fault and reset odometry/map
status
```

Optional diagnostic sequence:

```text
odom_dbg
map
exp
odo_freeze 1
odo_freeze 0
odo_freeze 1
```

## Expected Log Snippets

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
APP LIDAR_OBS: state=RECOVER ...
[CMD_RX] line=enc_dbg
ODO_DBG: frozen=1 cntL=... cntR=...
ODO_DBG: total rawL=... rawR=... signedL=... signedR=...
[CMD_RX] line=stop
[AMR] state EXPLORE -> IDLE reason=serial_stop
[BTN] long_press
FAULT: code=USER_ESTOP
FAULT: safe stop
[ESTOP] motor outputs disabled
[BTN] action=clear_fault_reset_odom
FAULT: cleared
[ODOM] reset
```

## Shooting Checklist

Before recording:

- Battery or power source is stable.
- Robot has enough floor space for backup and turn.
- LiDAR is spinning and connected to UART4.
- Serial terminal is open at 115200 baud, 8N1.
- Terminal font is large enough for the camera.
- `status` shows `lidar ready=1`.
- `status` or `tel` shows `odo_frozen=1`.
- Motor PWM is zero before starting.
- PC13 button is accessible for safety demonstration.
- Obstacle is soft and safe, such as a box or bag.

During recording:

- Keep hands clear of wheels during motion.
- Keep the serial terminal and robot visible where possible.
- Do not claim full autonomous benchmark navigation.
- Verbally identify `odo_frozen=1` as an engineering stability decision.
- Show at least one `enc_dbg` output for calibration evidence.
- Demonstrate `stop` and PC13 long-press ESTOP.

After recording:

- Send `stop` or trigger ESTOP before handling the robot.
- Confirm `state=IDLE` or `state=ESTOP`.
- Confirm PWM is zero.
- Save serial logs if required for the final report.

## Notes for Editing

- Keep the final video under 7 minutes.
- Use captions for key commands: `status`, `tel`, `start`, `stop`, `enc_dbg`,
  and PC13 long press.
- Include a short text overlay: "Final demo: LiDAR obstacle avoidance + safety +
  serial telemetry. Not full benchmark autonomy."
- Avoid long raw serial scrolling; show only the evidence lines needed for the
  narration.

