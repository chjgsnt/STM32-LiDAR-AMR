# EBU6475 Final Technical Documentation

## STM32 NUCLEO-F446RE 2D LiDAR AMR

This report documents the final stable demonstration build of the STM32
FreeRTOS-based AMR project. The final demo focuses on a reliable embedded
Sense-Think-Act pipeline: LiDAR front-distance sensing, local obstacle
avoidance, motor actuation, safe stop behavior, serial telemetry, and
engineering validation evidence.

The final demo does not claim complete autonomous Start-to-Exit-to-Return maze
benchmark completion. Software frameworks for odometry, mapping, and
exploration have been implemented, but live odometry-map navigation is not
enabled in the final stable demo because ground testing showed wheel asymmetry,
slip, and pose instability.

## Final Demo Configuration

| Item | Final demo setting |
| --- | --- |
| MCU | STM32 NUCLEO-F446RE |
| RTOS | FreeRTOS |
| Active mode | `LiDARObstacleAvoidance` |
| LiDAR | UART4, 460800 baud |
| Debug serial | USART2 virtual COM, 115200 8N1 |
| Odometry integration | Frozen by default, `APP_ODO_FREEZE_DEFAULT=1` |
| Encoder diagnostics | Enabled through `enc_dbg`, `odom_dbg`, `odo_freeze` |
| Encoder ticks/rev | Calibrated to `ODO_ENCODER_TICKS_PER_REV=1694` |
| OLED | Disabled by default due to pin/resource constraints |
| UI evidence | Serial telemetry through `status` and `tel` |
| Bluetooth | Not implemented |
| Map/explorer | Implemented as software framework/skeleton, not final live navigation |
| Emergency stop | Software ESTOP through command and PC13 long press |

## 1. Project Overview and Requirements Interpretation

The project brief requires an embedded mobile robot system that demonstrates a
Sense-Think-Act pipeline and includes embedded control, sensing, safety,
interface, and validation evidence. The intended benchmark context is a small
maze-like environment, but the final deliverable must be technically honest and
robust under real hardware constraints.

The final project interpretation is:

- Sense: receive and parse 2D LiDAR scan data, extract front obstacle distance,
  read wheel encoders, and expose diagnostic telemetry.
- Think: run an AMR state machine, LiDAR obstacle avoidance state machine,
  fault/safety checks, and command handling under FreeRTOS.
- Act: drive the differential chassis through existing chassis/motor layers,
  execute stop, backup, turn, recover, and safe-stop actions.
- Validate: provide repeatable serial evidence, encoder calibration output,
  LiDAR distance logs, state transitions, and safety behavior.

The final demo prioritises reliable obstacle avoidance and safe operation over
claiming full autonomous maze solving. This is a deliberate engineering choice:
the robot can demonstrate embedded autonomy and validation evidence without
depending on unstable live odometry-map navigation.

### Requirement vs Implementation Status

| Requirement area | Status | Evidence / note |
| --- | --- | --- |
| LiDAR sensing | Implemented | UART4 LiDAR parser, front distance, ready/valid status |
| Motor actuation | Implemented | Chassis commands drive stop, forward, backup, turn |
| Encoder reading | Implemented | `enc_dbg`/`odom_dbg` report raw/signed deltas and distance |
| Speed/closed-loop support | Implemented in project | Encoder-based motor control and PI/PID test infrastructure exist |
| Obstacle avoidance | Implemented and final-demo enabled | `LiDARObstacleAvoidance` stop/backup/turn/recover |
| AMR state machine | Implemented | IDLE, EXPLORE, AVOID, RETURN, FAULT, ESTOP |
| Fault manager / safe stop | Implemented | LiDAR timeout, encoder stall, ESTOP safe stop |
| PC13 user input | Implemented | Long press enters USER_ESTOP; long press in fault clears/reset |
| Serial telemetry | Implemented and final-demo enabled | `status`, `tel`, command logs |
| Odometry pose | Implemented, not enabled in final demo navigation | `APP_ODO_FREEZE_DEFAULT=1`, `odo_frozen=1` |
| Map/explorer | Partially implemented, not enabled in final demo navigation | 5x5 map and DFS/frontier skeleton only |
| OLED UI | Implemented earlier, not enabled in final demo | Disabled due to pin/resource constraints |
| Bluetooth | Not implemented | No verified Bluetooth hardware/software integration |
| Full Start-to-Exit-to-Return | Not claimed | Future work requiring robust odometry/navigation |

## 2. Conceive: Problem Framing, Constraints, and Risks

The original concept was an autonomous mobile robot capable of exploring a
small maze, avoiding obstacles, and eventually returning to the start. During
bring-up, the project was reframed around a stable embedded demonstration that
could be verified repeatedly on the available hardware.

### Constraints

- Limited STM32 NUCLEO-F446RE pins and communication resources.
- LiDAR requires a high-speed UART and continuous receive handling.
- Motor and encoder signals are sensitive to wiring, slip, and wheel mismatch.
- OLED display was not practical in the final hardware setup due to pin/resource
  constraints.
- Bluetooth was not integrated or verified.
- Odometry was affected by left/right wheel distance mismatch during ground
  testing.
- The final demonstration needed to be stable and recoverable under live test
  conditions.

### Main Risks and Mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| LiDAR RX timeout or stale data | Robot could move without valid perception | Safety monitor faults and stops motion |
| Encoder stall | Motor commanded but wheel not moving | Fault manager detects stall and safe stops |
| Odometry pose jump | Map cell jumps and navigation becomes unsafe | Default freeze of odometry integration |
| Serial logging overload | Timing disruption or confusing evidence | Logging reduced and status/tel commands used |
| UI hardware conflict | OLED unreliable or unavailable | Serial telemetry used as UI evidence |
| Unstable full navigation | Failed final demo | Focus on verified LiDAR obstacle avoidance |

The final demo is therefore designed as a conservative AMR baseline: it senses a
front obstacle, decides a local avoidance action, drives the chassis, and enters
a safe state when faults occur.

## 3. Design: Sense-Think-Act Architecture

### System Architecture

The system is organised into application modules above the STM32 HAL and
FreeRTOS task layer.

```text
Sense:
  LiDAR UART4 parser
  Encoder counters
  PC13 button

Think:
  AMR state machine
  LiDAR obstacle avoidance state machine
  Fault manager / safety monitor
  Serial command parser
  Odometry diagnostics
  Map/explorer software framework

Act:
  Chassis command layer
  Motor PWM driver
  Safe stop / ESTOP paths
  Serial telemetry output
```

### FreeRTOS Task Structure

The project uses FreeRTOS to separate periodic control, sensing, safety, and UI
work. The exact task names are generated by the project, but the application
logic follows these timing roles:

- LiDAR receive/parser handling through UART4 callback and parser update logic.
- Control-period updates for AMR state, obstacle avoidance, return/explorer
  framework, odometry sampling, and map update.
- Safety update at a regular period to check LiDAR RX age and encoder stall.
- Serial command processing outside interrupt context.
- UI/telemetry update at low rate to avoid flooding the serial port.

Serial command execution was intentionally moved out of UART interrupt context
to avoid long `printf` operations inside callbacks.

### Module Design

| Module | Role |
| --- | --- |
| `app_lidar` | UART4 LiDAR receive, scan parsing, sector distance extraction |
| `app_lidar_obstacle_avoidance` | Final enabled local avoidance behavior |
| `amr_system` | High-level AMR state machine |
| `app_safety` | Runtime checks for LiDAR timeout and encoder stall |
| `app_fault` | Latched fault code and safe-stop behavior |
| `chassis` / `motor_driver` | Chassis-level commands and PWM output |
| `app_odometry` | Encoder diagnostics, calibration, optional pose integration |
| `app_map` | 5x5 cell-map software framework |
| `app_explorer` | DFS/frontier-style exploration skeleton |
| `app_ui` | Serial UI/telemetry; OLED disabled in final demo |
| `app_serial_command` | USART2 command parser and status output |
| `app_button_control` | PC13 long press ESTOP / fault clear behavior |

## 4. Implement

### LiDAR Obstacle Avoidance

The final active mode is `LiDARObstacleAvoidance`. It uses LiDAR front-sector
distance to decide local actions:

```text
IDLE -> EXPLORE
LiDAR clear: DRIVE_FORWARD
Obstacle close: stop/hold -> BACKUP -> TURN -> RECOVER
Fault or ESTOP: safe stop
```

The important evidence is the front distance and state transition log:

```text
[CMD_RX] line=start
[AMR] state IDLE -> EXPLORE reason=serial_start
[OBS] hold reason=front_blocked front=... stop=...
APP LIDAR_OBS: state=BACKUP reason=obstacle_detected
APP LIDAR_OBS: state=TURN reason=backup_done
APP LIDAR_OBS: state=RECOVER ...
```

This behavior demonstrates perception-driven actuation without requiring global
SLAM or path planning.

### Motor Control and Actuation

The final demo uses the existing chassis and motor layers rather than writing
directly to low-level PWM from application code. This keeps motor behavior
centralised and allows the safe-stop paths to disable outputs consistently.

Implemented motor actions include:

- stop all motor output,
- drive forward,
- backup,
- differential turn,
- safe stop on FAULT/ESTOP.

The project also includes encoder-based control and PI/PID wheel speed test
infrastructure. For the final demo, the critical evidence is that motor output
responds to LiDAR obstacle decisions and safe-stop commands.

### Fault Manager and Safe Stop

The fault manager is latched: once a fault is active, the system does not resume
motion automatically. It must be cleared by command or by the PC13 long-press
recovery path.

Faults supported in the project include:

- `FAULT_NONE`
- `FAULT_LIDAR_TIMEOUT`
- `FAULT_ENCODER_STALL`
- `FAULT_PLANNER_STUCK` as a reserved software/planner code
- `FAULT_USER_ESTOP`

Safe-stop behavior calls the chassis/motor stop path and prevents continued
motion in FAULT/ESTOP states. This is a software emergency stop, not a hardware
latched power cut.

### Serial Telemetry and UI

OLED hardware is disabled in the final build due to pin/resource constraints.
The UI evidence is serial based:

- `status`: multi-line detailed system state.
- `tel`: compact telemetry line for video and validation evidence.
- `ui` / `page`: serial fallback UI page state.

Expected telemetry includes:

```text
TEL: mode=IDLE fault=NONE odo_frozen=1 x=... y=... th=... cell=(...) ...
```

### Odometry Diagnostics

The odometry module implements differential-drive pose estimation, encoder
debug sampling, freeze control, and sanity guards. However, final demo pose
integration is frozen by default:

```text
APP_ODO_FREEZE_DEFAULT=1
status/tel -> odo_frozen=1
```

This allows encoder evidence to be captured without letting wheel mismatch or
slip corrupt the live pose/map state.

Encoder one-revolution calibration was performed:

| Wheel | Measured one-revolution ticks | Earlier distance estimate symptom |
| --- | ---: | --- |
| Left | approximately 1738 ticks | distance was over-scaled before calibration |
| Right | approximately 1650 ticks | distance was over-scaled before calibration |
| Average | approximately 1694 ticks/rev | final `ODO_ENCODER_TICKS_PER_REV=1694` |

The earlier value of `ticks_per_rev=390` was not valid for the assembled robot
and produced approximately 4.2x distance overestimation. The corrected value was
based on physical wheel rotation testing.

### Map and Explorer Framework

The project includes a 5x5 cell-level map and a DFS/frontier-style explorer
skeleton. These are useful for future development and demonstrate software
structure for maze navigation, but they are not used as final live navigation.

Final status:

- Implemented: map data structures, visited/wall fields, grid/status commands.
- Partially implemented: explorer state skeleton and path planning framework.
- Not enabled in final demo: live odometry-based cell movement and full
  autonomous Start-to-Exit-to-Return benchmark navigation.

This distinction is important because the final demo does not rely on unstable
odometry to decide movement through the maze.

## 5. Operate: Validation Tests and Evidence

### Validation Test vs Result

| Test | Method | Result |
| --- | --- | --- |
| LiDAR front distance | Move object in front of LiDAR and read `status`/logs | Front distance changes and valid flag reports correctly |
| Obstacle response | Send `start`, place obstacle within threshold | Robot stops/backs up/turns/recovers |
| Serial start/stop | Send `start`, then `stop` | State changes IDLE -> EXPLORE -> IDLE, motor output stops |
| PC13 USER_ESTOP | Long press PC13 for about 2 seconds | `FAULT_USER_ESTOP` / ESTOP safe stop triggered |
| Fault clear | Long press PC13 in FAULT/ESTOP | Fault cleared, odometry/map reset path executed |
| Encoder calibration | Rotate each wheel one revolution | Left about 1738 ticks, right about 1650 ticks |
| Odometry freeze | Boot and run `status`/`tel` | `odo_frozen=1`, pose integration disabled |
| Encoder diagnostics | Run `enc_dbg` / `odom_dbg` | Raw/signed deltas and distance estimates reported |
| Serial telemetry | Run `status` and `tel` | Detailed and compact evidence available |
| Map/explorer commands | Run `map`, `grid`, `exp` | Framework status prints, not used for final navigation |

### Representative Evidence

Expected stable-demo log evidence:

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
FAULT: code=USER_ESTOP
FAULT: safe stop
[ESTOP] motor outputs disabled
```

This evidence demonstrates that the final build can sense obstacles, make a
reactive local decision, actuate the chassis, report telemetry, and enter a safe
state when commanded.

## 6. Engineering Trade-offs and Limitations

The final demo intentionally favours a reliable, verifiable embedded behavior
over an over-ambitious navigation claim.

### Implemented and Enabled in Final Demo

- LiDAR obstacle detection.
- Local obstacle avoidance: stop, backup, turn, recover.
- AMR state transitions.
- Motor actuation through chassis layer.
- Fault manager and safe stop.
- PC13 long-press USER_ESTOP and fault-clear/reset behavior.
- Serial command interface and telemetry.
- Encoder calibration diagnostics.

### Implemented but Not Enabled as Final Navigation

- Odometry pose integration.
- 5x5 cell map.
- DFS/frontier-style explorer skeleton.
- Recorded-action/path framework.

These features remain useful software foundations but are not presented as
validated live navigation in the final demo.

### Not Implemented or Not Verified

- Full scan matching or ICP.
- Full A* or DWA maze navigation.
- Verified Bluetooth telemetry/control.
- OLED hardware display in the final build.
- Hardware latched emergency stop or motor power cut.
- Low-voltage protection.

### Rationale for Freezing Odometry

Ground testing showed large left/right wheel distance mismatch during forward
motion, for example `dl_mm=121` and `dr_mm=386`. Even after one-revolution tick
calibration became reasonable, wheel asymmetry and slip could still produce
heading drift and map cell jumps. To protect the final demo from unstable pose
integration:

- odometry integration is frozen by default,
- encoder sampling and diagnostics remain active,
- map updates are skipped while odometry is frozen,
- `odo_freeze 0` remains available for controlled experiments.

This is a practical engineering trade-off that preserves reliable LiDAR
obstacle avoidance and safety demonstration.

## 7. Responsible GenAI Use

Generative AI was used as an engineering assistant for structuring modules,
debugging hypotheses, and drafting code/documentation changes. It was not used
as an unverified source of truth.

### Useful GenAI Output Example

GenAI helped structure the odometry, fault manager, map framework, serial
command handling, and diagnostic test flow. For example, it suggested separating
fault handling into a latched fault manager, adding `enc_dbg`/`odom_dbg`
diagnostics, and documenting validation evidence through serial telemetry. These
suggestions were useful because they matched embedded software engineering
practice and were verified by builds and hardware logs.

### Misleading or Unhelpful Output Example

An early odometry assumption used `ticks_per_rev=390`. Hardware calibration
showed this was not valid for the assembled robot: one wheel revolution measured
approximately 1738 ticks on the left and 1650 ticks on the right. The incorrect
assumption caused distance overestimation by about 4.2x and had to be corrected
through a physical one-revolution wheel test. The final value was set to the
measured average, `ODO_ENCODER_TICKS_PER_REV=1694`.

### Verification Approach

All AI-assisted changes were checked through:

- clean CMake builds,
- serial boot logs,
- LiDAR ready/front-distance logs,
- state transition logs,
- encoder diagnostic logs,
- motor safe-stop observations,
- deliberate fault/ESTOP tests.

AI output was treated as a draft or hypothesis. Hardware evidence and build
results determined the final implementation decisions.

## 8. Reflection and Future Work

The project reached a stable embedded AMR demonstration, but several areas
remain open for future development.

### Reflection

The most important lesson was that an embedded robotics demo must be designed
around hardware evidence, not only planned algorithms. LiDAR obstacle avoidance
proved stable and demonstrable. In contrast, live odometry-map navigation was
not stable enough for the final demo because wheel mismatch and slip created
pose errors. Freezing odometry integration while keeping diagnostics available
was a reasonable final engineering compromise.

The project also showed the value of clear diagnostic commands. `status`, `tel`,
`enc_dbg`, and `odom_dbg` made it possible to separate LiDAR, motor, encoder,
odometry, and safety problems during testing.

### Future Work

- Improve wheel calibration and left/right closed-loop balance.
- Characterise wheel slip under real floor conditions.
- Re-enable live odometry after slip compensation and speed balance are stable.
- Add IMU yaw fusion to reduce heading drift.
- Implement robust cell navigation using the existing 5x5 map framework.
- Add frontier selection and later A* path planning.
- Add Bluetooth telemetry/control if spare UART/pins are available.
- Revisit OLED or an alternative UI only if resources permit.
- Add a hardware latched emergency stop that cuts motor power.
- Add low-voltage monitoring and protection.

## Final Conclusion

The final stable build demonstrates a conservative but credible embedded AMR
pipeline. It uses LiDAR to detect front obstacles, makes local avoidance
decisions, actuates the chassis, reports serial telemetry, and enters safe-stop
states through fault handling and PC13 USER_ESTOP.

The project includes odometry, map, and explorer software frameworks, but these
are explicitly separated from the final enabled demo because live odometry-based
navigation was not stable enough for reliable presentation. This distinction
keeps the final documentation technically accurate while still showing a clear
path for future expansion toward full maze navigation.
