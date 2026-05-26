# AMR LiDAR Robot Demo Guide

## 1. Hardware setup

- STM32 NUCLEO-F446RE.
- LiDAR connected to UART4 at 460800 baud.
- PC debug UART via USART2 / COM port, 115200 8N1.
- OLED and IMU on I2C.
- Differential drive chassis with left and right wheel encoders.
- USER BUTTON on PC13 for offline control.

## 2. Offline button controls

- Short press in IDLE: start exploration.
- Short press in EXPLORE or AVOID: start return-to-start.
- Short press in RETURN: stop and return to IDLE.
- Short press in FAULT or ESTOP: reset to IDLE.
- Long press for 2 seconds: emergency stop.

## 3. Serial commands

- `status`
- `start`
- `stop`
- `return`
- `estop`
- `reset_fault`
- `odom_reset`

## 4. OLED display meaning

- `MODE`: `IDLE`, `EXP`, `AVD`, `RET`, `FLT`, or `EST`.
- `FRONT`: LiDAR front distance in millimeters.
- `ENC`: left and right raw encoder delta.
- `PWM`: left and right chassis command.
- `RET:n` or `PATH:n`: remaining return/path action count when shown.

## 5. Demo procedure

1. Power on, wait for `MODE:IDLE` and a valid LiDAR front distance.
2. Short press the USER BUTTON to start.
3. Place an obstacle in front; the robot should backup, turn, and recover.
4. Let the robot record several path actions.
5. Short press the USER BUTTON again to enter RETURN.
6. The robot executes inverse recorded actions.
7. Long press the USER BUTTON to demonstrate ESTOP.
8. Short press again to reset to IDLE.

## 6. Expected serial evidence

```text
[BTN] action=start
[AMR] state IDLE -> EXPLORE reason=button_start
[PATH] record action=BACKUP ...
[RETURN] start count=...
[RETURN] exec action=...
[RETURN] done actions=0
[BTN] action=estop
[ESTOP] motor outputs disabled
```

## 7. Safety behavior

- LiDAR RX timeout enters FAULT.
- Encoder stall enters FAULT.
- ESTOP disables motor outputs.
- `stop` and `estop` can interrupt RETURN.

## 8. Known limitations

- Return-to-start is a recorded-action prototype, not full SLAM or A* planning.
- Odometry is used for telemetry but not yet closed-loop global navigation.
- Occupancy grid mapping and full maze planning remain future work.
- Return performance depends on wheel slip and obstacle layout.
