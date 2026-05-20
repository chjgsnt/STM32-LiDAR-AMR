# Step 4 Chassis Open-Loop Test

Target: STM32 NUCLEO-F446RE / STM32F446RE

Scope:

- Add a minimal chassis motion abstraction above the verified AT8236 motor driver.
- Verify open-loop chassis actions without PID control.
- Check that the left and right 520 encoder motors move in the expected direction.
- Confirm encoder counts continue updating on TIM2 and TIM4.
- Confirm I2C1 and the MPU6500 IMU continue reporting normally during chassis motion.

## Hardware Prerequisites

- AT8236 motor driver bring-up has passed.
- Motor A PWM and encoder counting have passed.
- Motor B PWM and encoder counting have passed.
- STM32 GND and motor driver GND are connected together.
- `VM` is supplied by the external motor power source.
- I2C1 MPU6500 wiring remains on `PB8/PB9` and should still report `APP IMU: ready=1`.

## Software Switches

Step4 uses the chassis open-loop test and keeps the earlier low-level motor tests disabled by default:

```c
#define APP_ENABLE_MOTOR_TEST 0
#define APP_ENABLE_MOTOR_GPIO_STATIC_TEST 0
#define APP_ENABLE_CHASSIS_OPENLOOP_TEST 1
#define APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST 0
#define APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST 0
```

The chassis layer currently assumes:

```c
#define CHASSIS_LEFT_SIGN  (-1)
#define CHASSIS_RIGHT_SIGN (1)
```

This means Motor A is treated as the left wheel, Motor B is treated as the right wheel, the left wheel command is inverted, and the right wheel command keeps the default direction.

## Motor Mapping

| Chassis wheel | Motor driver channel | PWM pins | Encoder timer |
|---|---|---|---|
| Left wheel | Motor A | PA6/PA7, TIM3_CH1/TIM3_CH2 | TIM2, PA0/PA1 |
| Right wheel | Motor B | PB0/PB1, TIM3_CH3/TIM3_CH4 | TIM4, PB7/PB6 |

- Motor A currently maps to the left wheel.
- Motor B currently maps to the right wheel.
- Direction is corrected using `CHASSIS_LEFT_SIGN` and `CHASSIS_RIGHT_SIGN`.

## Test Flow

The FreeRTOS control task runs this one-time sequence:

1. Print `[CHASSIS] open-loop test start`.
2. `Chassis_Init()`.
3. Stop for `2 s`.
4. Forward at `duty=500` for `3 s`, logging every `500 ms`.
5. Stop for `1 s`.
6. Backward at `duty=500` for `3 s`, logging every `500 ms`.
7. Stop for `1 s`.
8. Turn left at `duty=500` for `3 s`, logging every `500 ms`.
9. Stop for `1 s`.
10. Turn right at `duty=500` for `3 s`, logging every `500 ms`.
11. Stop.
12. Print `[CHASSIS] open-loop test done`.

Each chassis log line contains:

```text
[CHASSIS] action=<action> left_duty=<left> right_duty=<right> encA=<count> encB=<count>
```

The IMU log should continue printing `APP IMU: ready=1` while the chassis test is running.

## Direction Check

Expected wheel behavior:

- `Forward`: both wheels drive the chassis forward.
- `Backward`: both wheels drive the chassis backward.
- `TurnLeft`: left wheel reverses and right wheel moves forward.
- `TurnRight`: left wheel moves forward and right wheel reverses.

If the chassis moves forward and both encoder counts change consistently, the left/right mapping and direction signs are probably correct. If one wheel moves opposite from the expected direction, only that wheel sign should be changed.

## Direction Calibration Result

The chassis direction calibration test produced:

| Calibration action | Observed wheel direction |
| --- | --- |
| Left `+300` | Backward |
| Left `-300` | Forward |
| Right `+300` | Forward |
| Right `-300` | Backward |

Conclusion:

- Left wheel direction is reversed.
- Right wheel direction is correct.
- Final sign configuration is `CHASSIS_LEFT_SIGN = -1` and `CHASSIS_RIGHT_SIGN = 1`.

## Direction Sign Fix

If `Forward` becomes backward for both wheels, change both signs:

```c
#define CHASSIS_LEFT_SIGN  (-1)
#define CHASSIS_RIGHT_SIGN (-1)
```

If only the left wheel is reversed, change only:

```c
#define CHASSIS_LEFT_SIGN (-1)
```

If only the right wheel is reversed, change only:

```c
#define CHASSIS_RIGHT_SIGN (-1)
```

After changing signs, rerun the open-loop test with the wheels lifted and confirm `Forward`, `Backward`, `TurnLeft`, and `TurnRight` again.

## Pass Criteria

Step4 is considered passed when:

- `APP IMU: ready=1` continues during the whole chassis test.
- `Forward` causes both encoder counts to change continuously.
- `Backward` causes both encoder counts to change continuously in the opposite direction.
- `TurnLeft` and `TurnRight` cause the two wheels to rotate in opposite directions.
- The final log prints `[CHASSIS] open-loop test done`.

## Notes

- At `duty=250`, the left wheel may not start reliably during `Forward`; `duty=300` is used for lifted-wheel bring-up checks.
- `duty=300` is suitable for lifted-wheel direction verification.
- `duty=300` can move wheels when lifted, but may be weak on ground.
- The ground traction test is used to find the duty needed for reliable ground startup.
- `duty=500` is selected as the current best open-loop ground test duty.
- `duty=600` is kept as a short-duration upper test value only.
- Duty must be increased step by step, and the chassis should be held during the test.

## Safety Notes

- Keep the wheels lifted for the first chassis open-loop tests.
- Start with low duty values.
- Do not reverse `VM` and `GND`.
- Ground tests must be short and supervised.
- Confirm the motor driver power path before connecting any driver board `5V` pin.
- Stop the chassis before changing wiring or power connections.
