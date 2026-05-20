# Step 6 Wheel Speed PI Closed-Loop Test

Target: STM32 NUCLEO-F446RE / STM32F446RE

Scope:

- Add an independent PI speed loop for the left and right wheels.
- Use encoder count deltas as the first wheel-speed feedback signal.
- Keep the verified Step3/Step4/Step5 bring-up tests available.
- Keep the MPU6500 IMU and I2C1 baseline/recovery path unchanged.

## Goal

Step6 moves from left/right speed balancing into true wheel speed control. Each wheel now has its own PI controller:

- left wheel target: `800 ticks/sample`
- right wheel target: `800 ticks/sample`
- sample period: `200 ms`
- output: chassis-level duty magnitude, limited to `300..600`

The test still uses ticks per sample instead of RPM. That keeps the first closed-loop test simple and close to the Step5 encoder delta logs.

Status: passed.

## Why PI After Step5 P-Only Balance

Step5 verified that encoder deltas are usable and that P-only balance can reduce the speed difference between the left and right wheels. However, Step5 only reacts to the difference between wheels:

```text
err = speed_left - speed_right
```

That makes the chassis straighter, but it does not force either wheel to reach an absolute speed target. If both wheels are slow or both wheels are fast, Step5 has no absolute speed correction.

Step6 gives each wheel its own target:

```text
errL = target_ticks_per_sample - abs(deltaA)
errR = target_ticks_per_sample - abs(deltaB)
```

The proportional term responds immediately to speed error, while the integral term removes steady-state error caused by friction, battery voltage, motor mismatch, and load.

## Encoder Delta Speed Estimate

The controller samples encoder counts every `200 ms`.

For each control sample:

```text
encA_now = MotorDriver_GetEncoderA()
encB_now = MotorDriver_GetEncoderB()

deltaA = encA_now - encA_last
deltaB = encB_now - encB_last

measured_left = abs(deltaA)
measured_right = abs(deltaB)
```

The absolute delta is used as the speed feedback, so the same PI loop works for both forward and backward tests. Direction is applied only after the PI output magnitude is calculated.

## PI Control Formula

Each wheel uses an independent PI controller:

```text
error = target_ticks_per_sample - measured_ticks
integral += error
integral = clamp(integral, -integral_limit, integral_limit)

duty = feedforward_duty + Kp * error + Ki * integral
duty = clamp(duty, duty_min, duty_max)
```

The implementation uses integer fixed-point gains:

```c
Kp = 100 / 100 = 1.00
Ki = 5 / 100 = 0.05
```

The PI output is a positive duty magnitude. The test then applies direction:

```text
forward:  left_cmd = +left_mag, right_cmd = +right_mag
backward: left_cmd = -left_mag, right_cmd = -right_mag
```

`Chassis_SetRaw(left_cmd, right_cmd)` still applies:

```c
CHASSIS_LEFT_SIGN = -1
CHASSIS_RIGHT_SIGN = 1
```

## Initial Parameters

```c
sample_period_ms = 200
target_ticks_per_sample = 800
feedforward_duty = 500
Kp = 1.00
Ki = 0.05
integral_limit = +/-2000
duty_min = 300
duty_max = 600
```

Current Step5 trim remains available for the speed balance test, but Step6 PI does not apply static trim:

```c
CHASSIS_LEFT_TRIM = -10
CHASSIS_RIGHT_TRIM = 10
```

## Software Switches

Step6 enables only the wheel speed PI test:

```c
#define APP_ENABLE_MOTOR_TEST 0
#define APP_ENABLE_MOTOR_GPIO_STATIC_TEST 0
#define APP_ENABLE_CHASSIS_OPENLOOP_TEST 0
#define APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST 0
#define APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST 0
#define APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST 0
#define APP_ENABLE_WHEEL_SPEED_PI_TEST 1
```

## Test Flow

The FreeRTOS control task runs this one-time sequence:

1. Print `[PI] wheel speed PI test start`.
2. `Chassis_Init()`.
3. Stop for `2 s`.
4. Reset encoders.
5. Run forward PI speed control with `target=800 ticks/sample` for `6 s`.
6. Print every `200 ms`:

```text
[PI] dir=forward target=800 left_duty=... right_duty=... encA=... encB=... dA=... dB=... errL=... errR=... intL=... intR=...
```

7. Stop for `2 s`.
8. Reset encoders.
9. Run backward PI speed control with `target=800 ticks/sample` for `6 s`.
10. Print every `200 ms`:

```text
[PI] dir=backward target=800 left_duty=... right_duty=... encA=... encB=... dA=... dB=... errL=... errR=... intL=... intR=...
```

11. Stop.
12. Print `[PI] wheel speed PI test done`.

## Pass Criteria

Step6 is considered passed when:

- `APP IMU: ready=1` continues to print during the PI test.
- The I2C baseline/recovery logs still appear normally during boot.
- Forward and backward PI logs print every `200 ms`.
- `encA` and `encB` both update continuously while moving.
- `abs(dA)` and `abs(dB)` move toward `800 ticks/sample`.
- `left_duty` and `right_duty` stay within `300..600` magnitude.
- `errL` and `errR` generally decrease after startup instead of diverging.
- The final log prints `[PI] wheel speed PI test done`.

## Test Conclusion

Wheel speed PI closed-loop test passed.

Final verified test parameters:

```c
target_ticks_per_sample = 800
feedforward_duty = 500
duty_min = 300
duty_max = 600
```

Observed behavior:

- Forward PI and Backward PI both ran to completion.
- During startup, duty briefly reached the upper limit `600`.
- After the startup transient, duty stabilized at about `470..500`.
- In the later part of both forward and backward runs, encoder deltas were mostly stable at about `810..830 ticks/sample`, close to `target=800`.
- `APP IMU: ready=1` remained valid during the PI test.
- I2C baseline remained normal: `SCL=1`, `SDA=1`.

## Known Limitations

- The feedback unit is still ticks per `200 ms`, not RPM or chassis velocity.
- There is no acceleration ramp, so the first samples may saturate at `600`.
- Integral anti-windup is only a simple clamp.
- There is no derivative term.
- The same PI parameters are used for forward and backward motion.
- Actual wheel speed is still slightly higher than the target; later tuning can reduce `feedforward_duty` or `Ki`.
- UART logs may still interleave with IMU logs if both tasks print at the same time; a log mutex can be added later.
- The test is a one-shot bring-up sequence, not yet a reusable high-level velocity command interface.

## Next Stage

After Step6, continue with one of these paths:

- tune PI parameters for tighter tracking around `800 ticks/sample`;
- add IMU yaw heading hold above the wheel speed PI layer.
