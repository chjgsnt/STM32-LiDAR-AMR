# Progress Report

Last updated: 2026-05-20

## Bring-Up Summary

- Step 0: Baseline project structure created.
- Step 1: STM32CubeMX configuration checked for core bring-up peripherals and middleware.
- Step 2: Minimal hardware bring-up verified with USART2 logging, FreeRTOS startup, I2C1 scan, and basic I2C module checks.
- Step 3: AT8236 motor driver, Motor A/B PWM forward/reverse, and TIM2/TIM4 encoder counting verified.
- Step 4: Chassis open-loop test completed.
- Step 5: Encoder speed balance test completed.
- Step 6: Wheel speed PI closed-loop test completed.

## Step4 Chassis Open-Loop Test

### Test Objective

- Verify chassis-level open-loop motion above the tested AT8236 motor driver.
- Confirm Motor A/B PWM forward/reverse control still works through the chassis abstraction.
- Confirm TIM2/TIM4 encoder counts update during chassis movement.
- Confirm the MPU6500 IMU remains ready during motion.
- Verify chassis `Forward`, `Backward`, `TurnLeft`, and `TurnRight` actions.

### Key Implementation

- Added and used the chassis open-loop motion layer on top of the verified motor driver path.
- Mapped Motor A as the left wheel and Motor B as the right wheel.
- Used signed wheel commands to correct physical wheel direction.
- Ran the chassis sequence without encoder speed balance or PID control.

### Test Results

- `MPU6500 IMU ready=1` remained valid during the test.
- AT8236 motor driver operation was confirmed.
- Motor A/B PWM forward/reverse operation was confirmed.
- TIM2/TIM4 encoder counting was confirmed.
- Chassis `Forward`, `Backward`, `TurnLeft`, and `TurnRight` actions were verified.
- Step4 chassis open-loop test is complete.

### Final Direction Configuration

```c
CHASSIS_LEFT_SIGN = -1
CHASSIS_RIGHT_SIGN = 1
```

### Ground Duty Selection

- Lifted-wheel direction checks can run at lower duty values.
- Ground open-loop testing selected `duty=500` as the current reliable test value.

### Known Limitation

- Open-loop `Forward` / `Backward` motion is not straight enough because the left and right wheel speeds are not yet balanced.
- This is expected before encoder speed balance and speed PID are added.

### Next Stage Plan

- Step5 Encoder Speed Balance Test is complete.
- Proceed toward closed-loop wheel speed PID for straighter chassis motion.
- Optionally clean up UART logging with a log mutex / serialized logging path.

## Step5 Encoder Speed Balance Test

### Test Objective

- Estimate left and right wheel speed from TIM2/TIM4 encoder count deltas.
- Verify a first-pass P-only left/right wheel speed balance above the chassis open-loop layer.
- Confirm the MPU6500 IMU remains ready during the balance test.
- Improve straight-line `Forward` / `Backward` behavior before adding full speed PID.

### Key Implementation

- Used `base_duty=500` for the forward/backward balance test.
- Applied static duty trims:

```c
CHASSIS_LEFT_TRIM = -10
CHASSIS_RIGHT_TRIM = 10
```

- Used P-only correction from the left/right encoder delta error.
- Kept the final chassis direction configuration unchanged:

```c
CHASSIS_LEFT_SIGN = -1
CHASSIS_RIGHT_SIGN = 1
```

### Test Results

- Encoder speed balance test passed.
- P-only balance can reduce left/right wheel speed difference.
- During the second half of forward motion, `err` can usually be reduced to about `+/-10 ticks/sample`.
- Backward motion can run, but `err` fluctuates slightly more than forward motion.
- `MPU6500 IMU ready=1` remained valid during the test.

### Known Issue

- UART logs may interleave when IMU and BALANCE logs print at the same time.

### Next Stage Plan

- Add closed-loop wheel speed PID for more stable forward/backward motion.
- Continue optimizing backward speed balance in the PID stage.
- Optionally add a log mutex or serialized logging path to avoid interleaved UART messages.

## Step6 Wheel Speed PI Closed-Loop Test

### Test Objective

- Verify independent PI speed control for the left and right wheels.
- Use encoder delta magnitude as wheel speed feedback.
- Confirm Forward PI and Backward PI can complete with the chassis on the ground.
- Confirm the MPU6500 IMU and I2C baseline remain healthy during the PI test.

### Key Implementation

- Used `target_ticks_per_sample=800`.
- Used `feedforward_duty=500`.
- Limited PI output duty magnitude to `300..600`.
- Ran each direction with a `200 ms` control period.
- Calculated speed feedback from `abs(deltaA)` and `abs(deltaB)`.

### Test Results

- Wheel speed PI closed-loop test passed.
- Forward PI and Backward PI both ran to completion.
- During startup, duty briefly reached the upper limit `600`.
- After the startup transient, duty stabilized at about `470..500`.
- In the later part of both forward and backward runs, encoder deltas were mostly stable at about `810..830 ticks/sample`, close to `target=800`.
- `APP IMU: ready=1` remained valid during the PI test.
- I2C baseline remained normal: `SCL=1`, `SDA=1`.

### Known Limitations

- Actual wheel speed is still slightly higher than the target; later tuning can reduce `feedforward_duty` or `Ki`.
- UART logs may occasionally interleave with IMU logs; a log mutex can be added later.

### Next Stage Plan

- Tune PI parameters for tighter speed tracking.
- Or add IMU yaw heading hold above the wheel speed PI layer.
