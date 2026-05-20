# Progress Report

Last updated: 2026-05-20

## Bring-Up Summary

- Step 0: Baseline project structure created.
- Step 1: STM32CubeMX configuration checked for core bring-up peripherals and middleware.
- Step 2: Minimal hardware bring-up verified with USART2 logging, FreeRTOS startup, I2C1 scan, and basic I2C module checks.
- Step 3: AT8236 motor driver, Motor A/B PWM forward/reverse, and TIM2/TIM4 encoder counting verified.
- Step 4: Chassis open-loop test completed.
- Step 5: Encoder speed balance test completed.

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
