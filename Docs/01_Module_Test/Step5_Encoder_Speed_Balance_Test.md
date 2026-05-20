# Step 5 Encoder Speed Balance Test

Target: STM32 NUCLEO-F446RE / STM32F446RE

Scope:

- Estimate left and right wheel speed from encoder count deltas.
- Add a simple P-only speed balance test above the verified chassis open-loop layer.
- Improve straight-line `Forward` / `Backward` behavior without adding full PID yet.
- Keep the MPU6500 IMU and I2C1 baseline/recovery path unchanged.

## Goal

Step4 verified that the chassis can move on the ground with open-loop `duty=500`, but the chassis does not drive straight enough. Step5 uses TIM2/TIM4 encoder feedback to compare wheel speed and adjust left/right duty during a short test run.

This step is not a full speed controller. It is a bring-up test for encoder speed estimation and first-pass left/right wheel balancing.

Status: passed.

## Why Open-Loop Does Not Drive Straight

In open-loop mode, both wheels receive the same duty command, but the actual speed can still differ because of:

- motor manufacturing tolerance;
- wheel friction and ground contact differences;
- gearbox and bearing differences;
- battery voltage sag under load;
- weight distribution and chassis alignment.

The Step4 logs showed different left/right encoder counts during forward/backward movement. That means equal duty does not produce equal wheel speed.

## Encoder Delta Speed Estimate

Step5 samples encoder counts every `200 ms`.

For each sample:

```text
deltaA = encA_now - encA_last
deltaB = encB_now - encB_last
speed_left = abs(deltaA)
speed_right = abs(deltaB)
```

The current test uses ticks per sample as the speed estimate. RPM conversion can be added later with:

```text
rpm = delta_count / encoder_counts_per_rev / dt_seconds * 60
```

For the first RPM conversion pass, use:

```text
encoder_counts_per_rev = 390
dt_seconds = 0.2
```

## P-Only Balance Algorithm

The balance test compares absolute encoder deltas:

```text
err = speed_left - speed_right
correction = Kp * err
left_cmd = base_duty - correction
right_cmd = base_duty + correction
```

Current initial values:

```c
base_duty = 500
Kp = 1
correction limit = +/-100
duty magnitude limit = 300..600
```

If the left wheel is faster, `err` is positive, so left duty is reduced and right duty is increased. If the right wheel is faster, the correction moves in the opposite direction.

After the initial P-only balance test, the chassis completed the forward/backward balance sequence and `APP IMU: ready=1` stayed valid, but the left wheel was still slightly faster than the right wheel:

```text
forward:  dA ~= 850,  dB ~= 830..845, err ~= 10..20
backward: dA ~= -850, dB ~= -830..-844, err ~= 10..24
```

To remove this small residual bias without adding a full PID loop, Step5 now applies a small static trim before the final duty limit:

```c
CHASSIS_LEFT_TRIM = -10
CHASSIS_RIGHT_TRIM = 10
```

The trim slightly reduces the left wheel duty magnitude and slightly increases the right wheel duty magnitude. The P-only correction limit remains unchanged, and the final duty magnitude is still limited to `300..600`.

`Chassis_SetRaw(left, right)` still applies `CHASSIS_LEFT_SIGN` and `CHASSIS_RIGHT_SIGN`, so the balance code works in chassis-level left/right semantics only.

Final direction configuration remains:

```c
CHASSIS_LEFT_SIGN = -1
CHASSIS_RIGHT_SIGN = 1
```

## Software Switches

Step5 enables only the speed balance test:

```c
#define APP_ENABLE_MOTOR_TEST 0
#define APP_ENABLE_MOTOR_GPIO_STATIC_TEST 0
#define APP_ENABLE_CHASSIS_OPENLOOP_TEST 0
#define APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST 0
#define APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST 0
#define APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST 1
```

## Test Flow

The FreeRTOS control task runs this one-time sequence:

1. Print `[BALANCE] speed balance test start`.
2. `Chassis_Init()`.
3. Stop for `2 s`.
4. Reset encoders.
5. Run forward balanced test with `base_duty=500` for `5 s`.
6. Print every `200 ms`:

```text
[BALANCE] dir=forward base=500 left_trim=-10 right_trim=10 left_duty=... right_duty=... encA=... encB=... dA=... dB=... err=...
```

7. Stop for `2 s`.
8. Reset encoders.
9. Run backward balanced test with `base_duty=500` for `5 s`.
10. Print every `200 ms`:

```text
[BALANCE] dir=backward base=500 left_trim=-10 right_trim=10 left_duty=... right_duty=... encA=... encB=... dA=... dB=... err=...
```

11. Stop.
12. Print `[BALANCE] speed balance test done`.

## Pass Criteria

Step5 is considered passed when:

- `APP IMU: ready=1` continues to print during the test.
- Forward and backward balance logs print every `200 ms`.
- `encA` and `encB` both update continuously while moving.
- `dA` and `dB` are non-zero and usable as speed estimates.
- `err` generally moves toward a smaller magnitude compared with raw open-loop behavior.
- The chassis drives straighter than Step4 open-loop at the same `base_duty=500`.
- The final log prints `[BALANCE] speed balance test done`.

## Test Conclusion

Encoder speed balance test passed.

Final verified test parameters:

```c
base_duty = 500
CHASSIS_LEFT_TRIM = -10
CHASSIS_RIGHT_TRIM = 10
```

Observed behavior:

- P-only balance can reduce left/right wheel speed difference.
- During the second half of the forward run, `err` can usually be reduced to about `+/-10 ticks/sample`.
- Backward motion can run, but `err` fluctuates slightly more than forward motion. This will continue to be optimized in the later PID stage.
- `APP IMU: ready=1` remained valid during the test.

## Current Limitations

- This is P-only wheel speed balance, not full PID.
- There is no integral correction, derivative damping, target RPM conversion, or acceleration ramp yet.
- It balances left/right wheel speed only during the test sequence.
- UART logs may interleave when IMU and BALANCE logs print at the same time.
- Future work can extend this into a reusable closed-loop speed PID module.

## Next Stage

After Step5, proceed toward a proper speed closed loop or clean up logging concurrency:

- convert ticks per sample to RPM or chassis speed units;
- tune per-wheel speed PID;
- add acceleration limiting;
- use the speed loop from higher-level chassis velocity commands.
- optionally add a log mutex or serialized logging path to avoid interleaved UART messages.
