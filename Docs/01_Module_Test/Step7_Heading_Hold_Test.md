# Step 7 Heading Hold Straight-Line Test

Target: STM32 NUCLEO-F446RE / STM32F446RE

Scope:

- Add a short-duration straight-line heading hold test above the Step6 wheel speed PI loop.
- Estimate relative yaw by integrating MPU6500 gyro Z.
- Adjust left and right wheel speed targets to reduce yaw error.
- Keep the verified Step3 through Step6 tests available.
- Keep the MPU6500 I2C baseline/recovery path unchanged.

## Goal

Step6 verified that each wheel can track a speed target from encoder deltas. Step7 adds a heading correction layer so the chassis can better hold a straight line during forward and backward motion.

This stage is a short bring-up test, not an absolute navigation heading system. The MPU6500 has no magnetometer, so yaw is only a relative estimate from gyro integration.

## Why Heading Hold After Speed PI

Wheel speed PI can make each wheel approach `target_ticks_per_sample=800`, but straight-line motion can still drift because of:

- small wheel diameter differences;
- floor friction differences;
- left/right gearbox mismatch;
- chassis alignment;
- battery/load changes;
- short-term PI tracking mismatch.

Heading hold closes the loop around the chassis yaw. Instead of only commanding equal wheel speeds, it measures relative yaw drift and biases the left/right wheel speed targets.

## Gyro Z Yaw Integration

The current MPU6500 pipeline already calibrates gyro zero bias and exposes `gz_dps` through `MPU6500_GetLatest()`.

Step7 uses the latest bias-corrected gyro Z rate:

```text
gyro_z_dps = imu.gz_dps
dt = imu.last_update_ms - last_imu_update_ms
yaw_deg += gyro_z_dps * dt_seconds
```

The yaw estimate is relative. The test records:

```text
target_yaw = current_yaw
```

before each direction run. The controller then tries to keep yaw near that value for the next `6 s`.

## Heading Correction Formula

Heading error:

```text
yaw_error = current_yaw - target_yaw
```

Correction:

```text
k_heading = HEADING_K_FORWARD for forward
k_heading = HEADING_K_BACKWARD for backward
heading_correction = k_heading * yaw_error
corr_limit = HEADING_CORR_LIMIT_FORWARD for forward
corr_limit = HEADING_CORR_LIMIT_BACKWARD for backward
heading_correction = clamp(heading_correction, -corr_limit, +corr_limit)
```

Initial parameters:

```c
HEADING_K_FORWARD = 2 ticks/sample per degree
HEADING_K_BACKWARD = 2 ticks/sample per degree
HEADING_CORR_LIMIT_FORWARD = 50
HEADING_CORR_LIMIT_BACKWARD = 50
base_target = 800 ticks/sample
target_limit = 600..900 ticks/sample
```

The correction is clamped to the direction-specific `corr_limit`.

## Notes

- The old `corr_sign` formula was easy to confuse because the yaw-error direction and chassis turn direction were mixed in one signed value.
- Step7 now uses `yaw_error = current_yaw - target_yaw`.
- The left/right target adjustment is explicit: `left_target = base_target + heading_correction`, `right_target = base_target - heading_correction`.
- The explicit current-minus-target formula improved stability enough to keep Step7 as a useful saved baseline.
- Current tuning uses conservative `K=2` and `corr_limit=50` for both forward and backward.
- Forward heading hold is clearly improved, with final yaw around `7.1 deg`.
- Backward heading hold can run, but final error is larger at about `+7 deg` relative to its target.
- Backward heading hold still needs tuning.

Left/right wheel speed targets:

```text
left_target = base_target + heading_correction
right_target = base_target - heading_correction
```

Both targets are clamped to `600..900 ticks/sample`.

## Wheel Speed PI Reuse

Step7 reuses the Step6 `wheel_speed_controller` module. Each wheel receives its own target:

```text
left wheel PI target = left_target
right wheel PI target = right_target
```

The wheel speed PI duty output remains limited to `300..600`.

## Software Switches

Step7 enables only the heading hold test:

```c
#define APP_ENABLE_MOTOR_TEST 0
#define APP_ENABLE_MOTOR_GPIO_STATIC_TEST 0
#define APP_ENABLE_CHASSIS_OPENLOOP_TEST 0
#define APP_ENABLE_CHASSIS_DIRECTION_CAL_TEST 0
#define APP_ENABLE_CHASSIS_GROUND_TRACTION_TEST 0
#define APP_ENABLE_CHASSIS_SPEED_BALANCE_TEST 0
#define APP_ENABLE_WHEEL_SPEED_PI_TEST 0
#define APP_ENABLE_HEADING_HOLD_TEST 1
```

## Test Flow

The FreeRTOS control task runs this one-time sequence:

1. Print `[HEADING] heading hold test start`.
2. `Chassis_Init()`.
3. Stop for `2 s`.
4. Record `target_yaw = current_yaw`.
5. Run forward heading hold with `target_speed=800 ticks/sample` for `6 s`.
6. Print every `200 ms`:

```text
[HEADING] dir=forward target_yaw=... yaw=... yaw_error_current_minus_target=... k_heading=... corr_limit=... heading_corr=... left_target=... right_target=... dA=... dB=... left_duty=... right_duty=...
```

7. Stop for `2 s`.
8. Record `target_yaw = current_yaw` again.
9. Run backward heading hold with `target_speed=800 ticks/sample` for `6 s`.
10. Print every `200 ms`:

```text
[HEADING] dir=backward target_yaw=... yaw=... yaw_error_current_minus_target=... k_heading=... corr_limit=... heading_corr=... left_target=... right_target=... dA=... dB=... left_duty=... right_duty=...
```

11. Stop and call `MotorDriver_StopAll()`.
12. Print `[HEADING] heading hold test done`.

## Current Test Result

- The test uses gyro-Z yaw integration from the MPU6500.
- Heading error is calculated as `yaw_error = current_yaw - target_yaw`.
- The explicit target correction is `left_target = base_target + heading_correction`, `right_target = base_target - heading_correction`.
- Current parameters are `HEADING_K_FORWARD=2`, `HEADING_K_BACKWARD=2`, `HEADING_CORR_LIMIT_FORWARD=50`, and `HEADING_CORR_LIMIT_BACKWARD=50`.
- Forward heading hold is clearly improved; the final yaw is about `7.1 deg`.
- Backward heading hold can run, but the final error is about `+7 deg` relative to the backward target.
- `APP IMU: ready=1` remained valid and the I2C baseline stayed normal.

## Safety Notes

- Run the test on a clear floor area.
- Keep a hand near the chassis during the first run.
- For the first cautious test, `HEADING_HOLD_BASE_TARGET_TICKS_PER_SAMPLE` can be temporarily reduced from `800` to `700`.
- Confirm the final stop happens and motor PWM is disabled before lifting or handling the robot.

## Pass Criteria

Step7 is considered passed when:

- `APP IMU: ready=1` continues to print during the test.
- I2C baseline/recovery logs remain normal during boot.
- Forward and backward heading logs print every `200 ms`.
- `yaw` updates smoothly and does not jump unexpectedly.
- `yaw_error_current_minus_target` stays bounded during each `6 s` run.
- `heading_corr` stays within the direction-specific `corr_limit`.
- `left_target` and `right_target` stay within `600..900 ticks/sample`.
- `left_duty` and `right_duty` stay within `300..600` magnitude.
- The chassis visibly holds a straighter line than speed PI alone.
- The final log prints `[HEADING] heading hold test done`.

## Known Limitations

- Gyro-only yaw will drift over time.
- There is no magnetometer or absolute heading reference.
- Yaw integration uses the latest sampled gyro Z value, so it is suitable for this short test but not long-duration navigation.
- The old `corr_sign` formula was easy to confuse, so Step7 now uses explicit left/right target correction.
- Backward heading hold still needs tuning.
- There is no acceleration ramp yet.
- UART logs may still interleave with IMU logs.
- Next work can tune backward heading correction or add odometry-yaw fusion.
- Future work can also fuse yaw with LiDAR scan matching or a higher-level localization estimate.
