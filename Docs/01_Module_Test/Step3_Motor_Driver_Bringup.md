# Step 3 Motor Driver Bring-up

Target: STM32 NUCLEO-F446RE / STM32F446RE

Scope:

- Verify the AT8236 motor driver board.
- Verify forward and reverse PWM control for the left and right 520 encoder motors.
- Verify TIM3 PWM output on all four motor input channels.
- Verify TIM2 and TIM4 encoder counting.
- Confirm that I2C1 and the MPU6500 IMU remain healthy while the motor PWM test is running.

## Hardware Connection

| Signal | STM32F446RE / NUCLEO pin | Peripheral | Driver / motor connection |
| --- | --- | --- | --- |
| Motor A input 1 | `PA6` | `TIM3_CH1` | `AIN1` |
| Motor A input 2 | `PA7` | `TIM3_CH2` | `AIN2` |
| Motor B input 1 | `PB0` | `TIM3_CH3` | `BIN1` |
| Motor B input 2 | `PB1` | `TIM3_CH4` | `BIN2` |
| Encoder 1 phase A | `PA0` | `TIM2_CH1` | `E1A` |
| Encoder 1 phase B | `PA1` | `TIM2_CH2` | `E1B` |
| Encoder 2 phase A | `PB7` | `TIM4_CH2` | `E2A` |
| Encoder 2 phase B | `PB6` | `TIM4_CH1` | `E2B` |

Power and ground notes:

- STM32 GND and motor driver board GND must share a common ground.
- `VM` is supplied by the external motor power source.
- Do not connect the driver board `5V` to the STM32 board until the power path has been confirmed.

## Test Parameters

| Item | Value |
| --- | --- |
| Bring-up duty command | `300` |
| TIM3 ARR | `49` |
| Active CCR value | `15` |
| PWM duty | About `30%` |
| Per-direction run time | About `4 s` |

The motor bring-up sequence runs:

1. Motor A forward.
2. Motor A reverse.
3. Motor B forward.
4. Motor B reverse.

Each phase logs PWM register state and encoder counts every `500 ms`. The duty is intentionally limited to `300` for this bring-up stage.

## Passing Log Summary

Representative passing serial evidence:

```text
MPU6500 WHO_AM_I = 0x70
APP IMU: ready=1
[MOTOR] A forward ... encA from -283 to -6976
[MOTOR] A reverse ... encA from 287 to 7253
[MOTOR] B forward ... encB from 253 to 6083
[MOTOR] B reverse ... encB from -275 to -6135
[MOTOR] PWM test done
```

The motor logs include `ARR`, `CCR1`, `CCR2`, `CCR3`, `CCR4`, `pwm%`, `encA`, and `encB`. The IMU log continues to report `APP IMU: ready=1` during the motor PWM test, confirming that the I2C/MPU6500 path remains operational.

## Issues And Handling

- An earlier bring-up pass showed I2C SDA held low and HAL returning `HAL_BUSY`.
- The immediate recovery path was to disconnect the driver board wiring, disable the motor test, and restore the I2C baseline/recovery verification.
- After reconnecting the driver board, the MPU6500 remained healthy and continued reporting `APP IMU: ready=1`.
- The OLED currently still reports `not found`; this remains a separate follow-up debug item.

## Safety Notes

- Keep the wheels lifted during the first motor PWM tests.
- Do not reverse `VM` and `GND`.
- Start motor testing from a low duty value.
- Do not connect the driver board `5V` without confirming the power path.
- The motor test must call `MotorDriver_StopAll()` when it finishes.
