# MPU6500 IMU Bring-up Verification

Date: 2026-05-19

Target: STM32 NUCLEO-F446RE / STM32F446RE

Scope:

- Record the current MPU6500 IMU hardware bring-up status.
- Record I2C1 wiring, device identity check, sensor data conversion, attitude estimation, and App-layer integration.
- No firmware logic changes are covered by this document update.

## Hardware Connection

Current validation setup connects only the MPU6500 module on I2C1. The OLED module is not connected for this IMU verification pass.

| MPU6500 module pin | STM32F446RE / NUCLEO connection | Note |
| --- | --- | --- |
| `VCC` | `3.3V` | Use 3.3 V logic and power. |
| `GND` | `GND` | Common ground with the NUCLEO board. |
| `SCL` | `PB8` | `I2C1_SCL`. |
| `SDA` | `PB9` | `I2C1_SDA`. |
| `AD0` | `GND` or default low level | 7-bit I2C address is `0x68`. |
| `CS` / `NCS`, if present | `3.3V` | Pull high to select I2C mode on modules that expose this pin. |

I2C1 is configured on `PB8/PB9`. Confirm that SDA/SCL have pull-up resistors; many MPU6500 breakout boards include them, but bare modules may require external pull-ups.

## Verified Results

| Item | Result |
| --- | --- |
| I2C scan | OK, found device at `0x68`. |
| WHO_AM_I | OK, `WHO_AM_I = 0x70`. |
| Raw accelerometer / gyroscope data | OK. |
| Accelerometer `g` conversion | OK. |
| Gyroscope `dps` conversion | OK. |
| Gyro bias calibration | OK. |
| Accelerometer pitch / roll | OK. |
| Complementary filter fused pitch / roll | OK. |
| App-layer IMU readout | OK through `MPU6500_GetData()` or `MPU6500_GetLatest()`. |

Pitch and roll are available from the current IMU path. Yaw is not implemented yet because this setup has no magnetometer or other external heading reference.

## Default Serial Output

The default periodic serial output is the App-layer IMU status. A representative steady-state line is:

```text
APP IMU: ready=1 pitch=-1.1deg roll=0.0deg
```

Startup logs may include the I2C scan, `WHO_AM_I`, wake-up, and gyro calibration messages before the periodic App-layer line appears.

## Logging Notes

MPU6500 internal detail logs are disabled by default. They can be enabled with the compile-time macros in `AMR_LiDAR_Robot/Core/Src/i2c_scan.c`:

- `MPU6500_LOG_RAW`
- `MPU6500_LOG_UNITS`
- `MPU6500_LOG_ACCEL_ANGLE`
- `MPU6500_LOG_FUSED_ANGLE`
- `MPU6500_LOG_FILTER_DEBUG`

Leave these macros at `0` for the default concise App-layer output. Enable only the needed detail log when debugging raw samples, unit conversion, accelerometer angle, fused angle, or filter timing.

## Notes And Limitations

- Current hardware validation connects the MPU6500 only; the OLED is not connected.
- Keep the module still during gyro bias calibration.
- Pitch and roll are validated and usable.
- Yaw is currently unavailable without a magnetometer or another external heading reference.
- If I2C scan does not find `0x68`, check `AD0`, `CS/NCS`, pull-ups, and common ground first.
