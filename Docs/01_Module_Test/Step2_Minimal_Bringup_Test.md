# Step 2 Minimal Bring-up Test

Date: 2026-05-19

Target: STM32 NUCLEO-F446RE / STM32F446

Scope:

- Retarget `printf` to USART2 using `HAL_UART_Transmit`.
- Print basic boot messages with `HAL_GetTick()` timestamps.
- Toggle LD2 on PA5 every 1000 ms in `defaultTask`.
- Scan I2C1 once with `HAL_I2C_IsDeviceReady`.
- Detect common SSD1306 OLED addresses `0x3C` and `0x3D`.
- Initialize the SSD1306 OLED at `0x3C` and show a minimal display test.

## Current Hardware Verification Progress

The firmware builds successfully with the `Debug` CMake preset, the STM32F446 board can be programmed through ST-LINK, UART serial logging works on `COM11`, and FreeRTOS starts successfully.

The I2C1 bus was verified using `PB8` as SCL and `PB9` as SDA. Before connecting external devices, both lines were confirmed to be high, indicating an idle I2C bus. After connecting the OLED module, the firmware detected an I2C device at 7-bit address `0x3C`. A minimal SSD1306 display test was then added, and the OLED successfully displayed `OLED OK` and `I2C: 0x3C`. This confirms that the OLED wiring, I2C communication, and basic SSD1306 display output are working correctly.

| OLED pin | STM32F446 / NUCLEO connection | Note |
| --- | --- | --- |
| `VCC` | `3.3V` | OLED power |
| `GND` | `GND` | Common ground |
| `SCL` | `PB8` | `I2C1_SCL` |
| `SDA` | `PB9` | `I2C1_SDA` |

Recent OLED test firmware changes:

- `AMR_LiDAR_Robot/Core/Inc/ssd1306.h`
- `AMR_LiDAR_Robot/Core/Src/ssd1306.c`
- `AMR_LiDAR_Robot/Core/Src/i2c_scan.c`
- `AMR_LiDAR_Robot/CMakeLists.txt`

## Expected Serial Output

```text
[000123 ms] System start!
[000240 ms] FreeRTOS started.
[000510 ms] I2C scan started.
[000620 ms] I2C device found at 0x3C.
[000620 ms] OLED found at 0x3C.
[0008xx ms] OLED display test written at 0x3C.
[000900 ms] I2C scan finished.
```

If no I2C devices respond:

```text
[000123 ms] System start!
[000240 ms] FreeRTOS started.
[000510 ms] I2C scan started.
[001800 ms] No I2C devices found.
[001800 ms] OLED not found.
[001800 ms] I2C scan finished.
```

## Hardware Notes

- USART2 should be monitored on `COM11` at 115200 8N1.
- I2C1 uses PB8 as SCL and PB9 as SDA.
- With no I2C devices connected, SCL and SDA should both read high.
- LD2 on PA5 should toggle once per second while `defaultTask` is running.
- The I2C bus needs pull-up resistors. Many SSD1306 and MPU6500 breakout boards include them, but bare modules may not.
- Check OLED address solder jumpers if neither `0x3C` nor `0x3D` appears.
- Confirm all modules share GND with the NUCLEO board.
