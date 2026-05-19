# Step 2 Minimal Bring-up Test

Date: 2026-05-18

Target: STM32 NUCLEO-F446RE

Scope:

- Retarget `printf` to USART2 using `HAL_UART_Transmit`.
- Print basic boot messages with `HAL_GetTick()` timestamps.
- Toggle LD2 on PA5 every 1000 ms in `defaultTask`.
- Scan I2C1 once with `HAL_I2C_IsDeviceReady`.
- Detect common SSD1306 OLED addresses `0x3C` and `0x3D`.

## Expected Serial Output

```text
[000123 ms] System start!
[000240 ms] FreeRTOS started.
[000510 ms] I2C scan started.
[000620 ms] I2C device found at 0x3C.
[000620 ms] OLED found at 0x3C.
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

- USART2 should be monitored at 115200 8N1.
- I2C1 uses PB8 as SCL and PB9 as SDA.
- LD2 on PA5 should toggle once per second while `defaultTask` is running.
- The I2C bus needs pull-up resistors. Many SSD1306 and MPU6500 breakout boards include them, but bare modules may not.
- Check OLED address solder jumpers if neither `0x3C` nor `0x3D` appears.
- Confirm all modules share GND with the NUCLEO board.
