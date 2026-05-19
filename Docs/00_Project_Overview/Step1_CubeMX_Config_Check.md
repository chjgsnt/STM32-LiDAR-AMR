# Step 1 CubeMX Configuration Check

Date: 2026-05-18

Reviewed again: 2026-05-19

Project: Fully Embedded 2D-LiDAR based Autonomous Mobile Robot

Target board: STM32 NUCLEO-F446RE

CubeMX project file: `AMR_LiDAR_Robot/AMR_LiDAR_Robot.ioc`

Scope:

- Checked CubeMX `.ioc` settings and generated initialization code.
- Did not rewrite generated `Core`, `Drivers`, or `Middlewares` files.
- Did not add complete driver logic.

## Configuration Summary

| Item | Status | Detected result |
| --- | --- | --- |
| SYS Debug | PASS with traceability note | PA13/PA14 are configured as Serial Wire SWD pins. PB3 is reserved as SWO. No standalone `SYS.Debug` key was found in the `.ioc`; the pin modes provide the trace. |
| RCC / Clock | PASS with notes | SYSCLK is configured from HSI + PLL at 180 MHz. APB1 is 45 MHz, APB2 is 90 MHz, OverDrive is enabled. The `.ioc` also reserves HSE/LSE oscillator pins, but generated clock code currently uses HSI. |
| USART2 debug log | PASS with traceability note | Generated code uses USART2 at 115200 8N1 on PA2/PA3. `.ioc` stores asynchronous mode, but the default 115200 baud is not explicitly listed. |
| USART1 RPLidar C1 | PASS | `.ioc` stores USART1 baud rate 460800. Generated code uses 460800 8N1 on PA9/PA10. |
| USART1 RX DMA | PASS | USART1_RX uses DMA2 Stream2, Channel 4, peripheral-to-memory, circular mode, high priority. |
| I2C1 | PASS with hardware note | I2C1 is enabled in fast mode at 400 kHz on PB8/PB9. Generated GPIO pull setting is `GPIO_NOPULL`, so external pull-ups should be present. |
| FreeRTOS CMSIS_V2 | PASS | FreeRTOS is enabled through CMSIS-RTOS V2. Generated tasks include default, lidar, imu, oled, and control tasks. |
| USART1 / DMA NVIC | PASS | USART1_IRQn and DMA2_Stream2_IRQn are enabled at priority 5. This matches the FreeRTOS max syscall priority boundary. |
| User directories | PASS | `Firmware`, `Docs`, `Tools`, and `Tests` remain present at the repository root. |

## Notes And Recommendations

1. USART2 and USART1 generated code confirms 8N1 settings. CubeMX may omit default UART fields from the `.ioc`, so keep the generated code check as part of the review record.
2. The current 180 MHz clock setup is valid for STM32F446RE and includes OverDrive. If high-baud LiDAR communication becomes unstable, consider switching CubeMX to HSE bypass or another stable external clock source, or explicitly document that HSI is intentional.
3. The PLL 48 MHz domain does not appear to be set for an exact 48 MHz peripheral clock. This is acceptable while USB, RNG, or SDIO are unused, but should be fixed before enabling those peripherals.
4. I2C1 is configured for 400 kHz. Confirm that the MPU6500 and SSD1306 modules have pull-up resistors on SDA/SCL, or add external pull-ups on the bus.
5. If HSE/LSE oscillator pins remain reserved in CubeMX, keep the clock-source decision explicit in the project notes so the pinout and generated clock code do not appear to disagree.
6. Keep future application edits inside user-owned files or CubeMX `USER CODE` blocks only.
7. Step 2 can start with a minimal UART2 debug log smoke test and a USART1 DMA receive buffer smoke test, without implementing the full LiDAR parser yet.

## Update: Motor Driver Pinout Alignment

Date: 2026-05-18

After reviewing the motor driver board wiring diagram, the motor peripheral plan was aligned to the actual board pinout:

| Function | Pin(s) | Peripheral |
| --- | --- | --- |
| Motor A encoder | `PA0/PA1` | `TIM2_CH1/CH2` encoder mode |
| Motor B encoder | `PB6/PB7` | `TIM4_CH1/CH2` encoder mode |
| Motor A dual PWM | `PA6/PA7` | `TIM3_CH1/CH2` PWM, 20 kHz |
| Motor B dual PWM | `PB0/PB1` | `TIM3_CH3/CH4` PWM, 20 kHz |

USART2 on `PA2/PA3` remains reserved for ST-LINK virtual COM debug output. The motor driver board `ADC` voltage output shown on `PA2` is intentionally left unconnected for early bring-up.
