# Pinout Decision

Date: 2026-05-18

Project: Fully Embedded 2D-LiDAR based Autonomous Mobile Robot

Source: motor driver board wiring diagram provided during bring-up planning.

## Decision Summary

The motor driver board has a fixed wiring layout, so the firmware will follow the board pinout instead of moving the motor outputs to the earlier generic TIM1 proposal. This avoids rewiring an already connected chassis and keeps the first hardware tests focused.

USART2 on `PA2/PA3` is kept for ST-LINK virtual COM debug logs. The driver board ADC voltage output shown on `PA2` is therefore not connected during early bring-up.

## Motor Driver Pinout

| Function | Driver board label | NUCLEO pin | STM32 peripheral | Bring-up note |
| --- | --- | --- | --- | --- |
| Motor A encoder A | `E1A` | `PA0` | `TIM2_CH1` encoder | Direction must be calibrated. |
| Motor A encoder B | `E1B` | `PA1` | `TIM2_CH2` encoder | Direction must be calibrated. |
| Motor B encoder A | `E2A` | `PB7` | `TIM4_CH2` encoder | Direction must be calibrated. |
| Motor B encoder B | `E2B` | `PB6` | `TIM4_CH1` encoder | Direction must be calibrated. |
| Motor A input 1 | `AIN1` | `PA6` | `TIM3_CH1` PWM | Dual-PWM motor input, 20 kHz. |
| Motor A input 2 | `AIN2` | `PA7` | `TIM3_CH2` PWM | Dual-PWM motor input, 20 kHz. |
| Motor B input 1 | `BIN1` | `PB0` | `TIM3_CH3` PWM | Dual-PWM motor input, 20 kHz. |
| Motor B input 2 | `BIN2` | `PB1` | `TIM3_CH4` PWM | Dual-PWM motor input, 20 kHz. |
| Driver logic 5V | `5V` | NUCLEO `5V` | Power | Confirm the driver board power path before enabling motors. |
| Driver ground | `GND` | NUCLEO `GND` | Power | Common ground is mandatory. |
| Driver voltage sense | `ADC` | Not connected | Reserved | Do not connect to `PA2`; USART2 debug uses `PA2`. |

## Other Early Bring-up Connections

### OLED

| OLED pin | NUCLEO pin | Peripheral |
| --- | --- | --- |
| `VCC` | `3V3` | Power |
| `GND` | `GND` | Power |
| `SCL` | `PB8` | `I2C1_SCL` |
| `SDA` | `PB9` | `I2C1_SDA` |

### RPLIDAR C1

The current firmware keeps LiDAR on USART1 to avoid disturbing the motor driver pinout.

| RPLIDAR C1 pin | NUCLEO pin | Peripheral | Note |
| --- | --- | --- | --- |
| `5V` | Stable external `5V` | Power | Need startup current margin, about 800 mA. |
| `GND` | NUCLEO `GND` | Power | Common ground with NUCLEO. |
| `TX` | `PA10` | `USART1_RX` + DMA | MCU receives LiDAR data. |
| `RX` | `PA9` | `USART1_TX` | MCU sends commands. |

## Safety Rules For First Motor Test

1. Keep the wheels off the ground for all open-loop PWM tests.
2. Start with zero PWM after reset.
3. Test Motor A and Motor B separately.
4. Start with low duty, around 10 percent, and stop immediately if direction or current looks wrong.
5. Do not connect the driver board `ADC` output to `PA2` while USART2 debug is enabled.
6. Record encoder counts, motor direction, and wiring photos in `Docs/` before closing the chassis.
