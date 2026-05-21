# Pinout Decision

Date: 2026-05-18

Project: Fully Embedded 2D-LiDAR based Autonomous Mobile Robot

Source: motor driver board wiring diagram provided during bring-up planning.

## Decision Summary

The motor driver board has a fixed wiring layout, so the firmware will follow the board pinout instead of moving the motor outputs to the earlier generic TIM1 proposal. This avoids rewiring an already connected chassis and keeps the first hardware tests focused.

USART2 on `PA2/PA3` is kept for ST-LINK virtual COM debug logs. The driver board ADC voltage output shown on `PA2` is therefore not connected during early bring-up.

RPLIDAR C1 is assigned to `UART4` on `PC10/PC11`, so the validated motor-related pins do not change. USART1 remains reserved for debug logging and is not used for LiDAR in the current UART4 bring-up stage.

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

The current LiDAR bring-up uses the SLAMTEC RPLIDAR C1 TTL UART interface on `UART4` at `460800` baud, `8N1`.

| RPLIDAR C1 wire | RPLIDAR signal | NUCLEO pin | Peripheral | Note |
| --- | --- | --- | --- | --- |
| Red | `VCC` | `5V` | Power | LiDAR power. |
| Black | `GND` | `GND` | Power | Common ground with NUCLEO. |
| Yellow | `TX` | `PC11` | `UART4_RX` | LiDAR TX connects to MCU RX. |
| Green | `RX` | `PC10` | `UART4_TX` | LiDAR RX connects to MCU TX. |

TX/RX must be crossed: LiDAR `TX` -> MCU `RX`, and LiDAR `RX` -> MCU `TX`.

Current stage only prepares LiDAR UART bring-up. Distance parsing and obstacle avoidance are not completed yet.

## Safety Rules For First Motor Test

1. Keep the wheels off the ground for all open-loop PWM tests.
2. Start with zero PWM after reset.
3. Test Motor A and Motor B separately.
4. Start with low duty, around 10 percent, and stop immediately if direction or current looks wrong.
5. Do not connect the driver board `ADC` output to `PA2` while USART2 debug is enabled.
6. Record encoder counts, motor direction, and wiring photos in `Docs/` before closing the chassis.
