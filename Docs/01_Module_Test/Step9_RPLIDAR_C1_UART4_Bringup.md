# Step 9 RPLIDAR C1 UART4 Bring-up

Date: 2026-05-21

Target: STM32 NUCLEO-F446RE / STM32F446RETx

Scope:

- Record the RPLIDAR C1 UART4 wiring and CubeMX configuration.
- Prepare for raw UART reception bring-up at `460800` baud.
- Keep USART1 reserved for debug logging.
- Keep the validated motor-related pin assignments unchanged.
- Do not record distance parsing, scan parsing, mapping, or obstacle avoidance as completed in this stage.

## RPLIDAR C1 Key Parameters

| Item | Value |
| --- | --- |
| LiDAR model | SLAMTEC RPLIDAR C1 |
| Interface | TTL UART |
| MCU | STM32F446RETx |
| Assigned UART | `UART4` |
| Baud rate | `460800` |
| UART frame format | `8N1` |
| Power | `5V` and `GND` |
| Signal rule | TX/RX must be crossed between LiDAR and MCU. |

USART1 is not used for the RPLIDAR C1 in this bring-up stage; it remains reserved for debug logging.

## UART4 CubeMX Configuration

| Setting | Value |
| --- | --- |
| Peripheral | `UART4` |
| Mode | Asynchronous |
| Baud rate | `460800` |
| Word length | 8 bits |
| Parity | None |
| Stop bits | 1 |
| TX pin | `PC10` / `UART4_TX` |
| RX pin | `PC11` / `UART4_RX` |

This stage prepares the UART interface only. Raw reception, byte-count verification, frame parsing, distance parsing, and obstacle avoidance remain future work.

## Wiring Table

| RPLIDAR C1 wire | RPLIDAR signal | STM32F446RETx / NUCLEO connection | Note |
| --- | --- | --- | --- |
| Red | `VCC` | `5V` | LiDAR power. |
| Black | `GND` | `GND` | Common ground with MCU board. |
| Yellow | `TX` | `PC11` / `UART4_RX` | LiDAR transmit connects to MCU receive. |
| Green | `RX` | `PC10` / `UART4_TX` | LiDAR receive connects to MCU transmit. |

TX/RX crossing rule:

- LiDAR `TX` -> MCU `RX`.
- LiDAR `RX` -> MCU `TX`.

## Bring-up Test Objective

The next hardware test should confirm that the MCU can receive raw bytes from the RPLIDAR C1 on `UART4` at `460800` baud. The first firmware check should be a minimal raw UART reception path that counts received bytes and reports the count through the existing debug log path.

Do not change motor-related pins or motor test configuration while preparing this LiDAR UART bring-up, because motor bring-up has already been validated.

## Current Status / Next Step

- UART4 has been configured for RPLIDAR C1.
- Hardware wiring has been completed.
- Next step is to add raw UART reception code and verify rx byte count at 460800 baud.
