# Pinout Alignment Milestone

Date: 2026-05-18

## Scope

This milestone records the decision to follow the actual motor driver board wiring diagram and align the STM32 timer configuration before connecting the remaining peripherals.

## Completed

- Added `Docs/00_Project_Overview/Pinout_Decision.md`.
- Reserved `PA2/PA3` for USART2 debug logs.
- Marked the motor driver board `ADC` output as not connected during early bring-up.
- Added TIM2 encoder configuration for Motor A on `PA0/PA1`.
- Kept TIM4 encoder configuration for Motor B on `PB6/PB7`.
- Expanded TIM3 PWM from two channels to four channels:
  - Motor A: `PA6/PA7`, `TIM3_CH1/CH2`
  - Motor B: `PB0/PB1`, `TIM3_CH3/CH4`
- Set TIM3 PWM period for 20 kHz PWM under the current 90 MHz APB1 timer clock setup.

## Next Verification

1. Build the STM32CubeIDE project.
2. Flash the board with motor power disconnected or wheels lifted.
3. Confirm USART2 boot log output.
4. Connect OLED to `PB8/PB9` and run I2C scan.
5. Keep driver board `ADC` disconnected from `PA2`.

