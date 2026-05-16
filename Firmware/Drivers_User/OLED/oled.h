#ifndef OLED_H
#define OLED_H

/*
 * SSD1306 OLED module.
 *
 * This module will provide basic display initialization and screen update
 * helpers for status, diagnostics, and test output.
 */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Task(void);

#endif /* OLED_H */
