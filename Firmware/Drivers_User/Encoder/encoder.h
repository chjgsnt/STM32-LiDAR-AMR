#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/*
 * Encoder feedback module.
 *
 * This module will read wheel encoder counts and expose odometry-related
 * measurements to control and navigation layers.
 */

void Encoder_Init(void);
void Encoder_Reset(void);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

#endif /* ENCODER_H */
