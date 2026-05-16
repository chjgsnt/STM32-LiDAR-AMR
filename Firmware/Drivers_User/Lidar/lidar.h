#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

/*
 * LiDAR interface module.
 *
 * This module will manage RPLidar C1 transport, scan control, and raw byte
 * forwarding to the parser.
 */

void Lidar_Init(void);
void Lidar_Task(void);
void Lidar_ProcessRxByte(uint8_t byte);

#endif /* LIDAR_H */
