#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * RPLidar packet parser module.
 *
 * This module will convert raw RPLidar C1 byte streams into validated scan
 * samples for mapping and navigation.
 */

void LidarParser_Reset(void);
bool LidarParser_ParseByte(uint8_t byte);

#endif /* LIDAR_PARSER_H */
