#ifndef MAPPING_H
#define MAPPING_H

/*
 * Occupancy grid mapping module.
 *
 * This module will convert LiDAR and odometry data into a 2D occupancy grid
 * for local navigation.
 */

void Mapping_Init(void);
void Mapping_Update(void);
void Mapping_Task(void);

#endif /* MAPPING_H */
