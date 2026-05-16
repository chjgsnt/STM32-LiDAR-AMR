#ifndef MPU6500_H
#define MPU6500_H

/*
 * MPU6500 IMU module.
 *
 * This module will initialize the IMU and provide accelerometer, gyroscope,
 * and orientation-related data to the application.
 */

void MPU6500_Init(void);
void MPU6500_Task(void);

#endif /* MPU6500_H */
