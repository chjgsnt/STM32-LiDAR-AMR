#ifndef APP_TEST_CONFIG_H
#define APP_TEST_CONFIG_H

/*
 * TEST OPERATOR SWITCH: LiDAR obstacle motor linkage.
 *
 * Default MUST stay 0. With 0, the obstacle motor task is dry-run only and
 * must not call chassis or motor output functions.
 *
 * Change to 1 only for a temporary wheels-off-ground low-speed test after
 * confirming the robot is physically safe to run.
 *
 * Ground testing is not allowed until the wheels-off-ground test is completed.
 *
 * After testing, change this value back to 0.
 *
 * Do not commit or push an enabled=1 version to GitHub.
 */
#define APP_OBSTACLE_MOTOR_ENABLE 0

#if APP_OBSTACLE_MOTOR_ENABLE
#warning "APP_OBSTACLE_MOTOR_ENABLE is ON: lift wheels before testing"
#endif

#endif /* APP_TEST_CONFIG_H */
