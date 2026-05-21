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
#ifndef APP_OBSTACLE_MOTOR_ENABLE
#define APP_OBSTACLE_MOTOR_ENABLE 0
#endif

/*
 * TEST OPERATOR SWITCH: short ground test permission.
 *
 * Default MUST stay 0. Set to 1 only after the wheels-off-ground motor linkage
 * test has passed and the robot is placed in a safe ground-test area.
 *
 * Real motor output is allowed only when BOTH switches are 1:
 *   APP_OBSTACLE_MOTOR_ENABLE == 1
 *   APP_OBSTACLE_GROUND_TEST_ENABLE == 1
 *
 * The ground test still enforces a 3 second startup delay and a 10 second
 * maximum run window before forcing STOP.
 *
 * After testing, change this value back to 0.
 *
 * Do not commit or push a ground=1 version to GitHub.
 */
#ifndef APP_OBSTACLE_GROUND_TEST_ENABLE
#define APP_OBSTACLE_GROUND_TEST_ENABLE 0
#endif

/*
 * TEST OPERATOR SPEED SETTINGS.
 *
 * AIR_TEST_SPEED is used for wheels-off-ground/dry-run command previews.
 * GROUND_TEST_SPEED is used only when both motor and ground-test switches are 1.
 *
 * Default ground speed is 350 because the robot could move on the ground at
 * this duty during short testing. If the robot is too aggressive, reduce toward
 * 320 or 280. Make only small changes each time, keep the short-test timeout
 * active, and be ready to cut power immediately.
 */
#ifndef APP_OBSTACLE_AIR_TEST_SPEED
#define APP_OBSTACLE_AIR_TEST_SPEED 300
#endif

#ifndef APP_OBSTACLE_GROUND_TEST_SPEED
#define APP_OBSTACLE_GROUND_TEST_SPEED 350
#endif

#if APP_OBSTACLE_MOTOR_ENABLE
#warning "APP_OBSTACLE_MOTOR_ENABLE is ON: lift wheels before testing"
#endif

#if APP_OBSTACLE_GROUND_TEST_ENABLE
#warning "APP_OBSTACLE_GROUND_TEST_ENABLE is ON: 3s start delay, 10s max run"
#endif

#endif /* APP_TEST_CONFIG_H */
