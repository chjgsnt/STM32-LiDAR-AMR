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
 * AIR_TEST_SPEED is kept for legacy wheels-off-ground references.
 * GROUND_TEST_SPEED is used for obstacle motor dry-run previews and real
 * ground-test output, so the logged command matches the tested command.
 *
 * Default ground speed is 500 because this robot moved more reliably at that
 * duty during short ground tests. If it is too aggressive, reduce in small
 * steps. Keep the short-test timeout active and be ready to cut power.
 */
#ifndef APP_OBSTACLE_AIR_TEST_SPEED
#define APP_OBSTACLE_AIR_TEST_SPEED 300
#endif

#ifndef APP_OBSTACLE_GROUND_TEST_SPEED
#define APP_OBSTACLE_GROUND_TEST_SPEED 500
#endif

#ifndef APP_GROUND_CAUTION_SPEED
#define APP_GROUND_CAUTION_SPEED 350
#endif

/*
 * Ground turn stability.
 *
 * Once the ground state machine starts turning left/right, keep that direction
 * for at least this window. After the hold, switch to the opposite direction
 * only if that side is clearly more open by the configured margin.
 */
#ifndef APP_GROUND_TURN_MIN_HOLD_MS
#define APP_GROUND_TURN_MIN_HOLD_MS 700
#endif

#ifndef APP_GROUND_TURN_SWITCH_MARGIN_MM
#define APP_GROUND_TURN_SWITCH_MARGIN_MM 120
#endif

/*
 * Ground straight-line trim.
 *
 * Positive trim increases that side's forward duty. Negative trim reduces it.
 * The current robot drifts right in CLEAR_FORWARD, so start by reducing the
 * left side and increasing the right side. If it still drifts right, increase
 * the pair in small steps. If it drifts left, move the trims back toward 0.
 * These trims are applied only to forward/caution driving, not in-place turns.
 */
#ifndef APP_GROUND_LEFT_TRIM
#define APP_GROUND_LEFT_TRIM (-20)
#endif

#ifndef APP_GROUND_RIGHT_TRIM
#define APP_GROUND_RIGHT_TRIM 20
#endif

/*
 * Forward start boost.
 *
 * Applied only during the first short window after entering
 * FORWARD. Normal straight trim remains unchanged after this window.
 */
#ifndef APP_GROUND_FORWARD_START_BOOST_MS
#define APP_GROUND_FORWARD_START_BOOST_MS 600
#endif

#ifndef APP_GROUND_FORWARD_START_LEFT_EXTRA
#define APP_GROUND_FORWARD_START_LEFT_EXTRA 30
#endif

#ifndef APP_GROUND_FORWARD_START_RIGHT_EXTRA
#define APP_GROUND_FORWARD_START_RIGHT_EXTRA (-10)
#endif

/*
 * Contact recovery backup.
 *
 * Used when the front distance is already very close. The robot backs up
 * briefly, then turns toward the more open side.
 */
#ifndef APP_GROUND_BACKUP_MS
#define APP_GROUND_BACKUP_MS 400
#endif

#ifndef APP_GROUND_BACKUP_SPEED
#define APP_GROUND_BACKUP_SPEED 300
#endif

#ifndef APP_GROUND_BACKUP_COOLDOWN_MS
#define APP_GROUND_BACKUP_COOLDOWN_MS 2000
#endif

#ifndef APP_GROUND_ESCAPE_TURN_MS
#define APP_GROUND_ESCAPE_TURN_MS 1600
#endif

/*
 * Absolute duty clamp for ground obstacle motor output, after speed and trim
 * are combined. Applies to forward and turn actions.
 */
#ifndef APP_GROUND_MOTOR_MAX_ABS
#define APP_GROUND_MOTOR_MAX_ABS 650
#endif

#if APP_OBSTACLE_MOTOR_ENABLE
#warning "APP_OBSTACLE_MOTOR_ENABLE is ON: lift wheels before testing"
#endif

#if APP_OBSTACLE_GROUND_TEST_ENABLE
#warning "APP_OBSTACLE_GROUND_TEST_ENABLE is ON: 3s start delay, 10s max run"
#endif

#endif /* APP_TEST_CONFIG_H */
