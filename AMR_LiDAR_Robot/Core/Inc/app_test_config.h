#ifndef APP_TEST_CONFIG_H
#define APP_TEST_CONFIG_H

/*
 * CENTRAL TEST/RUN MODE SELECTOR.
 *
 * This demo build fixes APP_ACTIVE_MODE to LiDARObstacleAvoidance. The
 * lower-level enable flags are derived from this selector so test builds do
 * not require editing several independent macros.
 *
 * ServoScanObstacle is retained as an available mode, but it is not the active
 * mode because the current hardware has no SG90/MG90S servo installed.
 *
 * NoServoObstacleAvoidance is retained as an available mode, but it is not the
 * active mode because the current hardware has no ultrasonic module installed.
 *
 * MotorForcedSpinCheck is retained as an available mode, but it is not the
 * active mode for this demo.
 *
 * EncoderBringup is retained as an available mode for encoder count/delta
 * verification. It is not the active mode unless APP_ACTIVE_MODE is changed
 * explicitly for that test.
 *
 * Use APP_MODE_LIDAR_OBSTACLE_GROUND_TEST only for a supervised short ground
 * test after the lifted-wheel test has passed. Keep the 3 second start delay,
 * 10 second timeout, and emergency power access in place. Do not commit or
 * push a ground-test active mode to GitHub.
 */
#define APP_MODE_LIDAR_OBSTACLE_DRY_RUN     0
#define APP_MODE_LIDAR_OBSTACLE_GROUND_TEST 1
#define APP_MODE_MOTOR_TEST                 2
#define APP_MODE_IMU_TEST                   3
#define APP_MODE_SENSOR_BRINGUP             4
#define APP_MODE_IMU_HEADING_ASSIST_DRY_RUN 5
#define APP_MODE_LIDAR_OBSTACLE_STOP_CHECK  6
#define APP_MODE_MOTOR_FORCED_SPIN_CHECK    7
#define APP_MODE_SERVO_SCAN_OBSTACLE        8
#define APP_MODE_NO_SERVO_OBSTACLE          9
#define APP_MODE_LIDAR_OBSTACLE_AVOIDANCE   10
#define APP_MODE_ENCODER_BRINGUP            11

#ifdef APP_ACTIVE_MODE
#warning "APP_ACTIVE_MODE is fixed to APP_MODE_LIDAR_OBSTACLE_AVOIDANCE for this demo; direct override ignored"
#undef APP_ACTIVE_MODE
#endif
#define APP_ACTIVE_MODE APP_MODE_LIDAR_OBSTACLE_AVOIDANCE

#if (APP_ACTIVE_MODE < APP_MODE_LIDAR_OBSTACLE_DRY_RUN) || (APP_ACTIVE_MODE > APP_MODE_ENCODER_BRINGUP)
#error "APP_ACTIVE_MODE has an invalid value"
#endif

#define APP_MODE_IS_LIDAR_OBSTACLE_DRY_RUN     (APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_DRY_RUN)
#define APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST (APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_GROUND_TEST)
#define APP_MODE_IS_LIDAR_OBSTACLE_STOP_CHECK  (APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_STOP_CHECK)
#define APP_MODE_IS_LIDAR_OBSTACLE             (APP_MODE_IS_LIDAR_OBSTACLE_DRY_RUN || APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST || APP_MODE_IS_LIDAR_OBSTACLE_STOP_CHECK)
#define APP_MODE_IS_MOTOR_TEST                 (APP_ACTIVE_MODE == APP_MODE_MOTOR_TEST)
#define APP_MODE_IS_IMU_TEST                   (APP_ACTIVE_MODE == APP_MODE_IMU_TEST)
#define APP_MODE_IS_SENSOR_BRINGUP             (APP_ACTIVE_MODE == APP_MODE_SENSOR_BRINGUP)
#define APP_MODE_IS_IMU_HEADING_ASSIST_DRY_RUN (APP_ACTIVE_MODE == APP_MODE_IMU_HEADING_ASSIST_DRY_RUN)
#define APP_MODE_IS_MOTOR_FORCED_SPIN_CHECK    (APP_ACTIVE_MODE == APP_MODE_MOTOR_FORCED_SPIN_CHECK)
#define APP_MODE_IS_SERVO_SCAN_OBSTACLE        (APP_ACTIVE_MODE == APP_MODE_SERVO_SCAN_OBSTACLE)
#define APP_MODE_IS_NO_SERVO_OBSTACLE          (APP_ACTIVE_MODE == APP_MODE_NO_SERVO_OBSTACLE)
#define APP_MODE_IS_LIDAR_OBSTACLE_AVOIDANCE   (APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_AVOIDANCE)
#define APP_MODE_IS_ENCODER_BRINGUP            (APP_ACTIVE_MODE == APP_MODE_ENCODER_BRINGUP)
#define APP_MODE_USES_LIDAR_BRINGUP            (APP_MODE_IS_LIDAR_OBSTACLE || APP_MODE_IS_IMU_HEADING_ASSIST_DRY_RUN || APP_MODE_IS_LIDAR_OBSTACLE_AVOIDANCE)

/*
 * Derived mode flags. Do not edit these directly.
 */
#define APP_LIDAR_OBSTACLE_DRY_RUN_ENABLE     APP_MODE_IS_LIDAR_OBSTACLE_DRY_RUN
#define APP_LIDAR_OBSTACLE_GROUND_TEST_ENABLE APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST
#define APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE  APP_MODE_IS_LIDAR_OBSTACLE_STOP_CHECK
#define APP_TEST_MODE_ENABLE_MOTOR_TEST       APP_MODE_IS_MOTOR_TEST
#define APP_TEST_MODE_ENABLE_IMU_TEST         APP_MODE_IS_IMU_TEST
#define APP_TEST_MODE_ENABLE_SENSOR_BRINGUP   APP_MODE_IS_SENSOR_BRINGUP
#define APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE APP_MODE_IS_IMU_HEADING_ASSIST_DRY_RUN
#define APP_MOTOR_FORCED_SPIN_CHECK_ENABLE    APP_MODE_IS_MOTOR_FORCED_SPIN_CHECK
#define APP_SERVO_SCAN_OBSTACLE_ENABLE        APP_MODE_IS_SERVO_SCAN_OBSTACLE
#define APP_NO_SERVO_OBSTACLE_ENABLE          APP_MODE_IS_NO_SERVO_OBSTACLE
#define APP_LIDAR_OBSTACLE_AVOIDANCE_ENABLE   APP_MODE_IS_LIDAR_OBSTACLE_AVOIDANCE
#define APP_ENCODER_BRINGUP_ENABLE            APP_MODE_IS_ENCODER_BRINGUP

/*
 * LiDAR stop-check motor output arm.
 *
 * Fixed to 1 for this demo: clear-forward drives the wheels with the
 * LidarObstacleStopCheck forward command, while fail-stop states still command
 * a stop.
 */
#ifdef APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE
#warning "APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE is fixed to 1 for this demo; direct override ignored"
#undef APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE
#endif
#define APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE 1

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
 * Near-wall escape direction lock.
 *
 * When the front obstacle distance is very close, choose one escape turn
 * direction and keep it long enough to avoid left/right indecision near walls.
 */
#ifndef APP_GROUND_ESCAPE_LOCK_MS
#define APP_GROUND_ESCAPE_LOCK_MS 1200
#endif

#ifndef APP_GROUND_ESCAPE_ENTER_MM
#define APP_GROUND_ESCAPE_ENTER_MM 450
#endif

#ifndef APP_GROUND_ESCAPE_EXIT_MM
#define APP_GROUND_ESCAPE_EXIT_MM 520
#endif

#ifndef APP_GROUND_ESCAPE_TIMEOUT_SWITCH_MARGIN_MM
#define APP_GROUND_ESCAPE_TIMEOUT_SWITCH_MARGIN_MM 150
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

#ifndef APP_GROUND_CORNER_DETECT_MS
#define APP_GROUND_CORNER_DETECT_MS 3500
#endif

#ifndef APP_GROUND_CORNER_BACKUP_MS
#define APP_GROUND_CORNER_BACKUP_MS 1000
#endif

#ifndef APP_GROUND_CORNER_TURN_MS
#define APP_GROUND_CORNER_TURN_MS 2200
#endif

#ifndef APP_GROUND_CORNER_BACKUP_SPEED
#define APP_GROUND_CORNER_BACKUP_SPEED 350
#endif

#ifndef APP_GROUND_CORNER_COOLDOWN_MS
#define APP_GROUND_CORNER_COOLDOWN_MS 3000
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

/*
 * IMU heading assist for obstacle forward driving.
 *
 * ENABLE keeps the estimator and logs active. APPLY_TO_MOTOR must stay 0 until
 * the correction sign is verified from logs; with 0, left/right commands are
 * not changed.
 */
#ifndef APP_IMU_HEADING_ASSIST_ENABLE
#define APP_IMU_HEADING_ASSIST_ENABLE 1
#endif

#ifndef APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR
#define APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR 0
#endif

#ifndef APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE
#define APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE 0
#endif

#ifndef APP_IMU_HEADING_ASSIST_GROUND_TEST_ENABLE
#define APP_IMU_HEADING_ASSIST_GROUND_TEST_ENABLE 0
#endif

#define APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ACTIVE \
    (APP_MODE_IS_IMU_HEADING_ASSIST_DRY_RUN && (APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE != 0))

#define APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE \
    (APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ACTIVE && (APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR != 0))

#define APP_IMU_HEADING_ASSIST_GROUND_TEST_ACTIVE \
    (APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST && (APP_IMU_HEADING_ASSIST_GROUND_TEST_ENABLE != 0))

#define APP_IMU_HEADING_ASSIST_GROUND_OUTPUT_ENABLE \
    (APP_IMU_HEADING_ASSIST_GROUND_TEST_ACTIVE && (APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR != 0))

#if (APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR == 0) && \
    (APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE || APP_IMU_HEADING_ASSIST_GROUND_OUTPUT_ENABLE)
#error "IMU heading assist output must stay disabled when APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR is 0"
#endif

#ifndef APP_IMU_HEADING_KP
#define APP_IMU_HEADING_KP 8
#endif

#ifndef APP_IMU_HEADING_CORRECTION_MAX
#define APP_IMU_HEADING_CORRECTION_MAX 80
#endif

#ifndef APP_IMU_HEADING_DEADBAND_DEG
#define APP_IMU_HEADING_DEADBAND_DEG 2
#endif

#ifndef APP_IMU_HEADING_GROUND_KP
#define APP_IMU_HEADING_GROUND_KP 3
#endif

#ifndef APP_IMU_HEADING_GROUND_CORRECTION_MAX
#define APP_IMU_HEADING_GROUND_CORRECTION_MAX 30
#endif

#ifndef APP_IMU_HEADING_GROUND_DEADBAND_DEG
#define APP_IMU_HEADING_GROUND_DEADBAND_DEG 3
#endif

/*
 * Legacy obstacle motor switches are intentionally derived from APP_ACTIVE_MODE.
 * CLEAR SAFETY RULE:
 *   dry-run mode              -> enabled=0, ground=0, no wheel output
 *   ground-test mode          -> enabled=1, ground=1, guarded low-speed output allowed
 *   stop-check mode           -> enabled=0, ground=0, handled by app_lidar_stop_check only
 *   IMU lifted-wheel test     -> enabled=1, ground=0, only when explicitly armed below
 *
 * The lifted-wheel output path requires all of:
 *   APP_ACTIVE_MODE == APP_MODE_IMU_HEADING_ASSIST_DRY_RUN
 *   APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE == 1
 *   APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR == 1
 *
 * The ground heading assist output path requires all of:
 *   APP_ACTIVE_MODE == APP_MODE_LIDAR_OBSTACLE_GROUND_TEST
 *   APP_IMU_HEADING_ASSIST_GROUND_TEST_ENABLE == 1
 *   APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR == 1
 */
#ifdef APP_OBSTACLE_MOTOR_ENABLE
#warning "APP_OBSTACLE_MOTOR_ENABLE is derived from APP_ACTIVE_MODE; direct override ignored"
#undef APP_OBSTACLE_MOTOR_ENABLE
#endif
#define APP_OBSTACLE_MOTOR_ENABLE \
    (APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST || APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE)

#ifdef APP_OBSTACLE_GROUND_TEST_ENABLE
#warning "APP_OBSTACLE_GROUND_TEST_ENABLE is derived from APP_ACTIVE_MODE; direct override ignored"
#undef APP_OBSTACLE_GROUND_TEST_ENABLE
#endif
#define APP_OBSTACLE_GROUND_TEST_ENABLE APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST

#if APP_MODE_IS_LIDAR_OBSTACLE_STOP_CHECK && (APP_OBSTACLE_MOTOR_ENABLE || APP_OBSTACLE_GROUND_TEST_ENABLE)
#error "LidarObstacleStopCheck must not enable the obstacle motor state machine"
#endif

#if APP_MODE_IS_LIDAR_OBSTACLE_GROUND_TEST
#warning "APP_ACTIVE_MODE is LiDAR obstacle GROUND TEST: lift wheels first, 3s start delay, 10s max run"
#endif

#if APP_MODE_IS_LIDAR_OBSTACLE_STOP_CHECK
#warning "APP_ACTIVE_MODE is LiDAR obstacle STOP CHECK: lift wheels first, front-only motor output"
#endif

#if APP_MODE_IS_MOTOR_FORCED_SPIN_CHECK
#warning "APP_ACTIVE_MODE is MOTOR FORCED SPIN CHECK: lift wheels first, direct motor driver output"
#endif

#if APP_MODE_IS_SENSOR_BRINGUP
#warning "APP_ACTIVE_MODE is SENSOR BRINGUP: no motor, no servo, no ultrasonic obstacle output"
#endif

#if APP_MODE_IS_LIDAR_OBSTACLE_AVOIDANCE
#warning "APP_ACTIVE_MODE is LIDAR OBSTACLE AVOIDANCE: verify LiDAR front sector and lift wheels first"
#endif

#if APP_MODE_IS_SERVO_SCAN_OBSTACLE
#warning "APP_ACTIVE_MODE is SERVO SCAN OBSTACLE: verify servo/ultrasonic pins, lift wheels first"
#endif

#if APP_MODE_IS_NO_SERVO_OBSTACLE
#warning "APP_ACTIVE_MODE is NO SERVO OBSTACLE: ultrasonic-only backup/turn demo, lift wheels first"
#endif

#if APP_MODE_IS_ENCODER_BRINGUP
#warning "APP_ACTIVE_MODE is ENCODER BRINGUP: spin wheels by hand or run a supervised motor test"
#endif

#if APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE
#warning "IMU heading assist lifted-wheel test is ON: lift wheels before testing"
#endif

#if APP_IMU_HEADING_ASSIST_GROUND_TEST_ENABLE
#warning "IMU heading assist ground test is ON: verify open area and emergency stop"
#endif

#if APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR
#warning "APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR is ON: verify heading correction sign before any motor output"
#endif

#endif /* APP_TEST_CONFIG_H */
