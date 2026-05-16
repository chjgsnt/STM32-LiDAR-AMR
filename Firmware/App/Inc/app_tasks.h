#ifndef APP_TASKS_H
#define APP_TASKS_H

/*
 * Application task module.
 *
 * This module will own FreeRTOS task creation and high-level scheduling for
 * sensing, control, mapping, navigation, display, and debug logging.
 */

void AppTasks_Init(void);
void AppTasks_Start(void);

#endif /* APP_TASKS_H */
