#ifndef NAVIGATION_H
#define NAVIGATION_H

/*
 * Navigation and path planning module.
 *
 * This module will plan robot motion from map data and produce high-level
 * velocity or waypoint commands.
 */

void Navigation_Init(void);
void Navigation_Task(void);
void Navigation_Stop(void);

#endif /* NAVIGATION_H */
