#ifndef NAVIGATION_TASK_H
#define NAVIGATION_TASK_H

#include "cmsis_os.h"
#include "slam_types.h"
#include "astar.h"
#define MAX_PATH_LENGTH 128



void StartNavigationTask(void *argument);
void Navigation_UpdateControl(SlamPose2D_t* current_pose);

#endif /* NAVIGATION_TASK_H */
