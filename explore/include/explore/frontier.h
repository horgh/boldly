#ifndef _FRONTIER_H_
#define _FRONTIER_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>

PoseStamped chooseFrontier(const PoseStamped &currentPose, float currentBattery, nav_core::BaseGlobalPlanner &planner);

#endif
