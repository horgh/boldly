#ifndef _FRONTIER_H_
#define _FRONTIER_H_

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/Costmap2D.h>
#include <nav_core/base_global_planner.h>

PoseStamped chooseFrontier(const PoseStamped &currentPose, float currentBattery, const costmap_2d::Costmap2D &costmap, nav_core::BaseGlobalPlanner &planner);

#endif
