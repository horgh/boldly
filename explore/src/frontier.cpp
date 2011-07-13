#include "explore/frontier.h"

#include <vector>
#include <cmath>

double pathDistance(const geometry_msgs::PoseStamped &currentPose, const geometry_msgs::PoseStamped &frontier, nav_core::BaseGlobalPlanner &planner) {
  std::vector<geometry_msgs::PoseStamped> plan;
  if(!planner.makePlan(currentPose, goal, plan)) {
    // Frontier is unreachable
  }
  double pathDistance = 0;
  if(plan.size() > 1) {
    std::vector<geometry_msgs::PosteStamped>::const_iterator prev = plan.begin();
    for(std::vector<geometry_msgs::PosteStamped>::const_iterator curr = ++(plan.begin());
	curr != plan.end();
	++curr) {
      double x = curr->position.x - prev->position.x;
      double y = curr->position.y - prev->position.y;
      pathDistance += sqrt(x*x + y*y);
      prev = curr;
    }
  }
  
  return pathDistance;
}

PoseStamped chooseFrontier(const PoseStamped &currentPose, float currentBattery, nav_core::BaseGlobalPlanner &planner) {
  // Example path distance determination:
  geometry_msgs::PoseStamped frontier;
  frontier.position.z = 0.0;
  // Set these to the world-space position of a frontier
  frontier.position.x = 42.0;
  frontier.position.y = 42.0;
  double distance = pathDistance(currentPose, frontier, planner);
}
