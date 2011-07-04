#include <nav_core/base_global_planner.h>

#include "skeleplanner/waypoint.h"

class SkelePlanner : nav_core::BaseGlobalPlanner {
protected:
  costmap_2d::Costmap2D costmap;
  costmap_2d::Costmap2DROS *costmapros;
  std::vector<Waypoint*> *topomap;
  geometry_msgs::PoseStamped lastOrigin, safeOrigin;
  bool gotSafeOrigin;

  void wipeTopo();

public:
  SkelePlanner();
  ~SkelePlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap);
  void update();
  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector< geometry_msgs::PoseStamped > &plan);
};
