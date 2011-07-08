#include <nav_core/base_global_planner.h>

#include "skeleplanner/waypoint.h"

class SkelePlanner : nav_core::BaseGlobalPlanner {
protected:
  costmap_2d::Costmap2D costmap;
  costmap_2d::Costmap2DROS *costmapros;
  geometry_msgs::PoseStamped lastOrigin, safeOrigin;
  bool gotSafeOrigin;

  void wipeTopo();

public:
  std::vector<Waypoint*> *topomap;

  SkelePlanner();
  ~SkelePlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap);
  void update();
  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector< geometry_msgs::PoseStamped > &plan);

  void waypoints_to_plan(std::vector<geometry_msgs::PoseStamped>* plan, std::vector<Waypoint*>* waypoint_plan);
  Waypoint* find_min_score_waypoint(std::vector<Waypoint*>* open_set, std::map<Waypoint*, int>* f_score);
  int heuristic_cost_estimate(Waypoint* x, Waypoint* goal);
  std::vector<Waypoint*> reconstruct_path(std::map<Waypoint*, Waypoint*>* came_from, Waypoint* point);

  template <class T, class C>
  void EraseAll(T thing, C& container);
  template <class T, class C>
  bool isInVector(T thing, C& container);

  int dist_between(Waypoint* x, Waypoint* y);
  std::vector<Waypoint*> aStar(Waypoint* start, Waypoint* goal);
};