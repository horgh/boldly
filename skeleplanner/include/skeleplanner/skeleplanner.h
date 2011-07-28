#ifndef SKELEPLANNER_H_
#define SKELEPLANNER_H_

#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "skeleplanner/waypoint.h"

class SkelePlanner : public nav_core::BaseGlobalPlanner {
protected:
  costmap_2d::Costmap2D costmap;
  costmap_2d::Costmap2DROS *costmapros;
  geometry_msgs::PoseStamped lastOrigin, safeOrigin;

  bool gotSafeOrigin;
  double topomap_origin_x,
    topomap_origin_y;
  
  // colours of markers
  double r_, g_, b_, a_;

  Topostore* topo_memory;

  void wipeTopo();

public:
  SkelePlanner();
  ~SkelePlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap);
  void set_topomap_origin(double origin_x, double origin_y);
  std::vector<Waypoint*>* get_topomap();
  void update();
  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector< geometry_msgs::PoseStamped > &plan);

  void expand_plan(std::vector<geometry_msgs::PoseStamped>* plan);

  // A* functions
  void waypoints_to_plan(std::vector<geometry_msgs::PoseStamped> &plan, const std::vector<Waypoint*> &waypoint_plan);
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

#endif
