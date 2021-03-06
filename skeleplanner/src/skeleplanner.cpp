#include "skeleplanner/skeleplanner.h"
#include <pluginlib/class_list_macros.h>

#include <cmath>
#include <algorithm>

PLUGINLIB_DECLARE_CLASS(skeleplanner, SkelePlanner, SkelePlanner, nav_core::BaseGlobalPlanner)

#define DEBUG

#ifdef DEBUG
#include <iostream>
#endif

SkelePlanner::SkelePlanner() :
  topomap(NULL), gotSafeOrigin(false), marker_id_last(0),
  // red like loop closure
  //r_(1.0), g_(0.0), b_(0.0), a_(1.0)
  // yellow
  r_(255.0), g_(255.0), b_(0.0), a_(1.0)
{
  topomap = Topomap(costmap)
Topomap::Topomap(const costmap_2d::Costmap2D& costmap,
  double start_world_x, double start_world_y)
}

SkelePlanner::~SkelePlanner() {
  if(topomap) {
    wipeTopo();
  }
}

void SkelePlanner::wipeTopo() {
  //dont delete
  /*
  for(std::vector<Waypoint*>::iterator i = topomap->begin(); i != topomap->end(); ++i) {
    delete *i;
  }
  delete topomap;
  topomap = NULL;
  */
}

void SkelePlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmapros) {
  this->costmapros = costmapros;
}

/*
  Need  to set this before running makePlan() or update()
  (This is here rather than in a constructor or initialize as pluginlib
  wants those functions to be of specific definition)
*/
void SkelePlanner::set_topomap_origin(double origin_x, double origin_y) {
  topomap_origin_x = origin_x;
  topomap_origin_y = origin_y;
}

std::vector<Waypoint*>* SkelePlanner::get_topomap() {
  return topomap;
}

void SkelePlanner::update() {
  costmapros->getCostmapCopy(costmap);
  if(topomap) {
    wipeTopo();
  }
  //std::vector<Waypoint*> *result = topoFromPoint(lastOrigin.pose.position.x, lastOrigin.pose.position.y, costmap);
  std::vector<Waypoint*> *result = topoFromPoint(topomap_origin_x, topomap_origin_y, costmap, true, topo_memory);
  //gotSafeOrigin = result->size() > 1;

  /*if(gotSafeOrigin) {
    // Not trapped!
    safeOrigin = lastOrigin;
  } else {
    //dont delete
    //for(std::vector<Waypoint*>::iterator i = result->begin(); i != result->end(); ++i) {
    //  delete *i;
    //}
    //delete result;
    
    //result = topoFromPoint(safeOrigin.pose.position.x, safeOrigin.pose.position.y, costmap);
    result = topoFromPoint(topomap_origin_x, topomap_origin_y, costmap, false, topo_memory);
    gotSafeOrigin = result->size() > 1;
  }*/
  topomap = result;
}

bool SkelePlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector< geometry_msgs::PoseStamped > &plan) {
  lastOrigin = start;
  /*
  if(!gotSafeOrigin) {
    update();
    if(!gotSafeOrigin) {
      // No way out from start exists
      return false;
    }
  }
  */
  update();
  
/*
 * This is needed if we want to use as path planner (ie, begin & end to aStar())
  std::vector<float> dists(topomap->size());
  for(size_t i = 0; i < topomap->size(); ++i) {
    dists[i] = sqrt(((*topomap)[i]->x - start.pose.position.x)*((*topomap)[i]->x - start.pose.position.x) +
		    ((*topomap)[i]->y - start.pose.position.y)*((*topomap)[i]->y - start.pose.position.y));
  }

  Waypoint *begin = topomap->at(std::min_element(dists.begin(), dists.end()) - dists.begin());

  for(size_t i = 0; i < topomap->size(); ++i) {
    dists[i] = sqrt(((*topomap)[i]->x - goal.pose.position.x)*((*topomap)[i]->x - goal.pose.position.x) +
		    ((*topomap)[i]->y - goal.pose.position.y)*((*topomap)[i]->y - goal.pose.position.y));
  }

  Waypoint *end = topomap->at(std::min_element(dists.begin(), dists.end()) - dists.begin());
*/
  // TODO: Verify that goal is reachable from end

  // A*
  // We aren't using this for planning right now. So don't find a plan.
/*
  std::vector<Waypoint*> waypoint_plan = aStar(begin, end);
  waypoints_to_plan(plan, waypoint_plan);

  // XXX Hack? Always add goal to plan too.
  // Since we will usually not have a topo node at specific goal point
  // (when using the current frontier selector, anyway)
  plan.push_back( goal );

  expand_plan(&plan);

  // remove goal, since other planners do not include goal as part of plan
  plan.pop_back();
*/
  
  return true;
}

/*
  The result from A* is a plan with poses too far apart (waypoints) which
  fails to work with the local planner (?).

  Fill the gaps between these poses with new poses.
*/
void SkelePlanner::expand_plan(std::vector<geometry_msgs::PoseStamped>* plan)
{
#ifdef DEBUG
  ROS_WARN("expand_plan(): Original plan has poses:");
#endif
  std::vector<geometry_msgs::PoseStamped> original_plan;
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan->begin();
    it != plan->end();
    ++it)
  { 
    original_plan.push_back( *it );
#ifdef DEBUG
    ROS_WARN("\tOriginal plan pose: %f, %f", it->pose.position.x, it->pose.position.y);
#endif
  }
  plan->clear();

  double x_current, y_current,
    x_next, y_next,
    sign_x, sign_y;

  // 0.20 seems sufficient. We could do this based on resolution...
  double delta = 0.20;
    //delta = 0.01;
    //delta = 0.10;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = costmapros->getGlobalFrameID();
  pose_stamped.header.stamp = ros::Time::now();

  pose_stamped.pose.position.z = 0.0;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;
  
  bool done, x_done, y_done;
  std::vector<geometry_msgs::PoseStamped>::const_iterator next_it;

  // For each pose in the original plan, add intermediary points
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = original_plan.begin();
    it != original_plan.end();
    ++it)
  {
    // Add the original plan pose to the new plan
    // (These are the points which are too far apart which we
    // are filling in)
    plan->push_back( *it );
#ifdef DEBUG
    ROS_WARN("expand_plan(): Adding primary planned point %f, %f",
      it->pose.position.x, it->pose.position.y);
#endif

    next_it = it+1;
    if (next_it == original_plan.end()) {
#ifdef DEBUG
      ROS_WARN("expand_plan() ended correctly.");
#endif
      continue;
    }

    // And add poses in delta steps up to the next pose
    x_current = it->pose.position.x;
    y_current = it->pose.position.y;

    x_next = next_it->pose.position.x;
    y_next = next_it->pose.position.y;

    done = false;
    x_done = false;
    y_done = false;

    // To know which way to move our points
    sign_x = x_current - x_next > 0.0 ? -1.0 : 1.0;
    sign_y = y_current - y_next > 0.0 ? -1.0 : 1.0;

#ifdef DEBUG
    ROS_WARN("expand_plan(): sign x %f, sign y %f, diff x %f diff y %f",
      sign_x, sign_y, x_current - x_next, y_current - y_next);
#endif

    while (!done) {
      x_done = fabs(x_current - x_next) < delta;
      if (!x_done)
        x_current += delta * sign_x;
      y_done = fabs(y_current - y_next) < delta;
      if (!y_done)
        y_current += delta * sign_y;

      done = x_done && y_done;
      if (!done) {
        pose_stamped.pose.position.x = x_current;
        pose_stamped.pose.position.y = y_current;

        plan->push_back( pose_stamped );
#ifdef DEBUG
        ROS_WARN("expand_plan(): Adding intermediate point %f, %f",
          pose_stamped.pose.position.x, pose_stamped.pose.position.y);
#endif
      }
    }
  }
}

/*
  Take Waypoint plan and make PoseStamped plan
*/
void SkelePlanner::waypoints_to_plan(std::vector<geometry_msgs::PoseStamped>& plan, const std::vector<Waypoint*> &waypoint_plan) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = costmapros->getGlobalFrameID();
  pose_stamped.header.stamp = ros::Time::now();
  // waypoints are in reverse order
  for (std::vector<Waypoint*>::const_reverse_iterator it = waypoint_plan.rbegin();
    it != waypoint_plan.rend(); it++)
  {
    pose_stamped.pose.position.x = (*it)->x;
    pose_stamped.pose.position.y = (*it)->y;
    pose_stamped.pose.position.z = 0.0;

    // XXX Don't know whether this needs to be set. NavfnROS seems to set all
    // to 0.0, 0.0, 0.0, 1.0
    // May be the default anyway so this is not needed?
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;

    plan.push_back( pose_stamped );
  }
}

/*
  Find node in open_set having the lowest f_score value
*/
Waypoint* SkelePlanner::find_min_score_waypoint(std::vector<Waypoint*>* open_set, std::map<Waypoint*, int>* f_score) {
  Waypoint *min_waypoint = NULL;
  int min_score = -1;
  for (std::vector<Waypoint*>::iterator it = open_set->begin(); it != open_set->end(); it++) {
    if (min_waypoint == NULL) {
      min_waypoint = *it;
      min_score = (*f_score)[min_waypoint];
    } else if ( (*f_score)[*it] < min_score ) {
      min_waypoint = *it;
      min_score = (*f_score)[min_waypoint];
    }
  }
  assert(min_waypoint != NULL);
  return min_waypoint;
}

/*
  XXX Wrong?
*/
int SkelePlanner::heuristic_cost_estimate(Waypoint* x, Waypoint* goal) {
  return dist_between(x, goal);
}

/*
  Vector of waypoints to goal
*/
std::vector<Waypoint*> SkelePlanner::reconstruct_path(std::map<Waypoint*, Waypoint*>* came_from, Waypoint* point) {
  std::vector<Waypoint*> path;

  // Can be NULL here in the case that we planned a path to&from same location
  if (point != NULL) {
    path.push_back(point);
  }
#ifdef DEBUG
  else {
    ROS_WARN("** SkelePlanner had a NULL in path.");
  }
#endif

  while (came_from->count( point ) > 0) {
    point = (*came_from)[point];
    // Same as above check
    if (point != NULL) {
      path.push_back(point);
    }
#ifdef DEBUG
    else {
      ROS_WARN("** SkelePlanner had a NULL in path."); 
    }
#endif
  }

#ifdef DEBUG
  std::cout << "Returning path with size " << path.size() << " to point at address " << point << std::endl;
  for (std::vector<Waypoint*>::iterator it = path.begin(); it != path.end(); it++) {
    std::cout << "Memory address of waypoint in path: " << *it << std::endl;
  }
  for (std::map<Waypoint*, Waypoint*>::iterator it = came_from->begin(); it != came_from->end(); it++) {
    std::cout << "came_from: Memory address 1: " << it->first << " 2: " << it->second << std::endl;
  }
#endif

  return path;
}

/*
  Remove thing from container
*/
template <class T, class C>
void SkelePlanner::EraseAll(T thing, C& container) {
  container.erase(
    std::remove( container.begin(), container.end(), thing), container.end()
  );
}

/*
  True if thing is in container
*/
template <class T, class C>
bool SkelePlanner::isInVector(T thing, C& container) {
  return std::find(container.begin(), container.end(), thing) != container.end();
}

/*

*/
int SkelePlanner::dist_between(Waypoint* x, Waypoint* y) {
  double dx = x->x - y->x;
  double dy = x->y - y->y;
  return (int) sqrt(dx*dx + dy*dy);
}

/*
  Translated from wikipedia pseudocode!
*/
std::vector<Waypoint*> SkelePlanner::aStar(Waypoint* start, Waypoint* goal) {
#ifdef DEBUG
  std::cout << "A*: trying to find path from waypoint at (" << start << ") (" << start->x
    << ", " << start->y << ") to waypoint goal at (" << goal << ") (" << goal->x
    << ", " << goal->y << ")" << std::endl;
#endif

  // set of nodes already evaluated
  std::vector<Waypoint*> closed_set;
  // set of tentative nodes to be evaluated
  std::vector<Waypoint*> open_set;
  open_set.push_back( start );

  // map of navigated nodes
  std::map<Waypoint*, Waypoint*> came_from;

  // XXX Hack. We came from our current location to get to our current location.
  // Without this we are segfaulting if we try to go from start to goal where start == goal
  // as immediately return reconstruct_path() since x == goal, but we haven't yet
  // filled that.
  // Seems unneeded? Or not solves the problem. I forget!
  //came_from[start] = start;

  // cost from start along best known path
  std::map<Waypoint*, int> g_score;
  g_score.insert( std::pair<Waypoint*, int>(start, 0) );

  std::map<Waypoint*, int> h_score;
  h_score.insert( std::pair<Waypoint*, int>(start, heuristic_cost_estimate(start, goal)) );

  // Estimated total cost from start to goal through y
  std::map<Waypoint*, int> f_score;
  f_score.insert( std::pair<Waypoint*, int>(start, h_score[start]) );

  while ( !open_set.empty() ) {
    Waypoint *x = find_min_score_waypoint(&open_set, &f_score);

    if (x == goal) {
      return reconstruct_path(&came_from, came_from[goal]);
    }

    // Remove from open set, add to closed set
    EraseAll(x, open_set);
    closed_set.push_back(x);

    // for each y in neighbour nodes of x
    for (std::vector<Waypoint*>::iterator y = x->neighbors.begin(); y != x->neighbors.end(); y++) {
      // if y is in closed set
      if ( isInVector(*y, closed_set) )
        continue;

      int tentative_g_score = g_score[x] + dist_between(x, *y);

      bool tentative_is_better = false;

      if ( !isInVector(*y, open_set) ) {
        open_set.push_back(*y);
        tentative_is_better = true;

      } else if (tentative_g_score < g_score[*y]) {
        tentative_is_better = true;

      } else {
        tentative_is_better = false;
      }

      if (tentative_is_better) {
        if ( came_from.count(*y) == 0 ) {
          came_from.insert( std::pair<Waypoint*, Waypoint*>(*y, NULL) );
        }
        came_from[*y] = x;
        g_score[*y] = tentative_g_score;
        h_score[*y] = heuristic_cost_estimate(*y, goal);
        f_score[*y] = g_score[*y] + h_score[*y];
      }
    }
  }

  std::vector<Waypoint*> empty;
  return empty;
}
