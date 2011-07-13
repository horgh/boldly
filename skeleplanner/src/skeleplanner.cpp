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
  topomap(NULL), gotSafeOrigin(false), marker_id(0),
  // red like loop closure
  //r_(1.0), g_(0.0), b_(0.0), a_(1.0)
  // yellow
  r_(255.0), g_(255.0), b_(0.0), a_(1.0)
  { }

SkelePlanner::~SkelePlanner() {
  if(topomap) {
    wipeTopo();
  }
}

void SkelePlanner::wipeTopo() {
  for(std::vector<Waypoint*>::iterator i = topomap->begin(); i != topomap->end(); ++i) {
    delete *i;
  }
  delete topomap;
  topomap = NULL;
}

void SkelePlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmapros) {
  this->costmapros = costmapros;
}

void SkelePlanner::update() {
  costmapros->getCostmapCopy(costmap);
  if(topomap) {
    wipeTopo();
  }
  std::vector<Waypoint*> *result = topoFromPoint(lastOrigin.pose.position.x, lastOrigin.pose.position.y, costmap);
  gotSafeOrigin = result->size() > 1;

  if(gotSafeOrigin) {
    // Not trapped!
    safeOrigin = lastOrigin;
  } else {
    for(std::vector<Waypoint*>::iterator i = result->begin(); i != result->end(); ++i) {
      delete *i;
    }
    delete result;
    
    result = topoFromPoint(safeOrigin.pose.position.x, safeOrigin.pose.position.y, costmap);
    gotSafeOrigin = result->size() > 1;
  }
  topomap = result;
}

bool SkelePlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector< geometry_msgs::PoseStamped > &plan) {
  lastOrigin = start;
  if(!gotSafeOrigin) {
    update();
    if(!gotSafeOrigin) {
      // No way out from start exists
      return false;
    }
  }
  
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
  // TODO: Verify that goal is reachable from end

  // A*
  std::vector<Waypoint*> waypoint_plan = aStar(begin, end);
  waypoints_to_plan(plan, waypoint_plan);

  // XXX Hack? Always add goal to plan too.
  // Since we will usually not have a topo node at specific goal point
  // (when using the current frontier selector, anyway)
  plan.push_back( goal );

  expand_plan(&plan);
  
  return true;
}

/*
  The result from A* is a plan with poses too far apart (waypoints) which
  fails to work with the local planner (?).

  Fill the gaps between these poses with new poses.
*/
void SkelePlanner::expand_plan(std::vector<geometry_msgs::PoseStamped>* plan)
{
  std::vector<geometry_msgs::PoseStamped> old_plan;
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan->begin();
    it != plan->end();
    ++it)
  { 
    old_plan.push_back( *it );
    ROS_WARN("old plan %f, %f", it->pose.position.x, it->pose.position.y);
  }
  plan->clear();

  double x_current, y_current,
    x_next, y_next, 
    delta = 0.01;
    //delta = 0.10;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = costmapros->getGlobalFrameID();
  pose_stamped.header.stamp = ros::Time::now();
  
  bool done, x_done, y_done;
  std::vector<geometry_msgs::PoseStamped>::const_iterator next_it;

  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = old_plan.begin();
    it != old_plan.end();
    ++it)
  {
    // Add the current pose to the new plan
    plan->push_back( *it );
    ROS_WARN("adding main pt %f, %f", it->pose.position.x, it->pose.position.y);

    next_it = it+1;
    if (next_it == old_plan.end()) {
      //ROS_WARN("Ended correctly.");
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

    //double sign_x = x_current - x_next > 0.0 ? 1.0 : -1.0;
    //double sign_y = y_current - y_next > 0.0 ? 1.0 : -1.0;
    double sign_x = x_current - x_next > 0.0 ? -1.0 : 1.0;
    double sign_y = y_current - y_next > 0.0 ? -1.0 : 1.0;

    ROS_WARN("sign x %f, sign y %f, diff x %f diff y %f", sign_x, sign_y,
      x_current - x_next, y_current - y_next);
    //return;

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
        ROS_WARN("adding intermediate pt %f, %f", pose_stamped.pose.position.x, pose_stamped.pose.position.y);
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
    std::cerr << "** SkelePlanner had a NULL in path." << std::endl;
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
      std::cerr << "** SkelePlanner had a NULL in path." << std::endl;
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

/*
  Drawing functions
*/

/*
  Add a sphere marker at x, y to markers vector
  (Based on visualizeNode() in explore/loop_closure)
*/
void SkelePlanner::visualize_node(double x, double y, double scale, double r, double g,
  double b, double a, std::vector<visualization_msgs::Marker>* markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "skeletester";
  marker.id = marker_id++;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = x;
  marker.pose.position.y = y;

  // thick point like in loop_closure is 0.5
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  markers->push_back(marker);
}

/*
  Add a line marker between the two sets of points to markers vector
  (Based on visualizeEdge() in explore/loop_closure)
*/
void SkelePlanner::visualize_edge(double x1, double y1, double x2, double y2,
  double scale, double r, double g, double b, double a,
  std::vector<visualization_msgs::Marker>* markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "skeletester";
  marker.id = marker_id++;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p;
  p.x = x1;
  p.y = y1;
  marker.points.push_back(p);

  p.x = x2;
  p.y = y2;
  marker.points.push_back(p);

  // thick line like in loop closure is 0.25
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  // red is 1, 0, 0, 1
  // blue 0, 1, 0, 1?
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  markers->push_back(marker);
}

/*
  Uses the above 2 functions to publish the current topomap
*/
void SkelePlanner::publish_topomap(ros::Publisher* marker_pub) {
  marker_id = 0;
  std::vector<visualization_msgs::Marker> markers;

#ifdef DEBUG
  ROS_WARN("SkelePlanner publishing %d nodes in topomap", topomap->size());
#endif
  // Markers for each node in the topological map
  for (std::vector<Waypoint*>::const_iterator it = topomap->begin();
    it != topomap->end();
    ++it)
  {
    // 0.5 scale, red colour: 1.0, 0.0, 0.0, 1.0
    visualize_node( (*it)->x, (*it)->y, 0.5, r_, g_, b_, a_, &markers);
  }

  // Markers for the links between each node
  for (std::vector<Waypoint*>::const_iterator it = topomap->begin();
    it != topomap->end();
    ++it)
  {
    // For each neighbour of this node, add an edge to it
    for (std::vector<Waypoint*>::const_iterator it2 = (*it)->neighbors.begin();
      it2 != (*it)->neighbors.end();
      ++it2)
    {
      // 0.25 scale
      visualize_edge((*it)->x, (*it)->y, (*it2)->x, (*it2)->y, 0.25, r_, g_, b_, a_, &markers);
    }
  }

  // Now publish all of these markers
  for (std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin();
    it != markers.end();
    ++it)
  {
    marker_pub->publish( *it );
  }
}
