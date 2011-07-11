#include "skeleplanner/skeleplanner.h"

#include <cmath>
#include <algorithm>

SkelePlanner::SkelePlanner() :
  topomap(NULL), gotSafeOrigin(false) {
}

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

  // TODO: A* from begin to end.
  std::vector<Waypoint*> waypoint_plan = aStar(begin, end);
  // XXX uncommented since segfault, but will be needed
  //waypoints_to_plan(&plan, &waypoint_plan);
  
  return true;
}

/*
  Take Waypoint plan and make PoseStamped plan
*/
void SkelePlanner::waypoints_to_plan(std::vector<geometry_msgs::PoseStamped>* plan, std::vector<Waypoint*>* waypoint_plan) {
  // waypoints are in reverse order
  for (std::vector<Waypoint*>::reverse_iterator it = waypoint_plan->rbegin(); it != waypoint_plan->rend(); it++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = costmapros->getGlobalFrameID();
    pose.header.stamp = ros::Time::now();

    // XXX Need to set quaternion too?
    pose.pose.position.x = (*it)->x;
    pose.pose.position.y = (*it)->y;
    pose.pose.position.z = 0.0;
    plan->push_back( pose );
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
  return min_waypoint;
}

/*
  XXX Wrong
*/
int SkelePlanner::heuristic_cost_estimate(Waypoint* x, Waypoint* goal) {
  return dist_between(x, goal);
}

/*
  Vector of waypoints to goal
*/
std::vector<Waypoint*> SkelePlanner::reconstruct_path(std::map<Waypoint*, Waypoint*>* came_from, Waypoint* point) {
  std::vector<Waypoint*> path;

  path.push_back(point);

  while (came_from->count( point ) > 0) {
    point = (*came_from)[point];
    path.push_back(point);
  }

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
  // set of nodes already evaluated
  std::vector<Waypoint*> closed_set;
  // set of tentative nodes to be evaluated
  std::vector<Waypoint*> open_set;
  open_set.push_back( start );

  // map of navigated nodes
  std::map<Waypoint*, Waypoint*> came_from;

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

    if (x == goal)
      return reconstruct_path(&came_from, came_from[goal]);

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
