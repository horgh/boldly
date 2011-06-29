#include "skeleplanner/skeleplanner.h"

#include <cmath>
#include <algorithm>

SkelePlanner::SkelePlanner() :
  topomap(NULL), costmap(NULL), lastStart(/*TODO*/) {
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

void SkelePlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap) {
  this.costmap = costmap;
  update();
}

void SkelePlanner::update() {
  if(topomap) {
    wipeTopo();
  }
  // TODO: Coordinate conversion?
  topomap = topoFromPoint(lastStart.pose.positoin.x, lastStart.pose.position.y, INF, costmap);
}

bool SkelePlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector< geometry_msgs::PoseStamped > &plan) {
  lastStart = start;
  
  std::vector<float> dists(topomap->size());
  for(size_t i = 0; i < topomap->size(); ++i) {
    dists[i] = sqrt(((*topomap)[i].x - start.pose.position.x)*((*topomap)[i].x - start.pose.position.x) +
		    ((*topomap)[i].y - start.pose.position.y)*((*topomap)[i].y - start.pose.position.y));
  }

  Waypoint *begin = topomap->at(std::min_element(dists.begin(), dists.end()) - dists.begin());

  for(size_t i = 0; i < topomap->size(); ++i) {
    dists[i] = sqrt(((*topomap)[i].x - start.pose.position.x)*((*topomap)[i].x - goal.pose.position.x) +
		    ((*topomap)[i].y - start.pose.position.y)*((*topomap)[i].y - goal.pose.position.y));
  }

  Waypoint *end = topomap->at(std::min_element(dists.begin(), dists.end()) - dists.begin());

  // TODO: A* from begin to end.
  
  return true;
}
