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
  update();
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
  
  return true;
}
