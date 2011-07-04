#include <vector>
#include "costmap_2d/costmap_2d.h"

struct Waypoint {
  int x;
  int y;
  int space;
  std::vector<Waypoint*> neighbors;

Waypoint(int _x, int _y, int _space) : x(_x), y(_y), space(_space) {};
};

std::vector<Waypoint*> *topoFromPoint(int x, int y, const costmap_2d::Costmap2D &costmap, bool showDebug=false);
