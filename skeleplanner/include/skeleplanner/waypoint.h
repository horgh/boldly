#include <vector>
#include "costmap_2d/costmap_2d.h"

struct Waypoint {
  double x, y;
  int space;
  std::vector<Waypoint*> neighbors;

Waypoint(double _x, double _y, int _space) : x(_x), y(_y), space(_space) {};
};

std::vector<Waypoint*> *topoFromPoint(double x, double y, const costmap_2d::Costmap2D &costmap, bool showDebug=false);
