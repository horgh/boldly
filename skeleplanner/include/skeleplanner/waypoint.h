#include <vector>

#include "CImg.h"

struct Waypoint {
  int x;
  int y;
  int space;
  std::vector<Waypoint*> neighbors;

Waypoint(int _x, int _y, int _space) : x(_x), y(_y), space(_space) {};
};

std::vector<Waypoint*> *topoFromPoint(int x, int y, int maxPoints, cimg_library::CImg<unsigned char> image, bool showDebug=false);
