#include <vector>
#include "costmap_2d/costmap_2d.h"

struct Waypoint {
  double x, y;
  int space;
  std::vector<Waypoint*> neighbors;

Waypoint(double _x, double _y, int _space) : x(_x), y(_y), space(_space) {};
};

struct FrontierStats{
    double frontierDelta;
    double lineDeltas;
    
    FrontierStats(double a, double b) : frontierDelta(a), lineDeltas(b) {};
};
    
struct Topostore{
    Waypoint * home;
    vector<MapWaypoint*> *rtn;
    vector<Waypoint*> *worldRtn;
    vector<bool> * ignore;
    int ** memo;
    
    Topostore(Waypoint * home_, Topomap * rtn_, vector<bool> *ignore_, int **memo_) : home(home_), rtn(rtn_), ignore(ignore_), memo(memo_) {};
};

std::vector<Waypoint*> *topoFromPoint(double x, double y, const costmap_2d::Costmap2D &costmap, bool showDebug=false);
