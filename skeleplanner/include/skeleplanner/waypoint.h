#include <vector>
#include "costmap_2d/costmap_2d.h"

struct Waypoint {
  double x, y;
  int space;
  std::vector<Waypoint*> neighbors;

Waypoint(double _x, double _y, int _space) : x(_x), y(_y), space(_space) {};
};

struct MapWaypoint {
  unsigned x, y;
  int space;
  std::vector<MapWaypoint*> neighbors;

MapWaypoint(unsigned _x, unsigned _y, int _space) : x(_x), y(_y), space(_space) {};
};

struct FrontierStats{
    double frontierDelta;
    double lineDeltas;
    
    FrontierStats(double a, double b) : frontierDelta(a), lineDeltas(b) {};
};
    
struct Topostore{
    MapWaypoint * home;
    std::vector<MapWaypoint*> *rtn;
    std::vector<Waypoint*> *worldRtn;
    std::vector<bool> * ignore;
    int ** memo;
    
    Topostore(MapWaypoint * home_, std::vector<MapWaypoint*>* rtn_, std::vector<Waypoint*>* worldRtn_, std::vector<bool> *ignore_, int **memo_) : home(home_), rtn(rtn_), worldRtn(worldRtn_), ignore(ignore_), memo(memo_) {};
};

//std::vector<Waypoint*> *topoFromPoint(double x, double y, const costmap_2d::Costmap2D &costmap, bool showDebug=false);
std::vector<Waypoint*> * topoFromPoint(double worldx, double worldy, const costmap_2d::Costmap2D &costmap, bool showDebug=false, Topostore * memory=NULL);

std::vector<FrontierStats*> *frontierRatings(std::vector<WeightedFrontier> frontiers, const costmap_2d::Costmap2D &costmap, std::vector<Waypoint*> topo, int showDebug=0);
