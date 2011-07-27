#include <vector>
#include "costmap_2d/costmap_2d.h"

#define WAYPOINTRAD 3
//#define WAYPOINTSPACE 40
#define WAYPOINTSPACE 20
//#define WAYPOINTSPACE 5

#define IMPASSABLE_THRESH 127
#define PASSABLE_THRESH 127

struct Waypoint {
  double x, y;
  int space;
  std::vector<Waypoint*> neighbors;

  Waypoint(double _x, double _y, int _space) : x(_x), y(_y), space(_space) {};
};

struct MapWaypoint {
  int x, y;
  int space;
  std::vector<MapWaypoint*> neighbors;

  MapWaypoint(unsigned _x, unsigned _y, int _space) : x(_x), y(_y), space(_space) {};
};

struct Line {
  double m, b;
  Line(double m_, double b_) : m(m_), b(b_) {};
};

struct Point {
  int x, y;
  Point(int x_, int y_) : x(x_), y(y_) {};
};

struct Topostore {
    MapWaypoint * home;
    std::vector<MapWaypoint*> *rtn;
    std::vector<Waypoint*> *worldRtn;
    std::vector<bool> * ignore;
    int ** memo;
    
    Topostore(MapWaypoint * home_, std::vector<MapWaypoint*>* rtn_, std::vector<Waypoint*>* worldRtn_, std::vector<bool> *ignore_, int **memo_) : home(home_), rtn(rtn_), worldRtn(worldRtn_), ignore(ignore_), memo(memo_) {};
};

inline bool inBounds(unsigned int x, unsigned int y, const costmap_2d::Costmap2D &costmap) {
  return x < costmap.getSizeInCellsX() && y < costmap.getSizeInCellsY();
}

class Topomap {
protected:
  // Topomap in map coordinates
  std::vector<MapWaypoint> map_topomap;

  MapWayPoint home;

  // Originating point for topomap
  double start_world_x, start_world_y;
  unsigned int start_map_x, start_map_y;

public:
  // Topomap in world coordinates
  std::vector<Waypoint> topomap;

  Topomap(const costmap_2d::Costmap2D& costmap, double world_x, double world_y);

  void update(bool showDebug=false);
};
