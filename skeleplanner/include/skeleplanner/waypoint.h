#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include <vector>
#include <cstring>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#define WAYPOINTRAD 3
//#define WAYPOINTSPACE 40
#define WAYPOINTSPACE 15
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

  MapWaypoint() {};
  MapWaypoint(unsigned _x, unsigned _y, int _space) : x(_x), y(_y), space(_space) {};
};

struct Line {
  double m, b;
  Line(double m_, double b_) : m(m_), b(b_) {};
};

struct Point {
  unsigned int x, y;
  Point() {};
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
  std::vector<MapWaypoint*> map_topomap;

  MapWaypoint* home;

  // Originating point for topomap
  double start_world_x, start_world_y;
  unsigned int start_map_x, start_map_y;

  std::vector<std::vector<int> > memo;

  std::vector<bool> ignore;

  // Topomap in world coordinates
  std::vector<Waypoint*> topomap;

  // colours of markers
  double r_, g_, b_, a_;

  // Need to remember these & check in case map size changes
  unsigned int costmap_size_x, costmap_size_y;

  int calcSpace(int x, int y, const costmap_2d::Costmap2D& costmap);
// bool straightClear(int x1, int y1, int x2, int y2,
//  const costmap_2d::Costmap2D& costmap);
  MapWaypoint waypointBest(int x, int y, const costmap_2d::Costmap2D& costmap);

  void wipeTopo();
  void rebuild(const costmap_2d::Costmap2DROS* costmap_ros);

public:
 Topomap(const costmap_2d::Costmap2DROS* costmap_ros, double world_x, double world_y);
 ~Topomap();

  void update(const costmap_2d::Costmap2DROS* costmap_ros, bool showDebug=false);
  std::vector<Waypoint*>* get_topomap();

  // Drawing functions
  void visualize_node(double x, double y, double scale, double r, double g,
    double b, double a, std::vector<visualization_msgs::Marker>* markers,
    int marker_id);

  void visualize_edge(double x1, double y1, double x2, double y2,
    double scale, double r, double g, double b, double a,
    std::vector<visualization_msgs::Marker>* markers, int marker_id);

  int publish_topomap(ros::Publisher* marker_pub, int marker_id);
};

#endif
