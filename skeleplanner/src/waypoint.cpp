/* (c) Peter Neufeld 2011 */

#include "skeleplanner/waypoint.h"

using namespace std;

inline float dist(int x1, int y1, int x2, int y2)
{
  return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

//calculate the minimum space around a point
int Topomap::calcSpace(int x, int y, const costmap_2d::Costmap2D& costmap) {
  if (memo[x][y] != -1)
    return memo[x][y];

  int rtn = INT_MAX;
  int startrad = 0;

  //DPify!
  /* XXX Note: this was inside if (... && false) {
  //note: int/float dist trunc. errors mean be conservative by 3(more?)
  startrad = INT_MAX;
  for(int i = -1; i <= 1; i++) {
    for(int j = -1; j <= 1; j++) {
      if(!inBounds(x+i, y+j, costmap))
        continue;
      if((i != 0 || j != 0) && memo[x+i][y+j] != -1) {
        startrad = min(startrad, memo[x+i][y+j]-3);
      }
    }
  }
  */

  if(startrad == INT_MAX)
    startrad = 0;
  startrad = max(startrad, 0);

  for(int rad = startrad; rad < INT_MAX; rad++)
  {
      for(int i = x - rad; i <= x + rad; i++)
      {
        for(int j = y - rad; j <= y + rad; j += (i == x-rad || i == x+rad ? 1 : max(1, 2*rad)))
        {
          if(!inBounds(i, j, costmap))
            continue;
          if(costmap.getCost(i, j) > IMPASSABLE_THRESH)
            rtn = min((float)rtn, dist(i, j, x, y));
        }
      }
        
      if(rtn != INT_MAX && rad > rtn) {
        memo[x][y] = rtn;
        return rtn;
      }
  }
  return rtn;
}

/*
bool Topomap::straightClear(int x1, int y1, int x2, int y2,
  const costmap_2d::Costmap2D& costmap)
{
  if(x2-x1 == 0)
  {
    //annoying vertical line
    int yinc = (y2-y1 > 0 ? 1 : -1);
    for(int y = y1; y != y2; y += yinc)
    {
      if(calcSpace(x1, y, costmap) <= 1)
        return false;
    }
    return true;
  }

  //ensure left-to-right
  if(x2-x1 < 0) {
    int tx = x1;
    int ty = y1;
    x1 = x2;
    y1 = y2;
    x2 = tx;
    y2 = ty;
  }

  float slope = (float)(y2-y1)/(x2-x1);
  float xinc = 1;
  if(abs(slope) > 1)
    xinc = abs(1.0 / slope);

  //dont check the points themselves (?)
  for(float x = 0; x <= (x2-x1); x += xinc)
  {
    if(calcSpace((int)(x1+x), (int)(y1+(slope*x)), costmap) <= 1)
    {
      return false;
    }
  }

  return true;
}
*/

MapWaypoint Topomap::waypointBest(int x, int y, const costmap_2d::Costmap2D& costmap) {
  int newm = -1;
  int newx = -1;
  int newy = -1;

  for(int rad = WAYPOINTSPACE-1; rad >= 0; rad--)
  {
    for(int i = x - rad; i <= x + rad; i++)
    {
      for(int j = y - rad; j <= y + rad; j += (i == x-rad || i == x+rad ? 1 : max(1, 2*rad)))
      {
        if(!inBounds(i, j, costmap))
          continue;

        if(dist(i, j, x, y) > WAYPOINTSPACE-1)
          continue;

        //make sure a straight line clear
        //if(!straightClear(i, j, x, y, costmap, memo))
        //  continue;

        //straightline clear on orig image
        int tmps = calcSpace(i, j, costmap);
        //int otmps = tmps;
        int innerWaypoints = 0;
        double innerDistances = 0.0;
        for(vector<MapWaypoint*>::const_iterator w = map_topomap.begin();
          w != map_topomap.end(); ++w)
        {
          tmps = min(tmps, (int)(dist(i, j, (*w)->x, (*w)->y)));
          if(dist(i, j, (*w)->x, (*w)->y) <= WAYPOINTSPACE)
          {
            innerWaypoints++;
            innerDistances += dist(i, j, (*w)->x, (*w)->y);
          }
        }
                  
        //don't use new waypoints that are clustered around current ones
        if(innerWaypoints >= 2 || tmps <= 1)
          continue;
                      
        if(tmps > newm)
        {
          newm = tmps;
          newx = i;
          newy = j;

          if(newm >= rad)
            break;
        }
      }

      if(newm >= rad)
        break;
    }
    if(newm >= rad)
      break;
  }
    
  if(newm == -1)
    return MapWaypoint(-1, -1, -1);

  return MapWaypoint(newx, newy, newm);
}

Topomap::Topomap(const costmap_2d::Costmap2DROS* costmap_ros, double start_world_x,
  double start_world_y)
  : start_world_x(start_world_x), start_world_y(start_world_y),
    r_(255.0), g_(255.0), b_(0.0), a_(1.0) // yellow
{
  rebuild(costmap_ros);
}

Topomap::~Topomap() {
  wipeTopo();
}

void Topomap::wipeTopo() {
  for (std::vector<MapWaypoint*>::iterator it = map_topomap.begin();
    it < map_topomap.end(); ++it)
  {
    delete *it;
  }
  for (std::vector<Waypoint*>::iterator it = topomap.begin();
    it < topomap.end(); ++it)
  {
    delete *it;
  }

  map_topomap.clear();
  topomap.clear();

  memo.clear();
  ignore.clear();
}

/*
  Start topomap from scratch (for when map size changes)
*/
void Topomap::rebuild(const costmap_2d::Costmap2DROS* costmap_ros) {
  wipeTopo();

  costmap_2d::Costmap2D costmap;
  costmap_ros->getCostmapCopy(costmap);
  costmap.worldToMap(start_world_x, start_world_y, start_map_x, start_map_y);

  home = new MapWaypoint(start_map_x, start_map_y, 0);
  map_topomap.push_back(home);
  topomap.push_back(new Waypoint(start_world_x, start_world_y, 0));

  // Allocate a 2 dimensional array called memo, which is of size
  // costmap cells x by costmap cells y
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
    std::vector<int> column;
    column.resize(costmap.getSizeInCellsY());
    memo.push_back( column );
  }

  costmap_size_x = costmap.getSizeInCellsX();
  costmap_size_y = costmap.getSizeInCellsY();
}

//this function takes a starting point and creates a topological map of the image from that point.
//the function stops when it cannot make a new waypoint without placing one in an area already covered by waypoints
//however you can stop is short by specifying a maxPoints. Make maxPoints large (INT_MAX) if you want it to find
//the complete topological map.
//The other functions are workhorse functions, and are not meant for use outside of this function.

// This was topoFromPoint().
void Topomap::update(const costmap_2d::Costmap2DROS* costmap_ros, bool showDebug) {
  costmap_2d::Costmap2D costmap;
  costmap_ros->getCostmapCopy(costmap);

  // If costmap size has changed, rebuild whole thing (or we can segfault)
  if (costmap.getSizeInCellsX() != costmap_size_x || costmap.getSizeInCellsY() != costmap_size_y) {
    rebuild(costmap_ros);
    ROS_WARN("Rebuilt topomap.");
  }

  // reset the ignore array each time in case the map has changed enough
  ignore.clear();
  for (unsigned i = 0; i < map_topomap.size(); ++i) {
    ignore.push_back(false);
  }

  // reset the memo too
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
      memo[i][j] = -1;
    }
  }

  // add waypoints
  for (unsigned int i = 0; true; ++i)
  {
    if (showDebug && (i+1) % 5 == 0) {
      std::cout << "Finding Waypoint " << (i+1) << "..." << std::endl;
    }

    MapWaypoint* maxWaypoint = NULL;
    Waypoint* worldMax = NULL;
    MapWaypoint* newway = new MapWaypoint(0, 0, 0);
    Waypoint* newworld = new Waypoint(0, 0, 0);

    while (maxWaypoint == NULL)
    {
      int besti;
      // always find new max, as new waypoints can change the max. Assume [0]==home
      for (unsigned int j = 0; j < map_topomap.size(); j++)
      {
        MapWaypoint* tmp = map_topomap[j];
        Waypoint* worldtmp = topomap[j];
        if (!ignore[j] && (maxWaypoint == NULL
                           || (tmp->space >= maxWaypoint->space
                               && costmap.getCost(tmp->x, tmp->y) < PASSABLE_THRESH)))
        {
          maxWaypoint = tmp;
          worldMax = worldtmp;
          besti = j;
        }
      }

      // no more space?
      if (maxWaypoint == NULL)
      {
        delete newway;
        delete newworld;
        break;
      }
      
      /*
        XXX There was a reset of all memo[i][j] to -1 here... on purpose?
      */

      *newway = waypointBest(maxWaypoint->x, maxWaypoint->y, costmap);

      if (newway->x == -1) {
        ignore[besti] = true;
        maxWaypoint = NULL;
        worldMax = NULL;
        continue;
      }
    }

    if (maxWaypoint == NULL)
      break;

    newworld->space = newway->space;
    costmap.mapToWorld(newway->x, newway->y, newworld->x, newworld->y);

    ignore.push_back(false);
    topomap.push_back(newworld);
    map_topomap.push_back(newway);

    // only recalc waypointBest for nearby waypoints
    for(unsigned int j = 0; j < map_topomap.size() - 1; ++j)
    {
      MapWaypoint* tmp = map_topomap[j];

      if(tmp->x == newway->x && tmp->y == newway->y)
      {
        if(showDebug)
          std::cout << "WARNING: waypoint overlap" << std::endl;
        continue;
      }

      if (dist(tmp->x, tmp->y, newway->x, newway->y) <= WAYPOINTSPACE) {
        MapWaypoint tmp_mapwaypoint = waypointBest(tmp->x, tmp->y, costmap);
        tmp->space = tmp_mapwaypoint.space;
      }
    }

    maxWaypoint->neighbors.push_back(newway);
    worldMax->neighbors.push_back(newworld);
    newway->neighbors.push_back(maxWaypoint);
    newworld->neighbors.push_back(worldMax);
  }
}

std::vector<Waypoint*>* Topomap::get_topomap() {
  return &topomap;
}

/*
  Drawing functions
*/

/*
  Add a sphere marker at x, y to markers vector
  (Based on visualizeNode() in explore/loop_closure)
*/
void Topomap::visualize_node(double x, double y, double scale, double r, double g,
  double b, double a, std::vector<visualization_msgs::Marker>* markers, int marker_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "skeletester";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = x;
  marker.pose.position.y = y;

  // thick point like in loop_closure is 0.5
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  markers->push_back(marker);
}

/*
  Add a line marker between the two sets of points to markers vector
  (Based on visualizeEdge() in explore/loop_closure)
*/
void Topomap::visualize_edge(double x1, double y1, double x2, double y2,
  double scale, double r, double g, double b, double a,
  std::vector<visualization_msgs::Marker>* markers, int marker_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "skeletester";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p;
  p.x = x1;
  p.y = y1;
  marker.points.push_back(p);

  p.x = x2;
  p.y = y2;
  marker.points.push_back(p);

  // thick line like in loop closure is 0.25
  // only scale.x used
  marker.scale.x = scale;
  // red is 1, 0, 0, 1
  // blue 0, 1, 0, 1?
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  markers->push_back(marker);
}

/*
  Uses the above 2 functions to publish the current topomap
*/
int Topomap::publish_topomap(ros::Publisher* marker_pub, int marker_id) {
  std::vector<visualization_msgs::Marker> markers;

#ifdef DEBUG
  ROS_WARN("Topomap publishing %d nodes in topomap", topomap->size());
#endif
  // Markers for each node in the topological map
  for (std::vector<Waypoint*>::const_iterator it = topomap.begin();
    it != topomap.end(); ++it)
  {
    // 0.5 scale, red colour: 1.0, 0.0, 0.0, 1.0
    //visualize_node( (*it)->x, (*it)->y, 0.5, r_, g_, b_, a_, &markers);
    visualize_node( (*it)->x, (*it)->y, 0.3, r_, g_, b_, a_, &markers, marker_id);
    marker_id++;
  }

  // Markers for the links between each node
  for (std::vector<Waypoint*>::const_iterator it = topomap.begin();
    it != topomap.end(); ++it)
  {
    // For each neighbour of this node, add an edge to it
    for (std::vector<Waypoint*>::const_iterator it2 = (*it)->neighbors.begin();
      it2 != (*it)->neighbors.end(); ++it2)
    {
      // 0.25 scale
      //visualize_edge((*it)->x, (*it)->y, (*it2)->x, (*it2)->y, 0.25, r_, g_, b_, a_, &markers);
      visualize_edge((*it)->x, (*it)->y, (*it2)->x, (*it2)->y, 0.2, r_, g_, b_, a_, &markers, marker_id);
      marker_id++;
    }
  }

  // Delete any markers past that which we just published (stops strange
  // visualisation behaviour in rviz)
  // Taken from getVisualizationMarkers() in explore_frontier
  // Doesn't seem to work.
  /*
  for ( ; marker_id < marker_id_last; ++marker_id) {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
    m.id = marker_id;
    markers.push_back(visualization_msgs::Marker(m));
  }
  marker_id_last = markers.size();
  */

  // Now publish all of these markers
  for (std::vector<visualization_msgs::Marker>::iterator it = markers.begin();
    it != markers.end();
    ++it)
  {
    it->lifetime = ros::Duration(1);
    marker_pub->publish( *it );
  }

  return marker_id;
}
