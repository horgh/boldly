/*
  Tester for skeleplanner

  Take in a costmap (such as from map_server) and publish data relating
  to it so we can visualize it in rviz
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "skeleplanner/skeleplanner.h"

int marker_id;

/*
  Add a sphere marker at x, y to markers vector
  (Based on visualizeNode() in explore/loop_closure)
*/
void visualize_node(double x, double y, double scale, double r, double g,
  double b, double a, std::vector<visualization_msgs::Marker>* markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "skeletester";
  marker.id = marker_id++;
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
void visualize_edge(double x1, double y1, double x2, double y2,
  double scale, double r, double g, double b, double a,
  std::vector<visualization_msgs::Marker>* markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "skeletester";
  marker.id = marker_id++;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p;
  p.x = x1;
  p.y = y1;
  marker.points.push_back(p);

  p.x = x1;
  p.y = y2;
  marker.points.push_back(p);

  // thick line like in loop closure is 0.25
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  // red is 1, 0, 0, 1
  // blue 0, 1, 0, 1?
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  markers->push_back(marker);
}

/*
  Based on publishMap() from explore
*/
void publish_map(costmap_2d::Costmap2DROS* costmap_2d_ros, ros::Publisher* map_pub) {
  nav_msgs::OccupancyGrid map;
  map.header.stamp = ros::Time::now();

  costmap_2d::Costmap2D costmap;
  costmap_2d_ros->getCostmapCopy(costmap);

  map.info.width = costmap.getSizeInCellsX();
  map.info.height = costmap.getSizeInCellsY();
  map.info.resolution = costmap.getResolution();
  map.info.origin.position.x = costmap.getOriginX();
  map.info.origin.position.y = costmap.getOriginY();
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0;
  map.info.origin.orientation.y = 0;
  map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;

  int size = map.info.width * map.info.height;
  const unsigned char* char_map = costmap.getCharMap();

  map.data.resize(size);
  for (int i = 0; i < size; ++i) {
    if (char_map[i] == costmap_2d::NO_INFORMATION)
      map.data[i] = -1;
    else if (char_map[i] == costmap_2d::LETHAL_OBSTACLE)
      map.data[i] = 100;
    else
      map.data[i] = 0;
  }

  map_pub->publish(map);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "skeletester");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("skeletester_marker", 10);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("skeletester_map", 1, true);

  tf::TransformListener tf_(ros::Duration(10.0));
  costmap_2d::Costmap2DROS* costmap_ = new costmap_2d::Costmap2DROS(std::string("skeletester_costmap"), tf_);


  ROS_WARN("Initialising SkelePlanner object...");
  SkelePlanner skele_planner;
  skele_planner.initialize("skeletester", costmap_);
  ROS_WARN("SkelePlanner object initialised.");

  marker_id = 0;

  // 1hz
  ros::Rate loop_rate(1);

  while ( ros::ok() ) {
    ROS_WARN("Skeletester publishing...");

    // First publish the current occupancy grid
    publish_map(costmap_, &map_pub);


    // Markers for each node in the topological map
    std::vector<visualization_msgs::Marker> markers;
    for (std::vector<Waypoint*>::const_iterator it = skele_planner.topomap->begin();
      it != skele_planner.topomap->end(); ++it)
    {
      // 0.5 scale, red colour
      visualize_node( (*it)->x, (*it)->y, 0.5, 1.0, 0.0, 0.0, 1.0, &markers);
    }

    // Markers for the links between each node
    for (std::vector<Waypoint*>::const_iterator it = skele_planner.topomap->begin();
      it != skele_planner.topomap->end(); ++it)
    {
      // For each neighbour of this node, add an edge to it
      for (std::vector<Waypoint*>::const_iterator it2 = (*it)->neighbors.begin();
        it2 != (*it)->neighbors.end(); ++it2)
      {
        // 0.25 scale, red colour
        visualize_edge((*it)->x, (*it)->y, (*it2)->x, (*it2)->y, 0.25, 1.0, 0.0, 0.0, 1.0, &markers);
      }
    }

    // Now publish all of these markers
    for (std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin();
      it != markers.end(); ++it)
    {
      marker_pub.publish( *it );
    }

    // Additionally we can publish chosen path
    // makePlan() then add nodes & edges with different colours

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
