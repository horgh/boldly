/*
  Tester for skeleplanner

  Take in a costmap (such as from map_server) and publish data relating
  to it so we can visualize it in rviz
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "skeleplanner/skeleplanner.h"

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

/*
  Test expand_plan() in skeleplanner
*/
void test_expand_plan(SkelePlanner *skele_planner) {
  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose.position.x = 0.0;
  pose_stamped.pose.position.y = 0.0;
  plan.push_back( pose_stamped );
  pose_stamped.pose.position.x = 1.0;
  pose_stamped.pose.position.y = 1.0;
  plan.push_back( pose_stamped );

  skele_planner->expand_plan( &plan );

  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
    it != plan.end();
    ++it)
  {
    ROS_WARN("%f, %f", it->pose.position.x, it->pose.position.y);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "skeletester");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("skeletester_marker", 100);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("skeletester_map", 1, true);

  tf::TransformListener tf_(ros::Duration(10.0));
  costmap_2d::Costmap2DROS* costmap_ = new costmap_2d::Costmap2DROS(std::string("skeletester_costmap"), tf_);


  ROS_WARN("Initialising SkelePlanner object...");
  SkelePlanner skele_planner;
  skele_planner.initialize("skeletester", costmap_);
  skele_planner.set_topomap_origin(0.0, 0.0);

  // Make a plan from robot's location to its location
  tf::Stamped<tf::Pose> robot_pose;
  costmap_->getRobotPose(robot_pose);
  geometry_msgs::PoseStamped robot_pose_stamped;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_stamped);

  std::vector<geometry_msgs::PoseStamped> plan;
  skele_planner.makePlan(robot_pose_stamped, robot_pose_stamped, plan);

  ROS_WARN("SkelePlanner object initialised.");

  test_expand_plan(&skele_planner);

  // 1hz
  ros::Rate loop_rate(1);


  while ( ros::ok() ) {
    ROS_WARN("Skeletester publishing...");

    // First publish the current occupancy grid
    publish_map(costmap_, &map_pub);
  
    int marker_id = 0;

    // Publish the topomap
    marker_id = skele_planner.publish_topomap(&marker_pub, marker_id);

    // Additionally we can publish chosen path along topomap
    // Take the plan from makePlan() and then add nodes & edges with
    // different colours

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
