/*
  Find the furthest reachable distance from robot's starting pose (its
  home) given a map. Expects to be run with total map knowledge from
  map_server.
*/

#include <ros/ros.h>
#include <explore/explore.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "explore");
  ros::NodeHandle n;

  explore::Explore explore;

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    explore.find_furthest_reachable_point_from_home();
    explore.publishMap();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
