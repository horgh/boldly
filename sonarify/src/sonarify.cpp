/*
  Node which subscribes to laser data and sonar data, and attempts to merge
  the two intelligently. (For example, to account for glass.)
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <p2os_driver/SonarArray.h>

using namespace std;

ros::Publisher laser_pub;

/*
  Called whenever laser scan received
*/
void laser_callback(const sensor_msgs::LaserScan::ConstPtr & laser_scan) {
  // Take in laser scan, combine with sonar data if need be, and publish
  sensor_msgs::LaserScan sonarify_laser_scan;

  ROS_INFO("laser angle min %f angle max %f angle increment %f ranges.size %i",
    laser_scan->angle_min,
    laser_scan->angle_max,
    laser_scan->angle_increment,
    laser_scan->ranges.size()
  );

  laser_pub.publish(sonarify_laser_scan);
}

/*
  Called whenever sonar array received

  "The sonar's are ordered in the same manner as in the robot
  documentation (thus for p3dx it's 0 on the front left 1-6 on the front
  and 7 on the front right)."
  http://ros-users.122217.n3.nabble.com/sonar-data-for-navigation-stack-td986448.html

  P3DX spec says 8 sonar, 180 degrees @ 20 degree intervals each

  From Stage definition for P3DX sonars, seems to be
  [0] = -90 degrees (far left front sonar)
  [1] = -50
  [2] = -30
  [3] = -10 (left front sonar)
  [4] = 10 (right front sonar)
  [5] = 30
  [6] = 50
  [7] = 90
*/
void sonar_callback(const p2os_driver::SonarArray::ConstPtr & sonar_array) {
  ROS_WARN("sonar_array ranges size %i", sonar_array->ranges.size());

  // Only look at front sonars
  for (unsigned int i = 0; i <= 7; i++) {
    // Max valid range is 5 meters?
    ROS_WARN("sonar %i has range %f", i, sonar_array->ranges[i]);
  }
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "sonarify");
  ros::NodeHandle n;

  laser_pub = n.advertise<sensor_msgs::LaserScan>("sonarify_scan", 1000);

  ros::Subscriber laser_sub = n.subscribe("scan", 1000, laser_callback);
  ros::Subscriber sonar_sub = n.subscribe("sonar", 1000, sonar_callback);

  ROS_INFO("Sonarify ready.");

  ros::spin();
/*
  ros::Rate loop_rate(10);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
  }
*/

  return 0;
}
