/*
  Node which subscribes to laser data and sonar data, and attempts to merge
  the two intelligently. (For example, to account for glass.)

  Info on the sonar implementation / setup:

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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <p2os_driver/SonarArray.h>

using namespace std;

class p3dxSonar {
public:
  double angle_degree;
  double angle_radian;
  double range;

  p3dxSonar(double degree) {
    angle_degree = degree;
    angle_radian = dtor(angle_degree);
    range = 1.0;
    ROS_INFO("New sonar with degree %f, radian %f", angle_degree, angle_radian);
  }

  // From rtv's Antix
  static double dtor(double d) {
    return d * M_PI / 180.0;
  }
};

/*
  Only work with front sonar for now
*/
class p3dxSonarArray {
public:
  vector<p3dxSonar> sonars;

  p3dxSonarArray() {
    // far left
    sonars.push_back( p3dxSonar(-90.0) );
    sonars.push_back( p3dxSonar(-50.0) );
    sonars.push_back( p3dxSonar(-30.0) );
    // front left
    sonars.push_back( p3dxSonar(-10.0) );
    // front right
    sonars.push_back( p3dxSonar(10.0) );
    sonars.push_back( p3dxSonar(30.0) );
    sonars.push_back( p3dxSonar(50.0) );
    sonars.push_back( p3dxSonar(90.0) );
  }
};

// This many sonars on the front. Assume index starts at 0
#define NUM_SONARS 8

ros::Publisher laser_pub;
p3dxSonarArray sonar_array;

/*
  Called whenever laser scan received
*/
void laser_callback(const sensor_msgs::LaserScan::ConstPtr & laser_scan) {
  // Take in laser scan, combine with sonar data if need be, and publish
  sensor_msgs::LaserScan sonarify_laser_scan = *laser_scan;

/*
  ROS_INFO("laser angle min %f angle max %f angle increment %f ranges.size %i",
    laser_scan->angle_min,
    laser_scan->angle_max,
    laser_scan->angle_increment,
    laser_scan->ranges.size()
  );
  */

  for (unsigned int i = 0; i < NUM_SONARS; i++) {
    // Assume sonar range only valid if < 5.0 meters
    if (sonar_array.sonars[i].range >= 5.0)
      continue;

    // Index into the laser range array depends on angle of sonar
    unsigned int laser_scan_index = floor(
      abs( laser_scan->angle_min - sonar_array.sonars[i].angle_radian )
      / laser_scan->angle_increment
    );

    // We only care if sonar range is less than laser range
    if (sonar_array.sonars[i].range < sonarify_laser_scan.ranges[laser_scan_index]) {
      sonarify_laser_scan.ranges[laser_scan_index] = sonar_array.sonars[i].range;

      ROS_INFO("laser range index %i (@ %f degrees) was %f but now %f",
        laser_scan_index,
        sonar_array.sonars[i].angle_degree,
        laser_scan->ranges[laser_scan_index],
        sonarify_laser_scan.ranges[laser_scan_index]
      );
    }
  }

  laser_pub.publish(sonarify_laser_scan);
}

/*
  Called whenever sonar array received
*/
void sonar_callback(const p2os_driver::SonarArray::ConstPtr & sonar_array_scan) {
  //ROS_WARN("sonar_array ranges size %i", sonar_array_scan->ranges.size());

  // Only look at front sonars
  for (unsigned int i = 0; i < NUM_SONARS; i++) {
    // Max valid range is 5 meters?
    sonar_array.sonars[i].range = sonar_array_scan->ranges[i];
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
