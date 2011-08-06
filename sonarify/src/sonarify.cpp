/*
  Node which subscribes to laser data and sonar data, and attempts to merge
  the two intelligently. (For example, to account for glass.)

  Note: We assume laser is at 0,0 on robot. We then manually transform the
  ranges of the sonar based on this. The sonar positions are given below.

  Info on the sonar implementation / setup:

  "The sonar's are ordered in the same manner as in the robot
  documentation (thus for p3dx it's 0 on the front left 1-6 on the front
  and 7 on the front right)."
  http://ros-users.122217.n3.nabble.com/sonar-data-for-navigation-stack-td986448.html

  P3DX spec says 8 sonar, 180 degrees @ 20 degree intervals each
  Stage says they have 30 degree fov.

  From Stage definition for P3DX sonars, seems to be
  [0] @ (0.069 -0.136) = -90 degrees (far left front sonar)
  [1] @ (0.114 -0.119) = -50 degrees
  [2] @ (0.148 -0.078) = -30 degrees
  [3] @ (0.166 -0.027) = -10 degrees (left front sonar)
  [4] @ (0.166 0.027) = 10 degrees (right front sonar)
  [5] @ (0.148 0.078) = 30 degrees
  [6] @ (0.114 0.119) = 50 degrees
  [7] @ (0.069 0.136) = 90 degrees
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <p2os_driver/SonarArray.h>

using namespace std;

class p3dxSonar {
public:
  // x, y from robot centre (assumed to be laser coordinates @ 0.0, 0.0)
  double x,
    y;
  // offset to add to range (i.e., to transform to look as though from laser)
  // this is distance to robot centre
  double offset;

  double angle_degree;
  double angle_radian;
  double range;

  p3dxSonar(double x, double y, double degree) : x(x), y(y), angle_degree(degree) {
    angle_radian = dtor(degree);
    range = 1.0;

    offset = sqrt( (0.0 - x) * (0.0 - x) + (0.0 - y) * (0.0 - y) );
    ROS_INFO("New sonar at (%f, %f) with degree %f, radian %f, distance from laser %f",
      x, y,
      angle_degree, angle_radian, offset
    );
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
    sonars.push_back( p3dxSonar(0.069, -0.136, -90.0) );
    sonars.push_back( p3dxSonar(0.114, -0.119, -50.0) );
    sonars.push_back( p3dxSonar(0.148, -0.078, -30.0) );
    // front left
    sonars.push_back( p3dxSonar(0.166, -0.027, -10.0) );
    // front right
    sonars.push_back( p3dxSonar(0.166, 0.027, 10.0) );
    sonars.push_back( p3dxSonar(0.148, 0.078, 30.0) );
    sonars.push_back( p3dxSonar(0.114, 0.119, 50.0) );
    sonars.push_back( p3dxSonar(0.069, 0.136, 90.0) );
  }
};

// This many sonars on the front. Assume index starts at 0
#define NUM_SONARS 8
// Samples to average to limit error
#define SAMPLE_COUNT 5
// We discount sonar readings when they are >= this value
#define SONAR_MAX_VALUE 2.0

// If this is 0, multiple laser ranges are replaced
#define SINGLE_RANGE_REPLACEMENT 0
// Number of indices into laser array to replace with sonar data
// (when found to be valuable) on either side of the centre of the
// sonar. So giving 5 here means replacing indices in the laser array
// from [centre_index - VALUE, centre_index + value]
// Only used when SINGLE_RANGE_REPLACEMENT is 0
// 12 = ceil(180 / 8 / 2) : 180 degrees / 8 sonars / 2 for indices being
// replaced on both sides of the centre index
#define NUM_REPLACEMENTS 12

//#define DEBUG

ros::Publisher laser_pub;
p3dxSonarArray sonar_array;
double samples[SAMPLE_COUNT][NUM_SONARS];

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
    // Our sonar is max range 5 meters, however, since the detection is
    // cone shaped, we restrict and only care about the sensor if it is
    // closer than 2 meters (pulled out of the air)
    // For reference, see http://comments.gmane.org/gmane.science.robotics.ros.user/6508
    if (sonar_array.sonars[i].range >= SONAR_MAX_VALUE)
      continue;

    // Index into the laser range array depends on angle of sonar
    int laser_scan_index = floor(
      abs( laser_scan->angle_min - sonar_array.sonars[i].angle_radian )
      / laser_scan->angle_increment
    );

    // We only care if sonar range is less than laser range
    // XXX May wish to include ranges of all laser points within the angle
    // of the sonar? See Lai, "Online Map [..]", 2005, page 3, column 2
    if (sonar_array.sonars[i].range < sonarify_laser_scan.ranges[laser_scan_index]) {
#if SINGLE_RANGE_REPLACEMENT
      sonarify_laser_scan.ranges[laser_scan_index] = sonar_array.sonars[i].range + sonar_array.sonars[i].offset;
#else
      for (int j = 0; j < NUM_REPLACEMENTS; j++) {
        if (laser_scan_index - j >= 0)
          sonarify_laser_scan.ranges[laser_scan_index-j] = sonar_array.sonars[i].range + sonar_array.sonars[i].offset;
        if (laser_scan_index + j < (int) sonarify_laser_scan.ranges.size())
          sonarify_laser_scan.ranges[laser_scan_index+j] = sonar_array.sonars[i].range + sonar_array.sonars[i].offset;
      }
#endif

#ifdef DEBUG
      ROS_INFO("laser range index %i (@ %f degrees) was %f but now %f",
        laser_scan_index,
        sonar_array.sonars[i].angle_degree,
        laser_scan->ranges[laser_scan_index],
        sonarify_laser_scan.ranges[laser_scan_index]
      );
#endif
    }
  }

  laser_pub.publish(sonarify_laser_scan);
}

/*
  Called whenever sonar array received
*/
void sonar_callback(const p2os_driver::SonarArray::ConstPtr & sonar_array_scan) {
  //ROS_WARN("sonar_array ranges size %i", sonar_array_scan->ranges.size());
  static bool initial = true;

  // Maintain list of five most recent samples
  if(initial) {
    for(unsigned i = 0; i < SAMPLE_COUNT; ++i) {
      for(unsigned j = 0; j < NUM_SONARS; ++j) {
	samples[i][j] = sonar_array_scan->ranges[j];
      }
    }
  } else {
    for(unsigned i = 1; i < SAMPLE_COUNT; ++i) {
      for(unsigned j = 0; j < NUM_SONARS; ++j) {
	samples[i][j] = samples[i-1][j];
      }
    }
    for(unsigned j = 0; j < NUM_SONARS; ++j) {
      samples[0][j] = sonar_array_scan->ranges[j];
    }
  }

  // Only look at front sonars
  for (unsigned int i = 0; i < NUM_SONARS; i++) {
    // Max valid range is 5 meters?
    double mean = 0;
    for(unsigned j = 0; j < SAMPLE_COUNT; ++j) {
      mean += samples[j][i];
    }
    mean /= (double)SAMPLE_COUNT;
    sonar_array.sonars[i].range = mean;
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
