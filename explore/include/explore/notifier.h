#ifndef NOTIFIER_H_
#define NOTIFIER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

class Notifier {
public:
  ros::Publisher state_pub;

  Notifier(ros::NodeHandle* n);

  void publish(std::string state);
};

#endif
