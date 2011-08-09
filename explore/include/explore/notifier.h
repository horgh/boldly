#ifndef NOTIFIER_H_
#define NOTIFIER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequest.h>

class Notifier {
public:
  ros::Publisher state_pub;
  ros::Publisher sound_pub;

  Notifier(ros::NodeHandle* n);

  void publish(std::string state);
};

#endif
