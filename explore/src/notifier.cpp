/*
 * Publish robot states and various important info (so we have it for
 * rosbag). Also publish to soundplay_node (sound_play) for text to speech.
 */

#include <explore/notifier.h>

Notifier::Notifier(ros::NodeHandle* n) {
  state_pub = n->advertise<std_msgs::String>("boldly_state", 1000);
}

void Notifier::publish(std::string state) {
  std::stringstream ss;
  ss << state;

  std_msgs::String msg;
  msg.data = ss.str();

  state_pub.publish(msg);
  ROS_INFO("Notifier publishing: %s", state.c_str());
}
