/*
 * Publish robot states and various important info (so we have it for
 * rosbag). Also publish to soundplay_node (sound_play) for text to speech.
 */

#include <explore/notifier.h>

Notifier::Notifier(ros::NodeHandle* n) {
  state_pub = n->advertise<std_msgs::String>("boldly_state", 1000);
  sound_pub = n->advertise<sound_play::SoundRequest>("robotsound", 1000);
}

void Notifier::publish(std::string state) {
  std::stringstream ss;
  ss << state;

  std_msgs::String msg;
  msg.data = ss.str();

  // Publish string
  state_pub.publish(msg);

  // Publish to audio
  sound_play::SoundRequest sound_request;
  sound_request.sound = sound_play::SoundRequest::SAY;
  sound_request.command = sound_play::SoundRequest::PLAY_ONCE;
  sound_request.arg = state;

  sound_pub.publish( sound_request );

  ROS_INFO("Notifier publishing: %s", state.c_str());
}
