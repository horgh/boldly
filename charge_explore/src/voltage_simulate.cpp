/*
  Voltage simulation node

  Publishes values corresponding to a simulated battery from a given
  max to a given cut off.

  Once the cut off is reached, cease decreasing and wait for a 'charge
  complete' signal to be published before beginning from the start.
*/

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <p2os_driver/BatteryState.h>

// XXX should match p2os
#define VOLTAGE_INITIAL 15.0
#define VOLTAGE_CUTOFF 11.0

using namespace std;

p2os_driver::BatteryState battery_state;

/*
  Message indicating charging is complete.
*/
void charge_complete_callback(const std_msgs::Empty::ConstPtr & msg) {
  ROS_INFO("Got signal we are charged.");
  battery_state.voltage = VOLTAGE_INITIAL;
}

int main(int argc, char** argv) {
  ROS_INFO("p2os Voltage simulator launching...");

  ros::init(argc, argv, "voltage_simulate");
  ros::NodeHandle n;

  ros::Publisher voltage_pub =
    n.advertise<p2os_driver::BatteryState>("battery_state", 1000);

  ros::Subscriber charge_sub =
    n.subscribe("charge_complete", 1000, charge_complete_callback);
  
  ros::Rate loop_rate(1);
  battery_state.voltage = VOLTAGE_INITIAL;

  while ( ros::ok() ) {
    ROS_INFO("Publishing voltage %f...", battery_state.voltage);
    voltage_pub.publish(battery_state);

    // Decrease by 0.1 every second if we're not at cut off
    if (battery_state.voltage > VOLTAGE_CUTOFF)
      //battery_state.voltage += -0.1;
      battery_state.voltage += -0.01;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
