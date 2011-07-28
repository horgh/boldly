/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GetMap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>
#include <explore/explore_frontier.h>
#include <explore/loop_closure.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>
#include <cmath>
#include <p2os_driver/BatteryState.h>
#include <std_msgs/Empty.h>
#include <skeleplanner/waypoint.h>

namespace explore {

/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the robot base to explore its environment.
 */
class Explore {
public:

  void charge_complete_callback(const std_msgs::Empty::ConstPtr & msg);
  void battery_state_callback(const p2os_driver::BatteryState::ConstPtr & msg);
  void visualize_arrow(int id, double x, double y, double scale, double r,
    double g, double b, double a, std::vector<visualization_msgs::Marker>* markers,
    std::string ns);
  void visualize_blacklisted();
  double distance_between_coords(double x1, double y1, double x2, double y2);
  void visualize_plan(std::vector<geometry_msgs::PoseStamped>& plan);
  void find_furthest_point();

  /**
   * @brief  Constructor
   * @return
   */
  Explore();

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~Explore();

  /**
   * @brief  Runs whenever a new goal is sent to the move_base
   * @param goal The goal to pursue
   * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
   * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
   */
//  virtual robot_actions::ResultStatus execute(const ExploreGoal& goal, ExploreFeedback& feedback);
  void execute();

  void spin();

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  /**
   * @brief  Publish a goal to the visualizer
   * @param  goal The goal to visualize
   */
  void publishGoal(const geometry_msgs::Pose& goal);

  /**
   * @brief publish map
   */
  void publishMap();

  void reachedGoal(const actionlib::SimpleClientGoalState& status, const move_base_msgs::MoveBaseResultConstPtr& result, geometry_msgs::PoseStamped frontier_goal);

  /**
   * @brief  Resets the costmaps to the static map outside a given window
   */
//  void resetCostmaps(double size_x, double size_y);

  bool goalOnBlacklist(const geometry_msgs::PoseStamped& goal);

  void removeUnsafeFrontiers(std::vector<geometry_msgs::Pose> * goals);

  int secsToHome();
  //void updateTimeToHome();
  geometry_msgs::PoseStamped currentPoseStamped();
  double distanceForPlan(geometry_msgs::PoseStamped * pose, std::vector<geometry_msgs::PoseStamped> * plan);
  double angleChangeForPlan(geometry_msgs::PoseStamped * pose, std::vector<geometry_msgs::PoseStamped> * plan);
  bool closeEnoughToPoseStamped(geometry_msgs::PoseStamped * pose_stamped);
  double distanceBetweenTwoPoses(geometry_msgs::Pose * pose1, geometry_msgs::Pose * pose2);
  bool atHome();
  bool atGoal();
  void checkIfStuck();
  void moveRandomDirection();
  int timeToTravel(geometry_msgs::PoseStamped* source_pose_stamped, geometry_msgs::Pose* target_pose);
  bool shouldGoHome_dynamic();
  //bool shouldGoHome_fast();
  int batteryTimeRemaining();
  void goHome();
  void reachedHome();
  void reachedHomeCallback(const actionlib::SimpleClientGoalState& status,
    const move_base_msgs::MoveBaseResultConstPtr& result,
    geometry_msgs::PoseStamped goal
  );
  bool atCriticalVoltage();
  void updateBatteryDuration();
  void setState(int new_state);


  ros::NodeHandle node_;
  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* explore_costmap_ros_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

  navfn::NavfnROS* planner_;
  std::string robot_base_frame_;

  ros::Publisher marker_publisher_;
  ros::Publisher marker_array_publisher_;
  ros::Publisher topomap_marker_publisher_;
  ros::Publisher map_publisher_;
  ros::Subscriber voltage_subscriber_;
  ros::Subscriber charged_subscriber_;

  ExploreFrontier* explorer_;

  tf::Stamped<tf::Pose> global_pose_;
  double planner_frequency_;
  int    visualize_;
  LoopClosure* loop_closure_;
  std::vector<geometry_msgs::PoseStamped> frontier_blacklist_;
  geometry_msgs::PoseStamped prev_goal_;
  geometry_msgs::PoseStamped current_goal_pose_stamped_;
  unsigned int prev_plan_size_;
  double time_since_progress_, progress_timeout_;
  double time_following_plan_;
  double potential_scale_, orientation_scale_, gain_scale_;
  boost::mutex client_mutex_;
  bool   close_loops_;

  // Location of our home base to charge
  geometry_msgs::PoseStamped home_pose_msg;

  // Last position, updated every iteration
  geometry_msgs::PoseStamped last_pose;

  // Robot's max speed
  double max_vel_x;
  // Robot's max turn speed
  double max_vel_th;

  // Last read voltage
  double battery_voltage;
  // Cutoff voltage
  double voltage_cutoff;

  // robot state
  int state;

  // Time in seconds for margin of battery life to return to charge
  int battery_safety_margin;

  ros::Time last_time_charged;
  ros::Duration battery_duration;

  // Current time to go home. Periodically updated.
  int secs_to_go_home;
  ros::Time last_time_update_secs_to_go_home;

  Topomap* topomap_;

  // Track last time we calculated and sent a new goal
  ros::Time last_goal_chosen;

  // Track how many exploration runs we have done
  int exploration_runs_;

  // Each marker needs unique id
  int topomap_publisher_marker_id;

  // Plan we're trying to carry out. Made in makePlan()
  std::vector<geometry_msgs::PoseStamped> current_plan_;
};

}

#endif

