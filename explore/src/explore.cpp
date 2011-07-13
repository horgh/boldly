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

// Compile with some debugging
#define DEBUG

// If uncommented, use frontier comparison algorithm where we don't try to go to those
// frontiers which we deem unsafe due to battery life, but may go to others intead
//#define FRONTIER_COMPARE

// Time until we decide we are stuck in seconds
#define PROGRESS_TIMEOUT 10.0

// Voltage that we consider too low and must charge. This should
// be the same as robot starts sounding alarums
#define VOLTAGE_WARNING 12.0

// Our battery is a timer. Makes us go home when its up
#define BATTERY_TIMER

// Always periodically return (uses INITIAL_EXPLORE_TIME) rather than
// operate with our voltage logic. Requires BATTERY_TIMER as well.
//#define CONSTANT_BATTERY_TIME

// Life of battery in seconds
#define BATTERY_TIME 120
// Start heading back with at least this margin of safety (wrt battery time remaining)
#define MIN_BATTERY_SAFETY_MARGIN 10
// Starting safety margin
#define START_SAFETY_MARGIN 0

// Initial explore time before we return home (initial behaviour)
#define INITIAL_EXPLORE_TIME 30

/*
  Possible global states
*/
// We're in initial state: return to home frequently, but don't charge
#define GLOBAL_STATE_INITIAL 0
// Explore, return home only to charge
#define GLOBAL_STATE_EXPLORE 1

/*
  Possible local states
*/
// Heading home
#define STATE_HEADING_HOME 0
// Exploring
#define STATE_EXPLORING 1
// Waiting for a new exploration goal
#define STATE_WAITING_FOR_GOAL 2
// Waiting at home for charging to complete
#define STATE_CHARGING 3
// Completed whatever state we're in (e.g. no more exploration goals)
#define STATE_DONE 4

#include <explore/explore.h>
#include <explore/explore_frontier.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace costmap_2d;
using namespace navfn;
using namespace visualization_msgs;
using namespace geometry_msgs;

namespace explore {

double sign(double x) {
  return x < 0.0 ? -1.0 : 1.0;
}

void Explore::charge_complete_callback(const std_msgs::Empty::ConstPtr & msg) {
  if ( state == STATE_CHARGING ) {
    ROS_WARN("Got signal we are charged.");
    setLocalState(STATE_WAITING_FOR_GOAL);
    last_time_at_home = ros::Time::now();

    time_to_home = 0;
    last_time_update_time_to_home = ros::Time::now();

    last_time_charged = ros::Time::now();
  }
}

void Explore::battery_state_callback(const p2os_driver::BatteryState::ConstPtr & msg) {
  //ROS_INFO("Got battery state (voltage)");
  battery_voltage = msg->voltage;
}

Explore::Explore() :
  node_(),
  tf_(ros::Duration(10.0)),
  explore_costmap_ros_(NULL),
  move_base_client_("move_base"),
  planner_(NULL),
  done_exploring_(false),
  explorer_(NULL),
  prev_plan_size_(0)
{
  ros::NodeHandle private_nh("~");

  marker_publisher_ = node_.advertise<Marker>("visualization_marker",10);
  marker_array_publisher_ = node_.advertise<MarkerArray>("visualization_marker_array",10);
  topomap_marker_publisher_ = node_.advertise<Marker>("topomap_marker", 1000);
  map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  voltage_subscriber_ = node_.subscribe<p2os_driver::BatteryState>("battery_state", 1, &Explore::battery_state_callback, this);
  charged_subscriber_ = node_.subscribe<std_msgs::Empty>("charge_complete", 1, &Explore::charge_complete_callback, this);
  battery_voltage = -1.0;

  private_nh.param("navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("planner_frequency", planner_frequency_, 1.0);
  private_nh.param("progress_timeout", progress_timeout_, PROGRESS_TIMEOUT);
  private_nh.param("visualize", visualize_, 1);
  double loop_closure_addition_dist_min;
  double loop_closure_loop_dist_min;
  double loop_closure_loop_dist_max;
  double loop_closure_slam_entropy_max;
  private_nh.param("close_loops", close_loops_, false); // TODO: switch default to true once gmapping 1.1 has been released
  private_nh.param("loop_closure_addition_dist_min", loop_closure_addition_dist_min, 2.5);
  private_nh.param("loop_closure_loop_dist_min", loop_closure_loop_dist_min, 6.0);
  private_nh.param("loop_closure_loop_dist_max", loop_closure_loop_dist_max, 20.0);
  private_nh.param("loop_closure_slam_entropy_max", loop_closure_slam_entropy_max, 3.0);
  private_nh.param("potential_scale", potential_scale_, 1e-3);
  private_nh.param("orientation_scale", orientation_scale_, 0.0); // TODO: set this back to 0.318 once getOrientationChange is fixed
  private_nh.param("gain_scale", gain_scale_, 1.0);

  explore_costmap_ros_ = new Costmap2DROS(std::string("explore_costmap"), tf_);
  explore_costmap_ros_->clearRobotFootprint();

  planner_ = new navfn::NavfnROS(std::string("explore_planner"), explore_costmap_ros_);
  explorer_ = new ExploreFrontier();
  loop_closure_ = new LoopClosure(loop_closure_addition_dist_min,
        loop_closure_loop_dist_min,
        loop_closure_loop_dist_max,
        loop_closure_slam_entropy_max,
        planner_frequency_,
        move_base_client_,
        *explore_costmap_ros_,
        client_mutex_);

  // Assume start position is our home. Record it
  tf::Stamped<tf::Pose> robot_pose;
  explore_costmap_ros_->getRobotPose(robot_pose);
  PoseStamped robot_pose_msg;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);

  home_pose_msg.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  home_pose_msg.pose.position = robot_pose_msg.pose.position;
  home_pose_msg.pose.orientation = robot_pose_msg.pose.orientation;
  ROS_WARN("Robot home at %f, %f, %f",
    home_pose_msg.pose.position.x,
    home_pose_msg.pose.position.y,
    home_pose_msg.pose.position.z);

  // Last time we were at home is right now
  last_time_at_home = ros::Time::now();

  time_to_home = 0;
  last_time_update_time_to_home = ros::Time::now();

  battery_duration = ros::Duration(BATTERY_TIME);
  initial_time_away_from_home = ros::Duration(INITIAL_EXPLORE_TIME);

  // Find robot's max speed
  max_vel_x = 0.0;
  private_nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", max_vel_x);
  ROS_WARN("Robot has max speed %f", max_vel_x);
  assert(max_vel_x != 0.0);

  // And max turn speed
  max_vel_th = 0.0;
  private_nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_th", max_vel_th);
  ROS_WARN("Robot has max turn speed %f", max_vel_th);
  assert(max_vel_th != 0.0);

  battery_safety_margin = START_SAFETY_MARGIN;

  last_pose = currentPoseStamped();

#ifdef BATTERY_TIMER
  global_state = GLOBAL_STATE_INITIAL;
#else
  global_state = GLOBAL_STATE_EXPLORE;
#endif
  state = STATE_WAITING_FOR_GOAL;

  skeleplanner_ = new SkelePlanner();
  skeleplanner_->initialize(std::string("skeleplanner"), explore_costmap_ros_);
}

Explore::~Explore() {
  if(loop_closure_ != NULL)
    delete loop_closure_;

  if(planner_ != NULL)
    delete planner_;

  if(explorer_ != NULL)
    delete explorer_;

  if(explore_costmap_ros_ != NULL)
    delete explore_costmap_ros_;
}

/*
	Similar/same to above function. Sends map data (whether occupied, etc)
*/
void Explore::publishMap() {
  nav_msgs::OccupancyGrid map;
  map.header.stamp = ros::Time::now();

  Costmap2D explore_costmap;
  explore_costmap_ros_->getCostmapCopy(explore_costmap);

  map.info.width = explore_costmap.getSizeInCellsX();
  map.info.height = explore_costmap.getSizeInCellsY();
  map.info.resolution = explore_costmap.getResolution();
  map.info.origin.position.x = explore_costmap.getOriginX();
  map.info.origin.position.y = explore_costmap.getOriginY();
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0;
  map.info.origin.orientation.y = 0;
  map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;

  int size = map.info.width * map.info.height;
  const unsigned char* char_map = explore_costmap.getCharMap();

  //map.set_data_size(size);
  // XXX May be wrong. Previously was the above
  map.data.resize(size);
  for (int i=0; i<size; i++) {
    if (char_map[i] == NO_INFORMATION)
      map.data[i] = -1;
    else if (char_map[i] == LETHAL_OBSTACLE)
      map.data[i] = 100;
    else
      map.data[i] = 0;
  }

  map_publisher_.publish(map);
}

/*
	For visualization
*/
void Explore::publishGoal(const geometry_msgs::Pose& goal){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "explore_goal";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.pose = goal;
  marker.scale.x = 0.5;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(5);
  marker_publisher_.publish(marker);
}

/*
  We are given a list of frontiers (goals) sorted by their cost (according to costmap?)
  with the cheapest first.

  Examine these based on the criteria that we need to both get to the goal, and afterwards
  get home. Remove those which we deem impossible to reach.

  For example, if we can reach a goal, but not reach a goal and then reach home, we do
  not want to go to that goal. Instead, choose a goal with which this is possible.

  Note: This does not take into account the case where we may still want to go to a goal
  even if we cannot get all the way there, such as when we may want to go towards that goal
  so that we can get range data.

  Precondition:
    - Must have called computePotentialFromRobot() first
      (this is currently done prior to calling makePlan() which calls this function)
*/
void Explore::removeUnsafeFrontiers(std::vector<geometry_msgs::Pose> * goals) {
  // Indices in this vector correspond to indices in the goals vector
  std::vector<int> goal_time_costs;
  goal_time_costs.resize( goals->size() );

#ifdef DEBUG
  for (std::vector<int>::iterator it = goal_time_costs.begin(); it != goal_time_costs.end(); it++) {
    assert( *it == 0 );
  }
#endif

  geometry_msgs::PoseStamped current_pose_stamped = currentPoseStamped();

  // Assuming we have called computePotentialFromRobot()
  // First we find time cost from robot's current position to each goal
  geometry_msgs::Pose pose;
  for (unsigned int i = 0; i < goals->size(); i++) {
    // XXX Probably need to not copy this
    pose = goals->at(i);
    //goal_time_costs[i] = timeToTravel( &current_pose_stamped, &(goals->at(i)) );
    goal_time_costs[i] = timeToTravel( &current_pose_stamped, &pose );
  }

  ROS_WARN("Finished calculating time to each goal from current position.");

  // Now find time cost from home to each goal
  // (We assume this is the same path as that from the goal to home)
  explorer_->computePotentialFromPoint(explore_costmap_ros_, planner_, & home_pose_msg.pose.position);
  for (unsigned int i = 0; i < goals->size(); i++) {
    // XXX Probably need to not copy this
    pose = goals->at(i);
    //goal_time_costs[i] += timeToTravel( &home_pose_msg, &(goals->at(i)) );
    goal_time_costs[i] += timeToTravel( &home_pose_msg, &pose );
  }

  ROS_WARN("Finished calculating time to each goal from home position.");

  // These two costs together yield the total cost from our current position to the goal,
  // and from the goal to home
  std::vector<geometry_msgs::Pose>::iterator it = goals->begin();
  std::vector<int>::iterator it2 = goal_time_costs.begin();
  while (it != goals->end() && it2 != goal_time_costs.end()) {
    if (*it2 + battery_safety_margin >= batteryTimeRemaining()) {
      it = goals->erase(it);
      it2 = goal_time_costs.erase(it2);
      ROS_WARN("Removed a frontier goal since we can't reach it right now.");
    } else {
      it++;
      it2++;
    }
  }

  ROS_WARN("%d frontier goals remaining after removing unsafe ones.", goals->size());

  // Recompute potential from robot as we need it later
  explorer_->computePotentialFromRobot(explore_costmap_ros_, planner_);
}

/*
	Get goals from explore_frontier & choose one to go to
	If goal has changed not changed from previous, ensure makes progress
	If sufficient time elapsed and could not reach, blacklist
	Sends the goal pose to the base with a callback function to call when goal reached
*/
void Explore::makePlan() {
  // since this gets called on handle activate
  if(explore_costmap_ros_ == NULL)
    return;

  // Calculate potential from robot to each point on map
  explorer_->computePotentialFromRobot(explore_costmap_ros_, planner_);

  // Since we are updating cost to points on map anyway, also update
  // our time to go home.
  updateTimeToHome();

  tf::Stamped<tf::Pose> robot_pose;
  explore_costmap_ros_->getRobotPose(robot_pose);

  std::vector<geometry_msgs::Pose> goals;

  // Find frontier goals
  if (! explorer_->getExplorationGoals(*explore_costmap_ros_, robot_pose, planner_, goals, potential_scale_, orientation_scale_, gain_scale_) ) {
    ROS_WARN("No frontiers found?");
  }

  // Before filtering goals, we check if exploration is done
  if (goals.size() == 0) {
    done_exploring_ = true;
    goHome();
    return;
  }

#ifdef FRONTIER_COMPARE
  removeUnsafeFrontiers(&goals);

  // Go home if there are no reachable goals at the moment
  if (goals.size() == 0) {
    goHome();
    return;
  }
#endif

  bool valid_plan = false;
  std::vector<geometry_msgs::PoseStamped> plan;

  PoseStamped goal_pose;
  goal_pose.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  goal_pose.header.stamp = ros::Time::now();

  int blacklist_count = 0;
  for (unsigned int i = 0; i < goals.size(); i++) {
    goal_pose.pose = goals[i];
    if (goalOnBlacklist(goal_pose)) {
      blacklist_count++;
      continue;
    }

    // Build plan to given goal. If such a plan exists, we can use it
    valid_plan = (planner_->getPlanFromPotential(goal_pose, plan) && !plan.empty());
    if (valid_plan) {
      break;
    }
  }

  if (valid_plan) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;
    current_goal_pose_stamped_ = goal_pose;
    move_base_client_.sendGoal(goal, boost::bind(&Explore::reachedGoal, this, _1, _2, goal_pose));
    ROS_WARN("Sent new goal to move_base.");

    setLocalState(STATE_EXPLORING);
    time_since_progress_ = 0.0;

    if (visualize_) {
      publishGoal(goal_pose.pose);
    }
  } else {
    ROS_WARN("Done exploring with %d goals left that could not be reached. There are %d goals on our blacklist, and %d of the frontier goals are too close to them to pursue. The rest had global planning fail to them. \n", (int)goals.size(), (int)frontier_blacklist_.size(), blacklist_count);
    ROS_INFO("Exploration finished. Hooray.");
    done_exploring_ = true;
    goHome();
  }
}

/*
  Check if current goal is too close to a blacklisted frontier
*/
bool Explore::goalOnBlacklist(const geometry_msgs::PoseStamped& goal){
  //check if a goal is on the blacklist for goals that we're pursuing
  for (unsigned int i = 0; i < frontier_blacklist_.size(); ++i) {
    double x_diff = fabs(goal.pose.position.x - frontier_blacklist_[i].pose.position.x);
    double y_diff = fabs(goal.pose.position.y - frontier_blacklist_[i].pose.position.y);

    if(x_diff < 2 * explore_costmap_ros_->getResolution() &&
      y_diff < 2 * explore_costmap_ros_->getResolution())
    {
      return true;
    }
  }
  return false;
}

/*
  Callback called when goal reached (from nav stack)
*/
void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
    const move_base_msgs::MoveBaseResultConstPtr& result,
    geometry_msgs::PoseStamped frontier_goal)
{
  ROS_WARN("Reached goal.");
  setLocalState(STATE_WAITING_FOR_GOAL);

  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_WARN("Adding current goal to black list (aborted, but reached goal).");
  }
}

/*
  Only accurate if computePotential() recently called.
*/
int Explore::timeToHome() {
  PoseStamped current_pose = currentPoseStamped();
  int time_to_home = timeToTravel(&current_pose, &home_pose_msg.pose);
  return time_to_home;
}

/*
  Assume computePotential() has been recently/just called.
  Update our time_to_home, and update when this was last calculated.
*/
void Explore::updateTimeToHome() {
  time_to_home = timeToHome();
  last_time_update_time_to_home = ros::Time::now();
}

/*
  Get the robot's current pose
*/
geometry_msgs::PoseStamped Explore::currentPoseStamped() {
  tf::Stamped<tf::Pose> robot_pose;
  explore_costmap_ros_->getRobotPose(robot_pose);

  PoseStamped robot_pose_stamped;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_stamped);

  //ROS_WARN("Robot pose currently %f, %f", robot_pose_msg.pose.position.x, robot_pose_msg.pose.position.y);

  return robot_pose_stamped;
}

/*
  Find the distance in meters for given plan starting from given pose
*/
double Explore::distanceForPlan(PoseStamped * pose, std::vector<geometry_msgs::PoseStamped> * plan) {
  double distance = 0.0;
  PoseStamped previous_pose = *pose;

  for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan->begin(); it != plan->end(); it++) {
    double dx = previous_pose.pose.position.x - it->pose.position.x;
    double dy = previous_pose.pose.position.y - it->pose.position.y;
    distance += sqrt(dx*dx + dy*dy);
    previous_pose = *it;
  }

  return distance;
}

/*
  Find the amount of angle change needed for given plan starting from given pose
*/
double Explore::angleChangeForPlan(PoseStamped * pose, std::vector<geometry_msgs::PoseStamped> * plan) {
  double angle_change = 0.0;
  PoseStamped previous_pose = *pose;
  double previous_angle = tf::getYaw(previous_pose.pose.orientation);
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan->begin(); it != plan->end(); it++) {
    double dx = previous_pose.pose.position.x - it->pose.position.x;
    double dy = previous_pose.pose.position.y - it->pose.position.y;
    double da = atan2(dx, dy);

    angle_change += fabs(previous_angle - da);

    //ROS_WARN("dx %f dy %f da %f previous_angle %f angle_change %f", dx, dy, da, previous_angle, angle_change);

    previous_angle = da;
    previous_pose = *it;
  }

  return angle_change;
}

/*
  True if our current pose is "close enough" to given pose
*/
bool Explore::closeEnoughToPoseStamped(geometry_msgs::PoseStamped * pose_stamped) {
  PoseStamped current_pose = currentPoseStamped();

  double x_diff = fabs(current_pose.pose.position.x - pose_stamped->pose.position.x);
  double y_diff = fabs(current_pose.pose.position.y - pose_stamped->pose.position.y);

  return x_diff < 2 * explore_costmap_ros_->getResolution() &&
    y_diff < 2 * explore_costmap_ros_->getResolution();
}

/*
  Approximate 2d distance between the two poses
*/
double Explore::distanceBetweenTwoPoses(geometry_msgs::Pose * pose1, geometry_msgs::Pose * pose2) {
  double dx = pose1->position.x - pose2->position.x;
  double dy = pose1->position.y - pose2->position.y;
  return sqrt(dx*dx + dy*dy);
}

/*
  If we're close enough to home, assume we're there
  (so as not to care so much about angle once we're at home, etc)
*/
bool Explore::atHome() {
  return closeEnoughToPoseStamped(&home_pose_msg);
}

/*
  Are we close enough to our goal?
*/
bool Explore::atGoal() {
  return closeEnoughToPoseStamped(&current_goal_pose_stamped_);
}

/*
  Check if we appear to be stuck
  If we're stuck and trying to explore, blacklist the goal
  Otherwise we are going home, so try to move in a random direction
*/
void Explore::checkIfStuck() {
  PoseStamped current_pose = currentPoseStamped();

  double dist = distanceBetweenTwoPoses(&last_pose.pose, &current_pose.pose);
  if (dist < 0.10) {
    time_since_progress_ += 1.0f / planner_frequency_;
  } else {
    time_since_progress_ = 0.0;
  }
  ROS_WARN("Time since progress: %f (in global state %d, local state %d)", time_since_progress_,
    global_state, state);

  if (time_since_progress_ > progress_timeout_) {
    ROS_WARN("Decided we're stuck.");

    if (state == STATE_EXPLORING) {
      frontier_blacklist_.push_back(current_goal_pose_stamped_);
      ROS_WARN("Adding current goal to black list.");
      setLocalState(STATE_WAITING_FOR_GOAL);
    } else if (state == STATE_HEADING_HOME) {
      moveRandomDirection();
    } else {
      ROS_WARN("*** We're stuck but didn't expect this state! ***");
      assert(9 == 10);
    }
  }
}

/*
  Taken from SendingSimpleGoals tutorial
*/
void Explore::moveRandomDirection() {
  move_base_msgs::MoveBaseGoal goal;
  // in base link frame so that coords are distance to move?
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  // 1 meter
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = (double) rand() / (double) RAND_MAX;
  ROS_WARN("**** Moving random direction %f ****", goal.target_pose.pose.orientation.w);

  move_base_client_.sendGoal(goal);
  publishGoal(goal.target_pose.pose); 

  move_base_client_.waitForResult();

  if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    time_since_progress_ = 0.0;
  }
}

/*
  Estimated time in seconds to reach target_pose from source_pose

  Note: computePotentialFromPoint() must be called with source_pose
  prior to calling this!
*/
int Explore::timeToTravel(geometry_msgs::PoseStamped* source_pose_stamped, geometry_msgs::Pose* target_pose) {
  // Find the plan to go to target_pose
  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  target_pose_stamped.header.stamp = ros::Time::now();
  target_pose_stamped.pose = *target_pose;
  bool valid_plan = planner_->getPlanFromPotential(target_pose_stamped, plan) && !plan.empty();
  if (!valid_plan) {
    return -1;
  }

  // Calculate the distance from source point to target point using the plan
  double distance = distanceForPlan(source_pose_stamped, &plan);
  ROS_WARN("Distance to carry out plan: %f meters", distance);

  // Find the amount of angle changed needed for the plan
  double angle_change = angleChangeForPlan(source_pose_stamped, &plan);
  ROS_WARN("Angle change to carry out the plan: %f", angle_change);

  // Estimate time to travel using the plan
  int time = ceil(distance / max_vel_x) + ceil(angle_change / max_vel_th);
  ROS_WARN("Estimated time to carry out the plan: %d seconds", time);

  return time;
}

/*
  Decide whether we should go home based on robot's current position
  relative to home. Since this uses costmap data and plan generated from
  that, we must have called computePotential() first, making this expensive
  to be calling frequently.

  Note: Must have called computePotentialFromRobot() prior to calling this
*/
bool Explore::shouldGoHome_dynamic() {
  ROS_WARN("Cost to go home: %f", planner_->getPointPotential( home_pose_msg.pose.position ) );

  int time_to_home = timeToHome();
  if (time_to_home == -1) {
    ROS_WARN("No plan to get home.");
    assert(1 == 0);
  }
  ROS_WARN("Estimated time to reach home: %d seconds", time_to_home);

  ROS_WARN("Battery safety margin: %d seconds", battery_safety_margin);

  int battery_time_remaining = batteryTimeRemaining();
  ROS_WARN("Battery time remaining: %d seconds", battery_time_remaining);

  if (time_to_home + battery_safety_margin > battery_time_remaining) {
    ROS_WARN("Decided to return home.");
    return true;
  }

  return false;
}

/*
  Fast decision whether to go home or not.

  Assume entire time since last charge was spent moving away from home.
  We then need this same amount of time remaining on battery to return
  home.
*/
bool Explore::shouldGoHome_fast() {
  int time_since_charge = timeSinceCharge();
  int battery_time_remaining = batteryTimeRemaining();

  ROS_WARN("Time elapsed since charge: %d Time remaining on battery: %d",
    time_since_charge, battery_time_remaining);

  ros::Duration time_since_time_to_home_updated = ros::Time::now() - last_time_update_time_to_home;

  ROS_WARN("Time to home: %d Time since updated time to home: %d",
    time_to_home, time_since_time_to_home_updated.sec);

  ROS_WARN("Current max battery duration: %d", battery_duration.sec);

  // If our battery time estimate says to go home, do so
  if (time_to_home + time_since_time_to_home_updated.sec
    + battery_safety_margin > battery_time_remaining)
  {
    ROS_WARN("Decided to return home. (timer)");
    return true;
  }

  // Our battery time estimate may be off. Check our critical voltage too
  if (atCriticalVoltage()) {
    ROS_WARN("Decided to return home. (critical voltage)");

    // We hit critical voltage again. We can update our battery duration.
    // Find the duration of the previous cycle (time since last charge)
    ros::Duration previous_duration = ros::Time::now() - last_time_charged;
    if (previous_duration.sec > battery_duration.sec) {
      battery_duration.sec++;
      ROS_WARN("Increasing battery duration.");

    // Arbitrary requirement of at least 5 seconds duration, since we may read
    // old critical voltage readings which are invalid and cause us to reset
    // the time to 0 or so
    } else if (previous_duration.sec > 5) {
      battery_duration = previous_duration;
      ROS_WARN("Reset battery duration to current cycle's duration.");
    }

    return true;
  }

  return false;
}

/*
  In global state Initial we want to return home periodically.
  Check if we've been away from home for our set period.
*/
bool Explore::shouldGoHome_initial() {
  ros::Duration elapsed = ros::Time::now() - last_time_at_home;
  return elapsed > initial_time_away_from_home;
}

/*
  Return the time since charge in seconds
*/
int Explore::timeSinceCharge() {
  ros::Duration elapsed = ros::Time::now() - last_time_at_home;
  return elapsed.sec;
}

/*
  Estimation of amount of battery time we have left in seconds
*/
int Explore::batteryTimeRemaining() {
  int remaining = battery_duration.sec - timeSinceCharge();
  return remaining;
}

/*
  Send goal to go home to base
*/
void Explore::goHome() {
  // XXX Maybe we should call computePotential() and make a plan to go home
  // rather than just sending a goal.
  // Also, maybe we should be doing that periodically as we go home to ensure
  // we actually have a valid plan. Or is this necessary?

  setLocalState(STATE_HEADING_HOME);

  home_pose_msg.header.stamp = ros::Time::now();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = home_pose_msg;
  move_base_client_.sendGoal(goal, boost::bind(&Explore::reachedHomeCallback, this, _1, _2, home_pose_msg));
}

/*
  We reached home
*/
void Explore::reachedHome() {
  int battery_time_remaining = batteryTimeRemaining();

  if (battery_time_remaining <= 0) {
    ROS_WARN("I died!");
  }

  // Change next margin depending on time remaining on our battery
  /*
  if ( battery_time_remaining > MIN_BATTERY_SAFETY_MARGIN ) {
    battery_safety_margin--;
  } else if ( battery_time_remaining < MIN_BATTERY_SAFETY_MARGIN ) {
    //battery_safety_margin++;
    battery_safety_margin += MIN_BATTERY_SAFETY_MARGIN;
  }
  */

  last_time_at_home = ros::Time::now();
}

/*
  This callback called when home has been reached (by nav stack)
*/
void Explore::reachedHomeCallback(const actionlib::SimpleClientGoalState& status,
  const move_base_msgs::MoveBaseResultConstPtr& result,
  geometry_msgs::PoseStamped goal) {
  reachedHome();
}

/*
  Wait until we read an initial voltage
*/
void Explore::waitForInitialVoltage() {
  ros::Rate r(10.0);
  while ( node_.ok() && battery_voltage == -1.0 ) {
    r.sleep();
  }
  voltage_initial = battery_voltage;
  assert(voltage_initial != -1.0);
  ROS_INFO("Recorded initial voltage %f", voltage_initial);
  last_time_charged = ros::Time::now();
}

/*
  If we hear a voltage at a predefined level, this is our warning
  of low/empty battery.
*/
bool Explore::atCriticalVoltage() {
#ifdef DEBUG
  if (battery_voltage <= VOLTAGE_WARNING) {
    ROS_WARN("** At critical voltage. **");
  }
#endif
  return battery_voltage <= VOLTAGE_WARNING;
}

/*
  We leave the initial global state once we have heard the battery
  warning
*/
void Explore::updateGlobalState() {
#ifdef CONSTANT_BATTERY_TIME
  // We wish to remain in initial state forever if we are using a
  // constant periodic return to home. So never change state.
  return;
#endif

  if ( atCriticalVoltage() ) {
    setGlobalState(GLOBAL_STATE_EXPLORE);
    goHome();
    // Initially battery duration is the time between wake up ("last charged")
    // and run out of battery, which just occurred.
    battery_duration = ros::Time::now() - last_time_charged;
  }
}

void Explore::setGlobalState(int new_state) {
#ifdef DEBUG
  std::string s;
  switch (new_state) {
    case GLOBAL_STATE_INITIAL:
      s = "INITIAL";
      break;
    case GLOBAL_STATE_EXPLORE:
      s = "EXPLORE";
      break;
    default:
      s = "UNKNOWN";
  }

  ROS_WARN("Setting global state to: %s", s.c_str());
#endif

  global_state = new_state;
}

void Explore::setLocalState(int new_state) {
#ifdef DEBUG
  std::string s;
  switch (new_state) {
    case STATE_HEADING_HOME:
      s = "HEADING_HOME"; 
      break;
    case STATE_EXPLORING:
      s = "EXPLORING";
      break;
    case STATE_WAITING_FOR_GOAL:
      s = "WAITING_FOR_GOAL";
      break;
    case STATE_CHARGING:
      s = "CHARGING";
      break;
    case STATE_DONE:
      s = "DONE";
      break;
    default:
      s = "UNKNOWN";
  }
  ROS_WARN("Setting local state to: %s", s.c_str());
#endif

  state = new_state;
}

/*
	Continually run makePlan() after specified sleep
	This will cause goals to become blacklisted if not enough progress is made
	and not just wait for callback when goal is reached
*/
void Explore::execute() {
  while (! move_base_client_.waitForServer(ros::Duration(5,0)))
    ROS_WARN("Explore: Waiting to connect to move_base server");
  ROS_INFO("Explore: Connected to move_base server.");

#ifdef BATTERY_TIMER
  // Wait until we get an initial voltage reading
  waitForInitialVoltage();
#endif

  ros::Rate r(planner_frequency_);

  while (node_.ok()) {

    if (close_loops_) {
      tf::Stamped<tf::Pose> robot_pose;
      explore_costmap_ros_->getRobotPose(robot_pose);
      loop_closure_->updateGraph(robot_pose);
    }

#ifdef BATTERY_TIMER
    // Initial behaviour
    if (global_state == GLOBAL_STATE_INITIAL) {
      // If we're heading home, we may be there now
      if ( state == STATE_HEADING_HOME && atHome() ) {
        reachedHome();
        setLocalState(STATE_WAITING_FOR_GOAL);

      // Should we go home?
      } else if ( state != STATE_HEADING_HOME && shouldGoHome_initial() ) {
        goHome();

      // We need a new exploration goal
      } else if ( state == STATE_WAITING_FOR_GOAL || atGoal() ) {
        makePlan();

      } else {
        checkIfStuck();
      }

      // We may leave the initial state
      updateGlobalState();

    // Exploring
    } else if (global_state == GLOBAL_STATE_EXPLORE) {
      // If we're heading home, we may be there now
      if ( state == STATE_HEADING_HOME && atHome() ) {
        reachedHome();
        setLocalState(STATE_CHARGING);

      // Should we go home?
      } else if ( state != STATE_HEADING_HOME && state != STATE_CHARGING && shouldGoHome_fast() ) {
        goHome();

      // an else if (continued below)
      } else
#endif

      // We need a new exploration goal
      if ( state == STATE_WAITING_FOR_GOAL || atGoal() ) {
        makePlan();

      // We can get stuck (if we're not charging), so handle it.
      } else if ( state != STATE_CHARGING ) {
        checkIfStuck();
      }

#ifdef BATTERY_TIMER
    } else {
      ROS_WARN("Unknown global state.");
      assert(2 == 3);
    }
#endif

    if (visualize_) {
      // publish visualization markers
      std::vector<Marker> markers;
      explorer_->getVisualizationMarkers(markers);
      for (uint i=0; i < markers.size(); i++)
        marker_publisher_.publish(markers[i]);

      // and occupancy map
      publishMap();

      // and topomap
      // first generate a topomap (this happens with makePlan, though we don't want a plan...)
      geometry_msgs::PoseStamped current_pose_stamped = currentPoseStamped();
      std::vector<geometry_msgs::PoseStamped> plan_empty;
      skeleplanner_->makePlan( current_pose_stamped, current_pose_stamped, plan_empty );
      // then publish it
      skeleplanner_->publish_topomap(&topomap_marker_publisher_);
    }

    last_pose = currentPoseStamped();
    r.sleep();
  }

  move_base_client_.cancelAllGoals();
}

void Explore::spin() {
  ros::spinOnce();
  boost::thread t(boost::bind( &Explore::execute, this ));
  ros::spin();
  t.join();
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "explore");

  explore::Explore explore;
  explore.spin();

  return(0);
}

