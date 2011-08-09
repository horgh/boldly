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

// When we're in simulation we want some things to be faster, such as
// PROGRESS_TIMEOUT so uncomment this to do so
//#define SIMULATION

// Compile with some debugging
#define DEBUG

// Make a new topomap every time we want to update the topomap
// rather than rely on updating an existing one
// (May choose to do this to avoid crashing when map size changes)
//#define REMAKE_TOPOMAP

// If uncommented, use frontier comparison algorithm where we don't try to go to those
// frontiers which we deem unsafe due to battery life, but may go to others intead
// XXX Probably not working correctly now.
//#define FRONTIER_COMPARE

// Time until we decide we are stuck in seconds
#ifdef SIMULATION
#define PROGRESS_TIMEOUT 30.0
#else
#define PROGRESS_TIMEOUT 30.0
#endif

// Enable this to periodically make a new plan even if we're following one
#define REEVALUATE_PLANS
// Time until we make a new plan, even if we're currently following one
#define REEVALUATE_PLANS_TIME 10.0

// Number of times to go out exploring before we decide to go far
#define EXPLORATION_RUNS 400

// Our battery is a timer. Makes us go home when we judge we need to.
// With CONSTANT_BATTERY_TIME, always use BATTERY_TIME as our battery life.
// Otherwise we change battery time to duration until heard warning voltage.
#define BATTERY_TIMER

// Always use BATTERY_TIME as battery duration rather than voltage logic.
// Requires BATTERY_TIMER as well.
#define CONSTANT_BATTERY_TIME

// Life of battery in seconds
#ifdef SIMULATION
#define BATTERY_TIME 30
#else
#define BATTERY_TIME 1200
#endif
// Start heading back with at least this margin of safety (wrt battery time remaining)
#define MIN_BATTERY_SAFETY_MARGIN 10
// Starting safety margin
#define START_SAFETY_MARGIN 0

// Voltage that we consider too low and must charge. This should
// be the same as robot starts sounding alarums
#define VOLTAGE_WARNING 11.5

/*
  Possible states
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
    setState(STATE_WAITING_FOR_GOAL);

    last_time_charged = ros::Time::now();
  }
}

void Explore::battery_state_callback(const p2os_driver::BatteryState::ConstPtr & msg) {
  //ROS_INFO("Got battery state (voltage)");
  battery_voltage = msg->voltage;
}

/*
  Add an arrow marker to the given markers vector
*/
void Explore::visualize_arrow(int id, double x, double y, double scale, double r,
  double g, double b, double a, std::vector<visualization_msgs::Marker>* markers,
  std::string ns)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  //m.type = visualization_msgs::Marker::ARROW;
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = 0.0;
  // 1.0 for frontier arrow
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;

  m.lifetime = ros::Duration(5.0);

  markers->push_back( m );
}

/*
  Publish markers for each frontier_blacklist_ on topomap_marker publisher
*/
void Explore::visualize_blacklisted() {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "blacklisted";
  m.lifetime = ros::Duration(5.0);

  // Pink?
  m.color.r = 255.0;
  m.color.g = 0.0;
  m.color.b = 169.0;
  m.color.a = 0.5;

  // frontiers are 0.7 cubes
  double scale = 0.6;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;

  m.type = visualization_msgs::Marker::SPHERE;
  
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = frontier_blacklist_.begin();
    it < frontier_blacklist_.end();
    ++it)
  {
    m.pose = it->pose;
    m.id = topomap_publisher_marker_id;
    topomap_publisher_marker_id++;
    topomap_marker_publisher_.publish( m );
  }
}

/*
  Visualize a plan using the topomap_marker_publisher
*/
void Explore::visualize_plan(std::vector<geometry_msgs::PoseStamped>& plan) {
  std::vector<visualization_msgs::Marker> markers;

  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
    it < plan.end();
    ++it)
  {
    visualize_arrow(topomap_publisher_marker_id, it->pose.position.x, it->pose.position.y,
      0.4, // scale
      205.0, 173.0, 0.0, 0.5, // goldish
      &markers, "plan");
    topomap_publisher_marker_id++;
  }

  for (std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin();
    it < markers.end();
    ++it)
  {
    topomap_marker_publisher_.publish( *it );
  }
}

double Explore::distance_between_coords(double x1, double y1, double x2, double y2) {
  return std::sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}

/*
  Look at our current costmap. Find the longest straight line point from home
  that we can reach. Mark the point, and draw the plan.
*/
void Explore::find_furthest_point() {
  costmap_2d::Costmap2D costmap;
  explore_costmap_ros_->getCostmapCopy(costmap);

  unsigned int w = costmap.getSizeInCellsX();
  unsigned int h = costmap.getSizeInCellsY();
  unsigned int size = w * h;

  const unsigned char* map = costmap.getCharMap();

  geometry_msgs::PoseStamped goal_pose_stamped;
  goal_pose_stamped.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  goal_pose_stamped.header.stamp = ros::Time::now();
  goal_pose_stamped.pose.position.z = 0.0;
  goal_pose_stamped.pose.orientation.x = 0.0;
  goal_pose_stamped.pose.orientation.y = 0.0;
  goal_pose_stamped.pose.orientation.z = 0.0;
  goal_pose_stamped.pose.orientation.w = 0.0;

  std::vector<geometry_msgs::PoseStamped> plan;

  double furthest_distance = 0.0;
  double furthest_x, furthest_y;

  // Want to be able to find plans from home
  explorer_->computePotentialFromPoint(explore_costmap_ros_, planner_, &home_pose_msg.pose.position);

  for (unsigned int i = 0; i < size; ++i) {
    // Point must be in free space
    if (map[i] != costmap_2d::FREE_SPACE)
      continue;

    // From index to cell coords
    unsigned int cell_x, cell_y;
    costmap.indexToCells(i, cell_x, cell_y);

    // From cell coords to map coords
    double x, y;
    costmap.mapToWorld(cell_x, cell_y, x, y);

    // We want furthest distance
    double distance = distance_between_coords(home_pose_msg.pose.position.x, home_pose_msg.pose.position.y,
      x, y);
    if (distance <= furthest_distance)
      continue;

    // See if we can make a plan to the new furthest point
    plan.clear();
    goal_pose_stamped.pose.position.x = x;
    goal_pose_stamped.pose.position.y = y;
    // Seem to have a valid plan
    if ( planner_->getPlanFromPotential(goal_pose_stamped, plan) && !plan.empty() ) {
      // XXX May need to ensure that the plan does not go through unknown cells

      furthest_distance = distance;
      furthest_x = x;
      furthest_y = y;
    }
  }

  // Visualise the plan & furthest pt
  int id = 0;
  std::vector<visualization_msgs::Marker> markers;
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
    it < plan.end();
    ++it, ++id)
  {
    visualize_arrow(topomap_publisher_marker_id, it->pose.position.x, it->pose.position.y,
      0.2, // scale
      0.0, 245.0, 255.0, 0.5, // turquoise
      &markers, "furthest");
    topomap_publisher_marker_id++;
  }

  // Publish the markers
  for (std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin();
    it < markers.end();
    ++it)
  {
    topomap_marker_publisher_.publish( *it );
  }

  // Leave planner in expected state
  explorer_->computePotentialFromRobot(explore_costmap_ros_, planner_);
}

Explore::Explore() :
  node_(),
  tf_(ros::Duration(10.0)),
  explore_costmap_ros_(NULL),
  move_base_client_("move_base"),
  planner_(NULL),
  explorer_(NULL),
  prev_plan_size_(0)
{
  ros::NodeHandle private_nh("~");

  marker_publisher_ = node_.advertise<Marker>("visualization_marker",10);
  marker_array_publisher_ = node_.advertise<MarkerArray>("visualization_marker_array",10);
  topomap_marker_publisher_ = node_.advertise<Marker>("topomap_marker", 3000);
  map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  voltage_subscriber_ = node_.subscribe<p2os_driver::BatteryState>("battery_state", 1, &Explore::battery_state_callback, this);
  charged_subscriber_ = node_.subscribe<std_msgs::Empty>("charge_complete", 1, &Explore::charge_complete_callback, this);

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

  // Setup robot's home posestamped which doesn't change (other than header)
  home_pose_msg.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  home_pose_msg.pose.position = robot_pose_msg.pose.position;
  home_pose_msg.pose.orientation = robot_pose_msg.pose.orientation;
  ROS_WARN("Robot home at %f, %f, %f",
    home_pose_msg.pose.position.x,
    home_pose_msg.pose.position.y,
    home_pose_msg.pose.position.z);

  // Assume we start already charged
  last_time_charged = ros::Time::now();

  // Start with a specified battery time
  battery_duration = ros::Duration(BATTERY_TIME);

  battery_safety_margin = START_SAFETY_MARGIN;

  last_pose = currentPoseStamped();

  battery_voltage = 100.0;

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

  topomap_ = new Topomap(explore_costmap_ros_, home_pose_msg.pose.position.x,
    home_pose_msg.pose.position.y);

  // Haven't yet done any exploration runs
  exploration_runs_ = 0;

  notifier_ = new Notifier(&node_);

  setState(STATE_WAITING_FOR_GOAL);
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

  if (topomap_ != NULL)
    delete topomap_;
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
  // (Only needed for shouldGoHome_fast()
  //updateTimeToHome();

  tf::Stamped<tf::Pose> robot_pose;
  explore_costmap_ros_->getRobotPose(robot_pose);

  std::vector<geometry_msgs::Pose> goals;

  /*
    Find frontier goals
  */

  // getExplorationGoals() is old frontier rating system
  //if (! explorer_->getExplorationGoals(*explore_costmap_ros_, robot_pose, planner_, goals, potential_scale_, orientation_scale_, gain_scale_) ) {

  if (! explorer_->rateFrontiers(*explore_costmap_ros_, robot_pose, planner_, goals, potential_scale_, orientation_scale_, gain_scale_, topomap_->get_topomap() )) {
    ROS_WARN("No frontiers found?");
  }

  // Before filtering goals, we check if exploration is done
  if (goals.size() == 0) {
    // XXX With this we may goHome() prematurely (such as at the very first
    // call, for example in case where rateFrontiers() wants a topomap but
    // we haven't yet generated one.)
    //goHome();
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

    current_plan_ = plan;

    last_goal_chosen = ros::Time::now();

    setState(STATE_EXPLORING);
    //time_since_progress_ = 0.0;
    time_following_plan_ = 0.0;

    if (visualize_) {
      publishGoal(goal_pose.pose);
    }
  } else {
    ROS_WARN("Done exploring with %d goals left that could not be reached. There are %d goals on our blacklist, and %d of the frontier goals are too close to them to pursue. The rest had global planning fail to them. \n", (int)goals.size(), (int)frontier_blacklist_.size(), blacklist_count);
    ROS_INFO("Exploration finished. Hooray.");
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
  setState(STATE_WAITING_FOR_GOAL);

  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_WARN("Adding current goal to black list (aborted, but reached goal).");
  }
}

/*
  Only accurate if computePotential() recently called.
*/
int Explore::secsToHome() {
  PoseStamped current_pose = currentPoseStamped();
  int secs_to_home = timeToTravel(&current_pose, &home_pose_msg.pose);
  return secs_to_home;
}

/*
  Assume computePotential() has been recently/just called.
  Update our secs_to_go_home, and update when this was last calculated.
*/
/*
 * Used with shouldGoHome_fast()

void Explore::updateTimeToHome() {
  secs_to_go_home = secsToHome();
  last_time_update_secs_to_go_home = ros::Time::now();
}
*/

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
  ROS_WARN("Time since progress: %f (in state %d)", time_since_progress_, state);

  if (time_since_progress_ > progress_timeout_) {
    ROS_WARN("Decided we're stuck.");

    if (state == STATE_EXPLORING) {
      frontier_blacklist_.push_back(current_goal_pose_stamped_);
      ROS_WARN("Adding current goal to black list.");
      notifier_->publish("Adding current goal to black list.");
      setState(STATE_WAITING_FOR_GOAL);
      time_since_progress_ = 0.0;
    } else if (state == STATE_HEADING_HOME) {
      notifier_->publish("Stuck while heading home.");
      // Try to move in a random direction to get unstuck
      moveRandomDirection();
      // The above sent a new goal to move_base. When it's done, we need to
      // return to heading home.
      goHome();
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

  Note: Must have called computePotentialFromRobot() prior to calling this if
  we want accurate time to go home.
*/
bool Explore::shouldGoHome_dynamic() {
// If we're not in using CONSTANT_BATTERY_TIME, listen to voltage as
// an indicator of whether to go home & update battery duration based on this
#ifndef CONSTANT_BATTERY_TIME
  if (atCriticalVoltage()) {
    updateBatteryDuration();
    return true;
  }
#endif

  //ROS_WARN("Cost to go home: %f", planner_->getPointPotential( home_pose_msg.pose.position ) );

  int secs_to_home = secsToHome();
  if (secs_to_home == -1) {
    ROS_WARN("??? No plan to get home.");
    return false;
  }

  int battery_time_remaining = batteryTimeRemaining();
  ROS_WARN("Estimated time to reach home: %d seconds", secs_to_home);
  ROS_WARN("Battery time remaining: %d seconds (with battery safety margin %d)",
    battery_time_remaining, battery_safety_margin);

  if (secs_to_home + battery_safety_margin > battery_time_remaining) {
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
/*
bool Explore::shouldGoHome_fast() {
  int time_since_charge = timeSinceCharge();
  int battery_time_remaining = batteryTimeRemaining();

  ROS_WARN("Time elapsed since charge: %d Time remaining on battery: %d",
    time_since_charge, battery_time_remaining);

  ros::Duration time_since_time_to_home_updated = ros::Time::now() - last_time_update_time_to_home;

  ROS_WARN("Time to home: %d Time since updated time to home: %d",
    secs_to_go_home, time_since_time_to_home_updated.sec);

  ROS_WARN("Current max battery duration: %d", battery_duration.sec);

  // If our battery time estimate says to go home, do so
  if (secs_to_go_home + time_since_time_to_home_updated.sec
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
*/

/*
  Estimation of amount of battery time we have left in seconds
*/
int Explore::batteryTimeRemaining() {
  ros::Duration duration_since_charge = ros::Time::now() - last_time_charged;
  ros::Duration duration_remaining = battery_duration - duration_since_charge;
  return duration_remaining.sec;
}

/*
  Send goal to go home to base
*/
void Explore::goHome() {
  ROS_WARN("Going home...");
  setState(STATE_HEADING_HOME);

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

  exploration_runs_++;

#ifdef BATTERY_TIMER
  setState(STATE_CHARGING);
#endif

  // Change next margin depending on time remaining on our battery
  /*
  if ( battery_time_remaining > MIN_BATTERY_SAFETY_MARGIN ) {
    battery_safety_margin--;
  } else if ( battery_time_remaining < MIN_BATTERY_SAFETY_MARGIN ) {
    //battery_safety_margin++;
    battery_safety_margin += MIN_BATTERY_SAFETY_MARGIN;
  }
  */
}

/*
  This callback called when home has been reached (by nav stack)
*/
void Explore::reachedHomeCallback(const actionlib::SimpleClientGoalState& status,
  const move_base_msgs::MoveBaseResultConstPtr& result,
  geometry_msgs::PoseStamped goal)
{
  if (state == STATE_HEADING_HOME) {
    reachedHome();
  }
}

/*
  If we hear a voltage at a predefined level, this is our warning
  of low/empty battery.
*/
bool Explore::atCriticalVoltage() {
  if (battery_voltage <= VOLTAGE_WARNING) {
    ROS_WARN("** At critical voltage. **");
  }
  return battery_voltage <= VOLTAGE_WARNING;
}

/*
  We just hit critical voltage. We may want to alter our battery
  duration here if time since charge compared with time now is different
  from our current battery duration
*/
void Explore::updateBatteryDuration() {
//  TODO/XXX
}

void Explore::setState(int new_state) {
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
  ROS_WARN("Setting state to: %s", s.c_str());
#endif

  notifier_->publish(s);
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

  ros::Rate r(planner_frequency_);

  // We need this initially or may think we should go home already!
  explorer_->computePotentialFromRobot(explore_costmap_ros_, planner_);

  while (node_.ok()) {

    if (close_loops_) {
      tf::Stamped<tf::Pose> robot_pose;
      explore_costmap_ros_->getRobotPose(robot_pose);
      loop_closure_->updateGraph(robot_pose);
    }

    if (state != STATE_CHARGING && exploration_runs_ < EXPLORATION_RUNS) {
#ifdef BATTERY_TIMER
      // If we're heading home, we may be there now
      if ( state == STATE_HEADING_HOME && atHome() ) {
        reachedHome();

      // Should we go home?
      } else if ( state != STATE_HEADING_HOME && state != STATE_CHARGING && shouldGoHome_dynamic() ) {
        goHome();

      // an else if which is continued below out of our hashdef
      } else
  #endif

      // We need a new exploration goal
      if (state != STATE_HEADING_HOME
          && (state == STATE_WAITING_FOR_GOAL
              || atGoal()
#ifdef REEVALUATE_PLANS
              || time_following_plan_ > REEVALUATE_PLANS_TIME
#endif
             )
         )
      {
#ifdef REMAKE_TOPOMAP
        delete topomap_;
        topomap_ = new Topomap(explore_costmap_ros_, home_pose_msg.pose.position.x,
          home_pose_msg.pose.position.y);
#endif
        topomap_->update(explore_costmap_ros_);
        makePlan();

      }

      // We can get stuck (if we're not charging), so handle it.
      checkIfStuck();

      // Track how long we've been following a plan
      if (state == STATE_EXPLORING)
        time_following_plan_ += 1.0f / planner_frequency_;
    }

    if (exploration_runs_ >= EXPLORATION_RUNS) {
      ROS_WARN("* I've done all my exploration runs. Not going anywhere, but still visualising. (%d/%d runs)",
        exploration_runs_, EXPLORATION_RUNS);
    }

    if (visualize_) {
      // publish visualization markers
      std::vector<Marker> markers;
      explorer_->getVisualizationMarkers(markers);
      for (uint i=0; i < markers.size(); i++)
        marker_publisher_.publish(markers[i]);

      // and occupancy map
      publishMap();

      topomap_publisher_marker_id = 0;

      // and topomap
      topomap_publisher_marker_id = topomap_->publish_topomap(&topomap_marker_publisher_, topomap_publisher_marker_id);

      // and blacklisted frontiers
      visualize_blacklisted();

      // and the current plan
      //visualize_plan(current_plan_);
      // current goal
      std::vector<visualization_msgs::Marker> markers2;
      visualize_arrow(topomap_publisher_marker_id,
        current_goal_pose_stamped_.pose.position.x, current_goal_pose_stamped_.pose.position.y,
        3.0, // scale
        0.0, 238.0, 0.0, 0.3, // green
        &markers2, "curgoal");
      topomap_publisher_marker_id++;
      for (std::vector<visualization_msgs::Marker>::const_iterator it = markers2.begin();
        it < markers2.end();
        ++it)
      {
        topomap_marker_publisher_.publish( *it );
      }
    }

#ifdef SIMULATION
    find_furthest_point();
#endif

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

