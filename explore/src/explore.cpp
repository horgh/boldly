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

#define SIMULATION
// Start heading back with this margin of safety (wrt battery time remaining)
#define MARGIN_SECONDS 10

#include <explore/explore.h>
#include <explore/explore_frontier.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace costmap_2d;
using namespace navfn;
using namespace visualization_msgs;
using namespace geometry_msgs;

namespace explore {

double sign(double x){
  return x < 0.0 ? -1.0 : 1.0;
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
  map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_server_ = private_nh.advertiseService("explore_map", &Explore::mapCallback, this);

  private_nh.param("navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("planner_frequency", planner_frequency_, 1.0);
  private_nh.param("progress_timeout", progress_timeout_, 30.0);
  private_nh.param("visualize", visualize_, 1);
  double loop_closure_addition_dist_min;
  double loop_closure_loop_dist_min;
  double loop_closure_loop_dist_max;
  double loop_closure_slam_entropy_max;
  private_nh.param("close_loops", close_loops_, false); // TODO: switch default to true once gmapping 1.1 has been released
  private_nh.param("loop_closure_addition_dist_min", loop_closure_addition_dist_min, 2.5);
  private_nh.param("loop_closure_loop_dist_min", loop_closure_loop_dist_min, 6.0);
  private_nh.param("loop_closure_loop_dist_max", loop_closure_loop_dist_max, 20.0);
  //private_nh.param("loop_closure_loop_dist_max", loop_closure_loop_dist_max, 100.0);
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
  home_point = robot_pose_msg.pose.position;
  ROS_WARN("Robot home at %f, %f, %f", home_point.x, home_point.y, home_point.z);

  // Last time we were at home is right now
  last_time_at_home = ros::Time::now();

#ifdef SIMULATION
  // 60 seconds
  battery_duration = ros::Duration(60.0);
#endif

  // Find robot's max speed
  max_vel_x = 0.0;
  private_nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", max_vel_x);
  ROS_WARN("Robot has max speed %f", max_vel_x);
  assert(max_vel_x != 0.0);
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
	Called when map service on this node is queried?
	Returns map data (whether occupied, etc) in form of nav_msgs::GetMap::Response
*/
bool Explore::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  ROS_DEBUG("mapCallback");
  Costmap2D explore_costmap;
  explore_costmap_ros_->getCostmapCopy(explore_costmap);

  res.map.info.width = explore_costmap.getSizeInCellsX();
  res.map.info.height = explore_costmap.getSizeInCellsY();
  res.map.info.resolution = explore_costmap.getResolution();
  res.map.info.origin.position.x = explore_costmap.getOriginX();
  res.map.info.origin.position.y = explore_costmap.getOriginY();
  res.map.info.origin.position.z = 0;
  res.map.info.origin.orientation.x = 0;
  res.map.info.origin.orientation.y = 0;
  res.map.info.origin.orientation.z = 0;
  res.map.info.origin.orientation.w = 1;

  int size = res.map.info.width * res.map.info.height;
  const unsigned char* map = explore_costmap.getCharMap();

  res.map.set_data_size(size);
  for (int i=0; i<size; i++) {
    if (map[i] == NO_INFORMATION)
      res.map.data[i] = -1;
    else if (map[i] == LETHAL_OBSTACLE)
      res.map.data[i] = 100;
    else
      res.map.data[i] = 0;
  }

  return true;
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

  map.set_data_size(size);
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
	Get goals from explore_frontier & choose one to go to
	If goal has changed not changed from previous, ensure makes progress
	If sufficient time elapsed and could not reach, blacklist
	Sends the goal pose to the base with a callback function to call when goal reached
*/
void Explore::makePlan() {
  //since this gets called on handle activate
  if(explore_costmap_ros_ == NULL)
    return;

  tf::Stamped<tf::Pose> robot_pose;
  explore_costmap_ros_->getRobotPose(robot_pose);

  std::vector<geometry_msgs::Pose> goals;
  explore_costmap_ros_->clearRobotFootprint();
  bool res = explorer_->getExplorationGoals(*explore_costmap_ros_, robot_pose, planner_, goals, potential_scale_, orientation_scale_, gain_scale_);
	/*
	ROS_WARN("getExplorationGoals(): %d Have %d exploration goals/ frontiers", res, goals.size());
	for (std::vector<geometry_msgs::Pose>::iterator it = goals.begin(); it != goals.end(); it++) {
		ROS_WARN("Goal at %f, %f, %f", it->position.x, it->position.y, it->position.z);
		PoseStamped test_goal_pose;
		test_goal_pose.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
		test_goal_pose.header.stamp = ros::Time::now();
		test_goal_pose.pose = *it;
		if ( goalOnBlacklist(test_goal_pose) ) {
			ROS_WARN("Goal is on blacklist");
		}
	}
	*/
  if (goals.size() == 0)
    done_exploring_ = true;

  bool valid_plan = false;
  std::vector<geometry_msgs::PoseStamped> plan;
  PoseStamped goal_pose, robot_pose_msg;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);

  goal_pose.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  goal_pose.header.stamp = ros::Time::now();
  // Calculates navigation from this position to points in costmap
  planner_->computePotential(robot_pose_msg.pose.position); // just to be safe, though this should already have been done in explorer_->getExplorationGoals
#ifdef SIMULATION
  shouldGoHome(robot_pose_msg);
#endif

  int blacklist_count = 0;
  for (unsigned int i=0; i<goals.size(); i++) {
    goal_pose.pose = goals[i];
    if (goalOnBlacklist(goal_pose)) {
      blacklist_count++;
      continue;
    }

		/*
			Build plan to given goal. If such a plan exists, we can use it
		*/
    valid_plan = ((planner_->getPlanFromPotential(goal_pose, plan)) && (!plan.empty()));
    if (valid_plan) {
      break;
    }
  }

  // publish visualization markers
  if (visualize_) {
    std::vector<Marker> markers;
    explorer_->getVisualizationMarkers(markers);
    for (uint i=0; i < markers.size(); i++)
      marker_publisher_.publish(markers[i]);
  }

  if (valid_plan) {
		/*
			If plan size has changed, that means we are making some sort of progress
			Otherwise check if too much time has elapsed and we may need to blacklist the goal
		*/
    if (prev_plan_size_ != plan.size()) {
      time_since_progress_ = 0.0;
    } else {
      double dx = prev_goal_.pose.position.x - goal_pose.pose.position.x;
      double dy = prev_goal_.pose.position.y - goal_pose.pose.position.y;
      double dist = sqrt(dx*dx+dy*dy);
      if (dist < 0.01) {
        time_since_progress_ += 1.0f / planner_frequency_;
      }
    }

    // black list goals for which we've made no progress for a long time
    if (time_since_progress_ > progress_timeout_) {
      frontier_blacklist_.push_back(goal_pose);
      ROS_WARN("Adding current goal to black list");
    }

    prev_plan_size_ = plan.size();
    prev_goal_ = goal_pose;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;
    move_base_client_.sendGoal(goal, boost::bind(&Explore::reachedGoal, this, _1, _2, goal_pose));

    if (visualize_) {
      publishGoal(goal_pose.pose);
      publishMap();
    }
  } else {
    ROS_WARN("Done exploring with %d goals left that could not be reached. There are %d goals on our blacklist, and %d of the frontier goals are too close to them to pursue. The rest had global planning fail to them. \n", (int)goals.size(), (int)frontier_blacklist_.size(), blacklist_count);
    ROS_INFO("Exploration finished. Hooray.");
    done_exploring_ = true;
  }

}

/*
	Check if current goal is too close to a blacklisted frontier
*/
bool Explore::goalOnBlacklist(const geometry_msgs::PoseStamped& goal){
  //check if a goal is on the blacklist for goals that we're pursuing
  for(unsigned int i = 0; i < frontier_blacklist_.size(); ++i){
    double x_diff = fabs(goal.pose.position.x - frontier_blacklist_[i].pose.position.x);
    double y_diff = fabs(goal.pose.position.y - frontier_blacklist_[i].pose.position.y);

    if(x_diff < 2 * explore_costmap_ros_->getResolution() && y_diff < 2 * explore_costmap_ros_->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
    const move_base_msgs::MoveBaseResultConstPtr& result, geometry_msgs::PoseStamped frontier_goal){

  ROS_DEBUG("Reached goal");
  if(status == actionlib::SimpleClientGoalState::ABORTED){
    frontier_blacklist_.push_back(frontier_goal);
    ROS_WARN("Adding current goal to black list (aborted, but reached goal)");
  }

//  if(!done_exploring_){
//    //create a plan from the frontiers left and send a new goal to move_base
//    makePlan();
//  }
//  else{
//    ROS_INFO("Exploration finished. Hooray.");
//  }
}

/*
  Note: Must have called planner_->computePotential() prior to calling this
*/
bool Explore::shouldGoHome(PoseStamped robot_pose_msg) {
  ROS_WARN("Cost to go home: %f", planner_->getPointPotential( home_point ) );

  /*
    Get distance needed to get home
  */
  // Find the plan to go home
  std::vector<geometry_msgs::PoseStamped> plan;
  PoseStamped home_pose;
  home_pose.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  home_pose.header.stamp = ros::Time::now();
  bool valid_plan = planner_->getPlanFromPotential(home_pose, plan) && !plan.empty();
  if (!valid_plan) {
    ROS_WARN("No plan to get home!");
  }

  // Calculate the distance
  PoseStamped prev_pose;
  prev_pose = robot_pose_msg;
  double distance = 0.0;
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin(); it != plan.end(); it++) {
    double dx = prev_pose.pose.position.x - it->pose.position.x;
    double dy = prev_pose.pose.position.y - it->pose.position.y;
    double dist = sqrt(dx*dx + dy*dy);
    distance += dist;
    prev_pose = *it;
  }
  ROS_WARN("Distance to go home: %f meters", distance);

  // XXX probably need to take into account turn time at each point too

  // Estimate time to travel home
  int time_to_home = ceil(distance / max_vel_x);
  ROS_WARN("Time to go home: %d seconds", time_to_home);

  if (time_to_home + MARGIN_SECONDS > batteryTimeRemaining()) {
    ROS_WARN("Decided to return home");
    return true;
  }

  return false;
}

/*
  Estimation of amount of battery time we have left in seconds
*/
int Explore::batteryTimeRemaining() {
#ifdef SIMULATION
  ros::Duration elapsed = ros::Time::now() - last_time_at_home;
  int remaining = battery_duration.sec - elapsed.sec;
  ROS_WARN("Time elapsed since charge: %d Time remaining on battery: %d", elapsed.sec, remaining);
  return remaining;
#else
  return 0;
#endif
}

/*
	Continually run makePlan() after specified sleep
	This will cause goals to become blacklisted if not enough progress is made
	and not just wait for callback when goal is reached
*/
void Explore::execute() {
  while (! move_base_client_.waitForServer(ros::Duration(5,0)))
    ROS_WARN("Waiting to connect to move_base server");

  ROS_INFO("Connected to move_base server");

  // This call sends the first goal, and sets up for future callbacks.
  makePlan();

  ros::Rate r(planner_frequency_);
  while (node_.ok() && (!done_exploring_)) {

    if (close_loops_) {
      tf::Stamped<tf::Pose> robot_pose;
      explore_costmap_ros_->getRobotPose(robot_pose);
      loop_closure_->updateGraph(robot_pose);
    }

    makePlan();
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

int main(int argc, char** argv){
  ros::init(argc, argv, "explore");

  explore::Explore explore;
  explore.spin();

  return(0);
}

