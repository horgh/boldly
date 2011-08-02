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

/*
 *  Created on: Apr 6, 2009
 *      Author: duhadway
 */

#include <explore/explore_frontier.h>


using namespace visualization_msgs;
using namespace costmap_2d;

namespace explore {

ExploreFrontier::ExploreFrontier() :
  map_(),
  lastMarkerCount_(0),
  planner_(NULL),
  frontiers_()
{
}

ExploreFrontier::~ExploreFrontier()
{

}

bool ExploreFrontier::getFrontiers(Costmap2DROS& costmap, std::vector<geometry_msgs::Pose>& frontiers)
{
  findFrontiers(costmap);
  if (frontiers_.size() == 0)
    return false;

  frontiers.clear();
  for (uint i=0; i < frontiers_.size(); i++) {
    geometry_msgs::Pose frontier;
    frontiers.push_back(frontiers_[i].pose);
  }

  return (frontiers.size() > 0);
}

float ExploreFrontier::getFrontierCost(const Frontier& frontier) {
  ROS_DEBUG("cost of frontier: %f, at position: (%.2f, %.2f, %.2f)", planner_->getPointPotential(frontier.pose.position),
      frontier.pose.position.x, frontier.pose.position.y, tf::getYaw(frontier.pose.orientation));
  if (planner_ != NULL)
    return planner_->getPointPotential(frontier.pose.position); // / 20000.0;
  else
    return 1.0;
}

/*
	Orientation change of robot wrt frontier?
*/
// TODO: what is this doing exactly?
double ExploreFrontier::getOrientationChange(const Frontier& frontier, const tf::Stamped<tf::Pose>& robot_pose){
  double robot_yaw = tf::getYaw(robot_pose.getRotation());
  double robot_atan2 = atan2(robot_pose.getOrigin().y() + sin(robot_yaw), robot_pose.getOrigin().x() + cos(robot_yaw));
  double frontier_atan2 = atan2(frontier.pose.position.x, frontier.pose.position.y);
  double orientation_change = robot_atan2 - frontier_atan2;
//  ROS_DEBUG("Orientation change: %.3f degrees, (%.3f radians)", orientation_change * (180.0 / M_PI), orientation_change);
  return orientation_change;
}

float ExploreFrontier::getFrontierGain(const Frontier& frontier, double map_resolution) {
  return frontier.size * map_resolution;
}

/*
  Compute potential to each point in the costmap from given position
*/
void ExploreFrontier::computePotentialFromPoint(Costmap2DROS* costmap,
  navfn::NavfnROS* planner,
  geometry_msgs::Point* position)
{
  costmap->clearRobotFootprint();
  planner->computePotential( *position );
}

/*
  Compute potential to each point in the costmap from robot's position
*/
void ExploreFrontier::computePotentialFromRobot(Costmap2DROS* costmap, navfn::NavfnROS* planner) {
  tf::Stamped<tf::Pose> robot_pose;
  costmap->getRobotPose(robot_pose);

  geometry_msgs::PoseStamped robot_pose_msg;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);

  computePotentialFromPoint(costmap, planner, & robot_pose_msg.pose.position );
}

/*
  Rate frontiers according to a new algorithm
*/
bool ExploreFrontier::rateFrontiers(Costmap2DROS& costmap_ros,
  tf::Stamped<tf::Pose> robot_pose, navfn::NavfnROS* planner,
  std::vector<geometry_msgs::Pose>& goals, double potential_scale,
  double orientation_scale, double gain_scale, std::vector<Waypoint*>* topo_map)
{
  // May not have created topomap yet
  if (topo_map == NULL)
    return false;

  // Find the raw frontiers
  findFrontiers(costmap_ros);
  if (frontiers_.size() == 0)
    return false;

  planner_ = planner;
  costmapResolution_ = costmap_ros.getResolution();

  costmap_2d::Costmap2D costmap;
  costmap_ros.getCostmapCopy(costmap);

  // Need average size for filtering out small frontiers
  double average_frontier_size = 0.0;

  // Group close individual frontiers together (crudely)
  for (std::vector<Frontier>::iterator it = frontiers_.begin();
    it < frontiers_.end() ; )
  {
    for (std::vector<Frontier>::iterator it2 = it+1;
      it2 < frontiers_.end() ; )
    {
      // Distance between the two frontiers
      double dx = it->pose.position.x - it2->pose.position.x;
      double dy = it->pose.position.y - it2->pose.position.y;
      double distance = sqrt(dx*dx + dy*dy);

      // We need to convert size of frontier to map units
      double size_in_map1 = gain_scale * getFrontierGain(*it, costmapResolution_);
      double size_in_map2 = gain_scale * getFrontierGain(*it2, costmapResolution_);

      if (distance < std::max(size_in_map1, size_in_map2) / 2.0) {
        // What was this calculation again? Approximate midpoint?
        it->pose.position.x = ( it->pose.position.x + it2->pose.position.x ) / 2.0;
        it->pose.position.y = ( it->pose.position.y + it2->pose.position.y ) / 2.0;
        it->pose.position.z = 0.0;

        // XXX May need to update quaternion (orientation)?

        it->size = (it->size + it2->size)/2.0 + (distance * costmapResolution_);
        it2 = frontiers_.erase(it2);
        continue;
      }
      ++it2;
    }
    average_frontier_size += gain_scale * getFrontierGain(*it, costmapResolution_);
    ++it;
  }

  average_frontier_size /= frontiers_.size();

  // We need the max area and max cost of found frontiers for normalisation
  double max_area = 0.0;
  double max_cost = 0.0;

  // XXX This could be vector of RatedFrontier already, but I don't want to
  // worry about altering frontierRatings() to reflect that
  std::vector<WeightedFrontier> weightedFrontiers;

  // Now we:
  // - filter frontiers out which are too small
  // - update position of frontier to be outside unknown
  // - assign cost
  // - find max cost
  // - find max area
  for (std::vector<Frontier>::iterator it = frontiers_.begin();
    it < frontiers_.end(); )
  {
    // First we filter out frontiers that don't meet a minimum size
    double size = gain_scale * getFrontierGain(*it, costmapResolution_);
    if (size < average_frontier_size / 2.0) {
      it = frontiers_.erase(it);
      continue;
    }

    WeightedFrontier weighted_frontier;
    weighted_frontier.frontier = *it;

    // Frontier may be in an unknown cell. Move it to an open one nearby.
    unsigned int map_x, map_y;
    costmap.worldToMap(weighted_frontier.frontier.pose.position.x, weighted_frontier.frontier.pose.position.y,
      map_x, map_y);
    if (costmap.getCost(map_x, map_y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      Point p = openPoint(weighted_frontier, costmap);
      // openPoint() returns map coords
      double world_x, world_y;
      costmap.mapToWorld(p.x, p.y, world_x, world_y);
      weighted_frontier.frontier.pose.position.x = world_x;
      weighted_frontier.frontier.pose.position.y = world_y;
      //ROS_WARN("x %d y %d map x %d map y %d world x %f world y %f", p.x, p.y, map_x, map_y, world_x, world_y);
    }

    // original explore cost calculation
    //weighted_frontier.cost = potential_scale * getFrontierCost(weighted_frontier.frontier) + orientation_scale * getOrientationChange(weighted_frontier.frontier, robot_pose) - gain_scale * getFrontierGain(weighted_frontier.frontier, costmapResolution_);
    weighted_frontier.cost = potential_scale * getFrontierCost(weighted_frontier.frontier) + orientation_scale
      * getOrientationChange(weighted_frontier.frontier, robot_pose);

    max_cost = std::max((double) weighted_frontier.cost, max_cost);
    max_area = std::max(gain_scale * getFrontierGain(weighted_frontier.frontier, costmapResolution_),
      max_area);

    weightedFrontiers.push_back(weighted_frontier);
    ++it;
  }

  // This gets each frontier's topological rating
  std::vector<FrontierStats> frontier_stats;
  frontierRatings(frontier_stats, weightedFrontiers, costmap, topo_map, 0);
  
  //get the max endness for normalization
  /*double max_end = 0.0;
  for (std::vector<FrontierStats>::const_iterator it = frontier_stats.begin();
    it != frontier_stats.end(); ++it)
  {
    double endness = sqrt(it->vectorx*it->vectorx + it->vectory*it->vectory);
    max_end = std::max(endness, max_end);
  }*/

  rated_frontiers_.clear();
  rated_frontiers_.reserve(weightedFrontiers.size());
  double lambda = 1.0/2000.0;

  /*
    Rate frontiers by goodness & cost.
    Store in rated_frontiers_.
  */
  std::vector<FrontierStats>::const_iterator it2 = frontier_stats.begin();
  for (std::vector<WeightedFrontier>::const_iterator it = weightedFrontiers.begin();
    it != weightedFrontiers.end(); ++it)
  {
    //double size = gain_scale * getFrontierGain(it->frontier, costmapResolution_);
    // Normalise size (area)
    double size = (gain_scale * getFrontierGain(it->frontier, costmapResolution_)) / max_area;

    //double rating = size * exp(-1.0*lambda*it->cost);
    // Normalise rating
    //double rating = size / (it->cost / max_cost);
    //double endness = sqrt(it2->vectorx*it2->vectorx + it2->vectory*it2->vectory);
    double rating = size * std::abs(it2->corrCoeff) * it2->endness;
    
    ROS_WARN("A %f L %f rating %f lambda %f", size, (it->cost / max_cost), rating, lambda);

    RatedFrontier rated_frontier;
    rated_frontier.rating = rating;
    rated_frontier.weighted_frontier = *it;
    rated_frontiers_.push_back(rated_frontier);

    ++it2;
  }

  // Sort rated frontiers: max to min
  std::sort(rated_frontiers_.begin(), rated_frontiers_.end());

  // We return our sorted frontiers as goal poses
  goals.clear();
  goals.reserve(rated_frontiers_.size());
  for (unsigned int i = 0; i < rated_frontiers_.size(); ++i) {
    goals.push_back(rated_frontiers_[i].weighted_frontier.frontier.pose);
  }

  return goals.size() > 0;
}

/*
	Goes through each existing frontier & assigns a cost to each
	Then sorts by cost and returns these as goals

  * This is the original method of rating frontiers in the explore package

  Must call ExploreFrontier::computePotential() first
  Note: This is always called by makePlan() in explore
*/
bool ExploreFrontier::getExplorationGoals(Costmap2DROS& costmap, tf::Stamped<tf::Pose> robot_pose, navfn::NavfnROS* planner, std::vector<geometry_msgs::Pose>& goals, double potential_scale, double orientation_scale, double gain_scale)
{
  findFrontiers(costmap);
  if (frontiers_.size() == 0)
    return false;

/*
  geometry_msgs::Point start;
  start.x = robot_pose.getOrigin().x();
  start.y = robot_pose.getOrigin().y();
  start.z = robot_pose.getOrigin().z();

  planner->computePotential(start);
*/

  planner_ = planner;
  costmapResolution_ = costmap.getResolution();

  //we'll make sure that we set goals for the frontier at least the circumscribed
  //radius away from unknown space
  float step = -1.0 * costmapResolution_;
  int c = ceil(costmap.getCircumscribedRadius() / costmapResolution_);
  WeightedFrontier goal;
  std::vector<WeightedFrontier> weightedFrontiers;
  weightedFrontiers.reserve(frontiers_.size() * c);
  for (uint i=0; i < frontiers_.size(); i++) {
    Frontier& frontier = frontiers_[i];
    WeightedFrontier weightedFrontier;
    weightedFrontier.frontier = frontier;

    tf::Point p(frontier.pose.position.x, frontier.pose.position.y, frontier.pose.position.z);
    tf::Quaternion bt;
    tf::quaternionMsgToTF(frontier.pose.orientation, bt);
    tf::Vector3 v(cos(bt.getAngle()), sin(bt.getAngle()), 0.0);

    for (int j=0; j <= c; j++) {
      tf::Vector3 check_point = p + (v * (step * j));
      weightedFrontier.frontier.pose.position.x = check_point.x();
      weightedFrontier.frontier.pose.position.y = check_point.y();
      weightedFrontier.frontier.pose.position.z = check_point.z();

      weightedFrontier.cost = potential_scale * getFrontierCost(weightedFrontier.frontier) + orientation_scale * getOrientationChange(weightedFrontier.frontier, robot_pose) - gain_scale * getFrontierGain(weightedFrontier.frontier, costmapResolution_);
//      weightedFrontier.cost = getFrontierCost(weightedFrontier.frontier) - getFrontierGain(weightedFrontier.frontier, costmapResolution_);
//      ROS_DEBUG("cost: %f (%f * %f + %f * %f - %f * %f)",
//          weightedFrontier.cost,
//          potential_scale,
//          getFrontierCost(weightedFrontier.frontier),
//          orientation_scale,
//          getOrientationChange(weightedFrontier.frontier, robot_pose),
//          gain_scale,
//          getFrontierGain(weightedFrontier.frontier, costmapResolution_) );
      weightedFrontiers.push_back(weightedFrontier);
    }
  }

  goals.clear();
  goals.reserve(weightedFrontiers.size());
  std::sort(weightedFrontiers.begin(), weightedFrontiers.end());
  for (uint i = 0; i < weightedFrontiers.size(); i++) {
    goals.push_back(weightedFrontiers[i].frontier.pose);
  }
  return (goals.size() > 0);
}

/*
	Build new list of frontiers from the costmap
		- Groups close regions into one large frontier & ensures large enough for robot to pass
*/
void ExploreFrontier::findFrontiers(Costmap2DROS& costmap_) {
  frontiers_.clear();

  Costmap2D costmap;
  costmap_.getCostmapCopy(costmap);

  unsigned int idx;
  unsigned int w = costmap.getSizeInCellsX();
  unsigned int h = costmap.getSizeInCellsY();
  unsigned int size = w * h;

  map_.info.width = w;
  map_.info.height = h;
  //map_.set_data_size(size);
  map_.data.resize(size);
  map_.info.resolution = costmap.getResolution();
  map_.info.origin.position.x = costmap.getOriginX();
  map_.info.origin.position.y = costmap.getOriginY();

  // Find all frontiers (open cells next to unknown cells).
  const unsigned char* map = costmap.getCharMap();
  for (idx = 0; idx < size; idx++) {
//    //get the world point for the index
//    unsigned int mx, my;
//    costmap.indexToCells(idx, mx, my);
//    geometry_msgs::Point p;
//    costmap.mapToWorld(mx, my, p.x, p.y);
//
    //check if the point has valid potential and is next to unknown space
//    bool valid_point = planner_->validPointPotential(p);
    bool valid_point = map[idx] < LETHAL_OBSTACLE;

		// Check if there is a cell with no information around our cell
    if ((valid_point && map) &&
        ( (idx+1 < size && map[idx+1] == NO_INFORMATION) ||
          (idx-1 < size && map[idx-1] == NO_INFORMATION) ||
          (idx+w < size && map[idx+w] == NO_INFORMATION) ||
          (idx-w < size && map[idx-w] == NO_INFORMATION) ))
    {
      map_.data[idx] = -128;
    } else {
      map_.data[idx] = -127;
    }
  }

  // Clean up frontiers detected on separate rows of the map
  idx = map_.info.height - 1;
  for (unsigned int y=0; y < map_.info.width; y++) {
    map_.data[idx] = -127;
    idx += map_.info.height;
  }

  // Group adjoining map_ pixels
  int segment_id = 127;
  std::vector< std::vector<FrontierPoint> > segments;
	// Track which cells are already part of frontier segments
	std::set<int> cells_in_segments;
  for (unsigned int i = 0; i < size; i++) {
		// If the cell is a frontier (open cell next to no info cell)
    if (map_.data[i] == -128) {
			// Already part of a segment
			if (cells_in_segments.count( i ) > 0 ) {
				continue;
			}
      std::vector<int> neighbors;
      std::vector<FrontierPoint> segment;
      neighbors.push_back(i);

      // claim all neighbors
      while (neighbors.size() > 0) {
        unsigned int idx = neighbors.back();
        neighbors.pop_back();
        map_.data[idx] = segment_id;

        btVector3 tot(0,0,0);
        int c = 0;
        if (idx+1 < size && map[idx+1] == NO_INFORMATION) {
          tot += btVector3(1,0,0);
          c++;
        }
        if (idx-1 < size && map[idx-1] == NO_INFORMATION) {
          tot += btVector3(-1,0,0);
          c++;
        }
        if (idx+w < size && map[idx+w] == NO_INFORMATION) {
          tot += btVector3(0,1,0);
          c++;
        }
        if (idx-w < size && map[idx-w] == NO_INFORMATION) {
          tot += btVector3(0,-1,0);
          c++;
        }
        assert(c > 0);
        segment.push_back(FrontierPoint(idx, tot / c));
				cells_in_segments.insert( idx );

				if (segment.size() >= 25) {
					//ROS_WARN("Limiting size of segment");
					neighbors.clear();
					continue;
				}

        // consider 8 neighborhood
        if (idx-1 < size && map_.data[idx-1] == -128)
          neighbors.push_back(idx-1);
        if (idx+1 < size && map_.data[idx+1] == -128)
          neighbors.push_back(idx+1);
        if (idx-map_.info.width < size && map_.data[idx-map_.info.width] == -128)
          neighbors.push_back(idx-map_.info.width);
        if (idx-map_.info.width+1 < size && map_.data[idx-map_.info.width+1] == -128)
          neighbors.push_back(idx-map_.info.width+1);
        if (idx-map_.info.width-1 < size && map_.data[idx-map_.info.width-1] == -128)
          neighbors.push_back(idx-map_.info.width-1);
        if (idx+map_.info.width < size && map_.data[idx+map_.info.width] == -128)
          neighbors.push_back(idx+map_.info.width);
        if (idx+map_.info.width+1 < size && map_.data[idx+map_.info.width+1] == -128)
          neighbors.push_back(idx+map_.info.width+1);
        if (idx+map_.info.width-1 < size && map_.data[idx+map_.info.width-1] == -128)
          neighbors.push_back(idx+map_.info.width-1);
      }
			//ROS_WARN("Segment size %d", segment.size() );

      segments.push_back(segment);
      segment_id--;
      if (segment_id < -127)
        break;
    }
  }

  int num_segments = 127 - segment_id;
  if (num_segments <= 0)
    return;

  for (unsigned int i=0; i < segments.size(); i++) {
    Frontier frontier;
    std::vector<FrontierPoint>& segment = segments[i];
    uint size = segment.size();

    //we want to make sure that the frontier is big enough for the robot to fit through
    if (size * costmap.getResolution() < costmap.getInscribedRadius())
      continue;

    float x = 0, y = 0;
    btVector3 d(0,0,0);

    for (uint j=0; j<size; j++) {
      d += segment[j].d;
      int idx = segment[j].idx;
      x += (idx % map_.info.width);
      y += (idx / map_.info.width);
    }
    d = d / size;
    frontier.pose.position.x = map_.info.origin.position.x + map_.info.resolution * (x / size);
    frontier.pose.position.y = map_.info.origin.position.y + map_.info.resolution * (y / size);
    frontier.pose.position.z = 0.0;

    frontier.pose.orientation = tf::createQuaternionMsgFromYaw(btAtan2(d.y(), d.x()));
    frontier.size = size;

    frontiers_.push_back(frontier);
  }
}

void ExploreFrontier::getVisualizationMarkers(std::vector<Marker>& markers)
{
  /*
    Frontier as a cube
  */
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "frontiers";
  //m.type = Marker::ARROW;
  m.type = Marker::CUBE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  /*
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  */
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  //m.color.a = 255;
  m.color.a = 0.5;
  m.lifetime = ros::Duration(0);
  m.action = Marker::ADD;

  /*
    Frontier's text
  */
  Marker m_text;
  m_text.header.frame_id = "map";
  m_text.header.stamp = ros::Time::now();
  m_text.id = 0;
  m_text.ns = "frontiers";
  m_text.type = Marker::TEXT_VIEW_FACING;
  // only scale.z used
  m_text.scale.z = 0.7;
/*
  m_text.color.r = 191.0;
  m_text.color.g = 84.0;
  m_text.color.b = 46.0;
  m_text.color.a = 255.0;
  */
  m_text.lifetime = ros::Duration(0);
  m_text.color.r = 0.0;
  m_text.color.g = 245.0;
  m_text.color.b = 255.0;
  m_text.color.a = 1.0;

  uint id = 0;
  for (uint i = 0; i < rated_frontiers_.size(); ++i) {
    RatedFrontier rated_frontier = rated_frontiers_[i];

    // Cube
    m.id = id;
    m.pose = rated_frontier.weighted_frontier.frontier.pose;
    m.scale.x = rated_frontier.weighted_frontier.frontier.size / 20.0;
    m.scale.y = rated_frontier.weighted_frontier.frontier.size / 20.0;
    m.scale.z = rated_frontier.weighted_frontier.frontier.size / 20.0;
    markers.push_back(Marker(m));
    id++;

    // Text
    m_text.id = id;
    m_text.pose = rated_frontier.weighted_frontier.frontier.pose;
    // the rating as text
    std::stringstream ss;
    ss << rated_frontier.rating;
    m_text.text = ss.str();
    markers.push_back(Marker(m_text));
    id++;
  }

  m.action = Marker::DELETE;
  for (; id < lastMarkerCount_; id++) {
    m.id = id;
    markers.push_back(Marker(m));
  }

  lastMarkerCount_ = markers.size();
}

//workhorse function for frontierRatings
Line leastSquares(std::vector<Waypoint*> points)
{
    int n = points.size();
    double sum_x = 0;
    double sum_y = 0;
    double sum_xx = 0;
    double sum_xy = 0;
    
    for(std::vector<Waypoint*>::iterator i = points.begin(); i != points.end(); i++)
    {
        sum_x = sum_x + (*i)->x;
        sum_y = sum_y + (*i)->y;

        sum_xx = sum_xx + ((*i)->x * (*i)->x);
        sum_xy = sum_xy + ((*i)->x * (*i)->y);
    }
    
    double m = (-sum_x*sum_y+n*sum_xy)/(n*sum_xx-sum_x*sum_x);
    double b = (-sum_x*sum_xy+sum_xx*sum_y)/(n*sum_xx-sum_x*sum_x);

    return Line(m, b);
}

//workhorse function for frontierRatings
Point ExploreFrontier::openPoint(WeightedFrontier frontier, const costmap_2d::Costmap2D &costmap)
{
    double x_world = frontier.frontier.pose.position.x;
    double y_world = frontier.frontier.pose.position.y;

    unsigned int x;
    unsigned int y;

    costmap.worldToMap(x_world, y_world, x, y);

    ROS_WARN("frontier size %d", frontier.frontier.size);
    
    //for(int rad = 0; rad <= frontier.frontier.size; rad++)
    for(int rad = 0; 1; rad++)
    {
        for(unsigned int i = x - rad; i <= x + rad; i++)
        {
            for(unsigned int j = y - rad; j <= y + rad; j += (i == x-rad || i == x+rad ? 1 : std::max(1, 2*rad)))
            {
                if (!inBounds(i, j, costmap)) {
                  continue;
                }
                //double tx, ty;
                //costmap.worldToMap(i, j, tx, ty);
                //costmap.mapToWorld(i, j, tx, ty);
                //if(calcSpace(i, j, image, NULL) > 1)
                //if(costmap.getCost(tx, ty) < PASSABLE_THRESH)
                if(costmap.getCost(i, j) < PASSABLE_THRESH)
                    return Point(i, j);
            }
        }
    }
    
    //give up and return the center
    return Point(x, y);
}

//workhorse function for frontierRatings
inline double perpenDist(double m, double b, double x, double y)
{
    return abs(y - (m*x) - b) / sqrt((m*m) + 1);
}

inline double linePointPosition2D ( double x1, double y1, double x2, double y2, double x3, float y3 )
{
    return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);   
}

inline double dist(double x1, double y1, double x2, double y2) {
  return std::sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

//this is the this least reliable, but the fastest. I have not copied to slower and more reliable version.
void ExploreFrontier::frontierRatings(std::vector<FrontierStats>& frontier_stats, std::vector<WeightedFrontier>& frontiers, const costmap_2d::Costmap2D& costmap, std::vector<Waypoint*>* topo, int showDebug)
{
    int counter = 1;
    
    for(std::vector<WeightedFrontier>::iterator i = frontiers.begin(); i != frontiers.end(); i++)
    {
        if(showDebug >= 1)
            std::cout << "Analyzing frontier " << counter++ << "/" << frontiers.size() << std::endl;
    
        WeightedFrontier frontier = (*i);
        Point startingPoint = openPoint(frontier, costmap);
        //CONVERT THE MAP COORDS OF OPENPOINT TO WORLD
        double tx, ty;
        costmap.mapToWorld(startingPoint.x, startingPoint.y, tx, ty);
        
        double bestDist = DBL_MAX;
        Waypoint * best = NULL;
        //find the closest waypoint
        for(std::vector<Waypoint*>::iterator j = topo->begin(); j != topo->end(); j++)
        {
            if((*j)->x == tx && (*j)->y == ty)
                continue;
        
            if(dist((*j)->x, (*j)->y, tx, ty) < bestDist)
            {
                bestDist = dist((*j)->x, (*j)->y, tx, ty);
                best = (*j);
            }
        }
        
        std::vector<Waypoint*> waypoints;
        std::queue<Waypoint*> bfs;
        if(best != NULL)
        {
            bfs.push(best);
            waypoints.push_back(best);
        }
            
        //bfs from that waypoint
        for(int j = 1; j < FRONTIERDEPTH && !bfs.empty(); )
        {
            Waypoint * current = bfs.front();
            bfs.pop();
            
            for(std::vector<Waypoint*>::iterator k = current->neighbors.begin(); k != current->neighbors.end(); k++)
            {
                bool alreadyVisited = false;
                for(std::vector<Waypoint*>::iterator m = waypoints.begin(); m != waypoints.end(); m++)
                    if((*m) == (*k))
                    {
                        alreadyVisited = true;
                        break;
                    }
                
                if(!alreadyVisited)
                {
                    j++;
                    waypoints.push_back(*k);
                    bfs.push(*k);
                }
            }
        }
        
        Line bestFit = leastSquares(waypoints);

        double frontierDelta = perpenDist(bestFit.m, bestFit.b, tx, ty);

        double lineDeltas = 0.0;
        /*double xsum = 0.0;
        double ysum = 0.0;
        double xysum = 0.0;
        double xsquares = 0.0;
        double ysquares = 0.0;*/
        int n = (waypoints.size() - 1);
        int sidea = 0; // 'right'
        int sideb = 0; // 'left'
        int sidec = 0; // on line
        double recipc = ty - ((1/bestFit.m)*tx);
        //do not include the home waypoint, or else we double count deltas
        //also, calc the x and y averages while we iterate
        for(std::vector<Waypoint*>::iterator j = waypoints.begin() + 1; j < waypoints.end(); ++j)
        {
            /*xsum += (*j)->x;
            ysum += (*j)->y;
            xysum += (*j)->x * (*j)->y;
            xsquares += (*j)->x*(*j)->x;
            ysquares += (*j)->y*(*j)->y;*/
            
            //we're going to count the number of waypoints on either side of the line perpendicular to the best fit line at the frontier
            int side = linePointPosition2D(0, recipc, 1, (1/bestFit.m) + recipc, (*j)->x, (*j)->y);
            if(side < 0)
                sidea++;
            else if(side > 0)
                sideb++;
            else
                sidec++;
            
            lineDeltas += perpenDist(bestFit.m, bestFit.b, (*j)->x, (*j)->y);
        }
        lineDeltas /= n;

        double vectorsum [2];
        //get the differences from the average
        //and get the vector sum to all waypoints from the frontier
        for(std::vector<Waypoint*>::iterator j = waypoints.begin() + 1; j < waypoints.end(); ++j)
        {
            vectorsum[0] += (*j)->x - tx;
            vectorsum[1] += (*j)->y - ty;
        }
        
        //double denom = sqrt((n*xsquares - xsum*xsum)*(n*ysquares - ysum*ysum));
        //prevent divide by zero
        //if(denom == 0.0)
        //    denom = 1.0;
        
        //note, corrCoeff is now decieving. It is not actually the correlation coefficient, as that does not work for all linearity.
        //It is actually 1 / (s + 1) where s is the sum of the perpendistances.
        //I am temporarily satisfied with this as the current measure of linearity. Needs endness to qualify.
        double corrCoeff = 1 / (lineDeltas + 1);
        
        //for now? one minus the percentage of nearby waypoints NOT on the correct side. So, 1 is good.
        //Since this value by itself cannot be less than 0.5, we subtract 0.5 and multiply by two.
        double smallSide = std::min(sidea, sideb);
        double endness = 2*((1 - (smallSide / waypoints.size())) - 0.5);

        //use frontierstats in your formula
        frontier_stats.push_back(FrontierStats(frontierDelta, lineDeltas, corrCoeff, vectorsum[0], vectorsum[1], endness));
    }
}

}
