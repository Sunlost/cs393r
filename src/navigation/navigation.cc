//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // here is where we should be able to see the obstacle and the goal. what does that mean?
  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  drive_msg_.velocity = 1;
  drive_msg_.curvature = 1;

    // cout << "vector0" << point_cloud_.at(0)[0] << endl;

    // cout << "vector1" << point_cloud_[1] << endl;

  // for (Vector2f v : point_cloud_) {
  //   cout << "vectorrr" << v << endl;
  // }


  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = 1;

  // For a fixed arc, implement obstacle detection and integrate it with 1-D TOC to drive up to the observed obstacle.
  // float *goal = nav_goal_loc_.data();
  // cout << "nav goal" << &goal << endl;
  // cout << nav_goal_angle_ << endl;

  // float *curr = robot_loc_.data();
  // cout << "robot goal" << &curr << endl;
  // cout << robot_angle_ << endl;
  pick_arc();

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::pick_arc() {
  // for loop of arcs
  // for each arc, get score
  // return the best arc
    float arc_score = 0.0; 
    float best_arc_score = 0.0;
    float clearance = 0.0;
    float fpl = 0.0;
    float dtgoal = 0.0;
    float arc_len = 0.0;
    PathOption *po = new PathOption();
    for(size_t i = -10; i < 10; i++) {
      // supposing that curvature = i / 10 since max curvature is 1
      // need location after 1 timestep and goal location
      // not just location, but along the arc, what the closest point is to the robot
      // closest point on arc is the intersection of the line from the center of the circle to the goal
      // calculate how far distance to goal would be
      
      // calculate free path length (dist car travels before obstructed)

      // calculate clearance around obstacle
      
      arc_score = clearance*1 + fpl + dtgoal*1;
      if (arc_score > best_arc_score) {
        po->curvature = i/10;
        best_arc_score = arc_score;
      }
          cout << po->curvature << endl;

    }

    visualization::DrawPathOption(
      po->curvature,
      arc_len,
      po->clearance,
      14,
      true,
      local_viz_msg_
      );
    // cout << best_c << endl;

//     struct PathOption {
//   float curvature;
//   float clearance;
//   float free_path_length;
//   Eigen::Vector2f obstruction;
//   Eigen::Vector2f closest_point;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// };
}


}  // namespace navigation
