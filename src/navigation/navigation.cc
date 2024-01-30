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
#include <algorithm>

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

bool decel_started;

float d_curr;
float d_max;

float v_max;
float a_max;
float decel_max;

int cycles_per_second;
float cycle_time;

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

  // decel started flag
  decel_started = false;

  // distance we have travelled so far.
  d_curr = 0;
  // distance we want to go. arbitrarily decide this to be 10m to test
  d_max = 10;

  // max velocity: 1.0 m/s
  v_max = 1.0;
  // max acceleration: 4.0 m/s^2
  a_max = 4.0;
  // max deceleration: 4.0 m/s^2
  decel_max = -4.0;

  cycles_per_second = 20;
  cycle_time = (float) 1 / cycles_per_second;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
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

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;
  
  toc1dstraightline();

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

// TODO: figure out how we work in the (v, ω) space -- our max velocity may be capped <1 when on a sharp curve
  // might also only be when we set v_max to 2 for the bonus problems, etc. should probably test on real car.
void Navigation::toc1dstraightline() {
  if(!decel_started) {
    if(drive_msg_.velocity == v_max) {
      // assume we cruise at max speed for one time step and then decel as fast as possible.
      // do we go over our desired endpoint?
      float v_i = drive_msg_.velocity;
      float v_f = 0;
      
      // calc total distance travelled if we cruise this cycle
      // d = vt
      float d_after_this_cycle = (d_curr + (v_max / cycles_per_second));

      // calc total distance travelled in the future if we start decelerating next cycle
      // v_f^2 = v_i^2 + 2ad
      // d = (v_f^2 - v_i^2) / (2a)
      float d_after_decel_to_zero = (v_f - pow(v_i, 2)) / (2 * decel_max)
                                    + d_after_this_cycle;

      if(d_after_decel_to_zero > d_max) { // handle decel phase at end
        decel_started = true;
      } else { // cruise phase
        drive_msg_.velocity = v_max; // redundant
        d_curr = d_after_this_cycle;
        return;
      } 
    } else {
      // assume we accel as much as possible for one time step and then decel as fast as possible.
      // do we go over our desired endpoint?
      float v_i = drive_msg_.velocity;

      // calc our final velocity if we accelerate as much as we can in 1 cycle, up to a cap of v_max
      // V_f = V_i + at
      float v_f = v_i + (a_max * cycle_time);

      // distance travelled while accelerating up to v_max
      float d_accel;
      // distance travelled at v_max
      float d_at_max_vel;

      if(v_f <= v_max) {
        // d_accel = (v_f^2 - v_i^2) / (2a)
        d_accel = (pow(v_f, 2) - pow(v_i, 2)) / (2 * a_max);
        // if v_f <= v_max, then d_at_max_vel = 0.
        d_at_max_vel = 0;
      } else {
        // d_accel = (v_f^2 - v_i^2) / (2a)    [in this case, v_f == v_max]
        d_accel = (pow(v_max, 2) - pow(v_i, 2)) / (2 * a_max);

        // calc time to reach max velocity
        // V_f = V_i + at    ->    t = (V_f - V_i) / a    [in this case, v_f == v_max]
        float t_accel = (v_max - v_i) / a_max;

        // we're at max velocity for the remainder of the cycle
        float t_at_max_vel = cycle_time - t_accel;

        // d_at_max_vel = v * t
        d_at_max_vel = v_max * t_at_max_vel;
      }

      // cap our v_f at v_max if v_f > v_max
      v_f = std::min(v_f, v_max);    

      float d_after_this_cycle = d_curr + d_accel + d_at_max_vel;

      // calc total distance travelled in the future if we start decelerating next cycle
      // v_f^2 = v_i^2 + 2ad
      // d = (v_f^2 - v_i^2) / (2a)
      float v_i2 = v_f;
      float v_f2 = 0;
      float d_after_decel_to_zero = (v_f2 - pow(v_i2, 2)) / (2 * decel_max)
                                    + d_after_this_cycle;

      if(d_after_decel_to_zero > d_max) { // handle decel phase at end
        decel_started = true;
      } else { // accel phase
        drive_msg_.velocity = v_f;
        d_curr = d_after_this_cycle;
        return;
      }
    }
  }
  // decel phase

  // how hard can we decel
  // v_f = v_i + at
  float v_i = drive_msg_.velocity;
  float v_f = v_i + (decel_max * cycle_time);
  if(v_f < 0) v_f = 0;

  // how far do we go while decelerating
  // d = (v_f^2 - v_i^2) / (2a)
  float d = (pow(v_f, 2) - pow(v_i, 2)) / (2 * decel_max);

  drive_msg_.velocity = v_f;
  d_curr = d_curr + d;

  return;
}


}  // namespace navigation
