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
#include<cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>

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
  // drive_msg_.velocity = 1;
  // drive_msg_.curvature = 1;


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

float magnitude(float x, float y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

void Navigation::pick_arc() {
  // for loop of arcs
  // for each arc, get score
  // return the best arc
    float arc_score = 0.0; 
    float best_arc_score = 0.0;
    float clearance = 0.0;
    float temp_fpl = 100;
    float robot_x = 0;
    float robot_y = 0;

    PathOption sample = new PathOption(1000.0,
                                       INFINITY,
                                       1000.0,
                                       Eigen::Vector2f(INFINITY, INFINITY),
                                       Eigen::Vector2f(INFINITY, INFINITY)
                                      );

    // We will define the goal as some distance ahead of the robot 
    Eigen::Vector2f dtgoal(robot_x + 10000.0, robot_y) ; // use this to truncate feasible paths?
    
    vector<PathOption> drawings(21);
    
    int loopcounter = 0;
    
    // -ve x is the right direction, +ve x is the left direction
    // remains the same as the mathemetical convention
    // what do we need magnitude for, what do we need direction for
    // lengths, for comparisons          everything else

    // If we reflect the arcs we want into the other side, we need to reflect
    // the point cloud as well to make sure we are still using the rights points
    // with our "correct"/"right" calculations

    // fpl = f(c, p) if c > 0
    // fpl = f(-c, Mirrored(p)) if c < 0, we flip our curve back into the +ve, but make sure to give it the new points to work with


    for(int i = -10; i <= 10; i++) {

      float radius = 10 / (i + 1e-6);


      cout << endl;

      PathOption& po = drawings.at(loopcounter); // an alias to the existing struct in the drawings vector
      po->free_path_length = 100;
      po->clearance = 100;
      
      float center_x = robot_x; 
      float center_y = robot_y + radius;
      mir_radius = 0.0; // used only for the flipping of certain paths
      bool mirrored = false;
      for (Vector2f point : point_cloud_) {
        // but make sure to flip it so the math is a bit more stable 
        if (radius < 0.0) {
          mirrored = true;
          radius = -1 * radius; 
          // make sure to flip the point as well, so that we don't get any repeats
          point.x() = -1 * point.x();
        } 
        

        // now the math should work as we know it should.
        float mag = magnitude(point.x() - center_x, point.y() - center_y);
        float r_1 = radius - w;
        float r_2 = magnitude(radius + car_width, car_height);
        float theta = atan2(point.x(), radius - point.y());
        float phi = (theta - atan2(h, radius - w));

        // this point is an obstruction for this path
        if (mag >= r_1 && mag <= r_2 ) {
          po->obstruction.x() = point.x();
          po.obstruction.y() = point.y();
          temp_fpl = radius * phi;

          if (temp_fpl < po->free_path_length) {
            po->free_path_length = temp_fpl;
          }

        }
        else if ((mag < r_1 && mag > r_2) && fabs(theta) > 0) { 
          // we know the fpl, so we can see if this the closest point
          // need to do some radius checks with mag.
          // old magnitude will just be the po's 

          // the current closest point with which to judge clearance is either
          // less than r1 or greater than r2

          temp_clear = (mag < r_1) ? fabs(mag) - fabs(r_1) : 
                                     fabs(mag) - fabs(r_2);
          
          if (temp_clear < po->clearance)
            po->clearance = curr_clear;
          
        }

      }
      loopcounter++;

      // calculate clearance around obstacle
      
      arc_score = po->clearance*1 + po->free_path_length + dtgoal*1;
      if (arc_score > best_arc_score) {
        po->curvature = i / 10.0;
        best_arc_score = arc_score;
      }
          // cout << po->curvature << endl;

    }

    cout << "pointoption " << drawings[0].x() << endl;
    

    for (Vector2f point : drawings) {
      // cout << "pointoption" << point.x() << endl;
      // cout << point.y() << endl;

      if (point.x() != 100) {
        visualization::DrawPathOption(
                    point.x(),
                    point.y(),
                    0,
                    0,
                    true,
                    local_viz_msg_
                    );
      }
      
    }
    
    
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