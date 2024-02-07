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
    float temp_pos_fpl = 100;

    float temp_neg_fpl = -100;
    float dtgoal = 0.0;
    // float arc_len = 0.0;
    PathOption *po = new PathOption();
    po->free_path_length = 100;
    for(int i = -10; i <= 10; i++) {
      float radius = 10 / (i + 1e-6);

      // okay never mind we're going from right to left

      // supposing that curvature = i / 10 since max curvature is 1
      // need location after 1 timestep and goal location
      // not just location, but along the arc, what the closest point is to the robot
      // closest point on arc is the intersection of the line from the center of the circle to the goal
      // calculate how far distance to goal would be
      // using robot as frame of reference, center of turning (if left) is (a, b - r)
      // then equation of the circle is (x - a) ^2 + (y - (b - r))^2 = r^2
      // then closest point is sqrt((c - a) ^2 + (d - (b - r)))^2)) - r

      // treat center of turning as (0, r). robot is located at (0,0).
      // given point x,y which is an obstacle... but we don't know if it's an obstacle until we calculate the arc
      // so I still need to calculate all the arcs. and find the conflict somehow
      float robot_x = 0;
      float robot_y = 0;

      float center_x = robot_x; 
      float center_y = robot_y + radius; // left = negative, so if turn radius is neg that's fine

      // cout << "radius " << radius << endl;
      // cout << "robot points " << robot_x << "    " << robot_y << endl;
      // cout << "center points " << center_x << "    " << center_y << endl;
        // int count = 0;
      // int point_drawn = 0;

      for (Vector2f point : point_cloud_) {
        // cout << "obstacles" << endl;
        // cout << point.x() << endl;
        // cout << point.y() << endl;
        float h = 0.535 + 0.1; // add .1 for safety margin?

        float w = 0.281 / 2 + 0.1;
        float mag = magnitude(point.x() - center_x, point.y() - center_y);
        // cout << "radius " << radius << endl;
        // cout << "wwww " << w << endl;
        float r_1 = radius - w;
        if(radius < 0) r_1 = w - radius;
        float r_2 = magnitude(radius + w, h);
        if(radius < 0) r_2 = magnitude(radius - w, h);
        // cout << "radius " << radius << endl;
        // cout << "pointy" << point.y() << endl;
        float theta = atan2(point.x(), radius - point.y());
        // if(radius > 0) cout << "pointyradius" << radius - point.y() << endl;
        if(radius < 0) theta = atan2(point.x(), 0 - (abs(radius) - point.y()));

        // if(radius < 0) theta = atan2(point.x(),  point.y() - radius);;
        // count++;
        // cout << "countjsklfjsklf" << count << endl;
        // if (theta > 0) {
        //   if (mag >= r_1 && mag <= r_2) {
        //     cout << "mag " << mag << endl;
        //     cout << "r_2 " << r_2 << endl;
        //     cout << "r1 " << r_1 << endl;
        //     cout << "theta " << theta << endl;
        //     cout << endl;
        //   }
          
        // }
        
        if (mag >= r_1 && mag <= r_2 && theta > 0) {
          Eigen::Vector2f p(point.x(), point.y());
          Eigen::Vector2f q(center_x, center_y);
          // point_drawn++;
        // visualization::DrawLine(p, q, 2, local_viz_msg_);

          // if (point_drawn == 1) {
          //     visualization::DrawLine(p, q, 2, local_viz_msg_);
          //     point_drawn = 2;
          // }
          po->obstruction = p;
          temp_pos_fpl = radius * (theta - atan2(h, radius - w));
          // if (i == 1) cout << "positiverawejfklwjr" << theta - atan2(h, radius - w) << endl;
          // if(i == -1) cout << "negativekjfksldfjsl" << theta - atan2(h, w - radius) << endl;
          
          if(radius < 0) temp_neg_fpl = abs(radius * (theta - atan2(h, w - radius)));
          if (temp_pos_fpl < po->free_path_length) {
            po->free_path_length = temp_pos_fpl;
            visualization::DrawPathOption(
              i,
              abs(temp_pos_fpl),
              0,
              14,
              true,
              local_viz_msg_
              );
            // cout << "tempfpl" << temp_fpl << endl;
          }
          if (radius < 0 && temp_neg_fpl > po->free_path_length) {
            po->free_path_length = temp_neg_fpl;
            visualization::DrawPathOption(
              i,
              temp_neg_fpl,
              0,
              14,
              true,
              local_viz_msg_
              );
            // cout << "tempfpl" << temp_fpl << endl;
          }
        }
      }

      // if we hit upon a point in a point cloud, stop checking points and calculate score


      // given robot and a turning center loc, I am able to calculate the arc
      // how to determine intersection?
      // calculate cspace for each point. that is a circle w radius car_len
      // later will have to figure out how to avoid an obstacle...


      // double c_y = robot_loc_.y() + radius
      // double v_x = nav_goal_loc_.x() - c_x;
      // double v_y = nav_goal_loc_.y() - c_y;

      // cout << "cx" << c_x << endl;
      // cout << "cy" << c_y << endl;
      // cout << "vx" << v_x << endl;
      // cout << "vy" << v_y << endl;

     
      

      // given closest point, robot loc, and radius, figure out phi
      // fpl = radius * (atan2(h, radius - w) - atan2(po->obstruction.x(), radius - po->obstruction.y()));

      // cout << "fpl" << fpl << endl;


      // double c_y = robot_loc_.y() + radius
      // double v_x = nav_goal_loc_.x() - c_x;
      // double v_y = nav_goal_loc_.y() - c_y;

      // cout << "cx" << c_x << endl;
      // cout << "cy" << c_y << endl;
      // cout << "vx" << v_x << endl;
      // cout << "vy" << v_y << endl;

      // double magV = sqrt(v_x*v_x + v_y*v_y);
      // cout << "magv" << magV << endl;

      // double p_x = robot_loc_.x() + v_x / magV * radius;
      // double p_y = robot_loc_.y() + v_y / magV * radius;

      // cout << "px" << p_x << endl;
      // cout << "py" << p_y << endl;

      // arcsin is supposed to be between -1 and 1

      // double arc_angle = 2 * asin(0.5 * (pow(c_x - p_x, 2) + pow(c_y - p_y, 2)) / radius);
      // double arc_length = arc_angle * radius;
      // if (arc_angle != arc_angle) {
      //   cout << "iiiii" << i << endl;
      //   cout << "pow(c_x - p_x, 2)" << pow(c_x - p_x, 2) << endl;
      //   cout << "pow(c_y - p_y, 2))" << pow(c_y - p_y, 2) << endl;
      //   cout << "radius" << radius << endl;
      //   cout << "arc_length" << arc_length << endl;


      // }
      // cout << "archangel...?" << arc_angle << endl;
      // cout << "arc_length...?" << arc_length << endl;

      // if point in point cloud collides, exit early
      
      // calculate free path length (dist car travels before obstructed)

      // calculate clearance around obstacle
      
      arc_score = clearance*1 + po->free_path_length + dtgoal*1;
      if (arc_score > best_arc_score) {
        po->curvature = i/10;
        best_arc_score = arc_score;
      }
          // cout << po->curvature << endl;

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
