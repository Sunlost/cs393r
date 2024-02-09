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

float magnitude(double x, double y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

void Navigation::pick_arc() {
    double temp_fpl = 100;
    PathOption *po = new PathOption();
    // remove
    float h = 0.535 + 0.1; // add .1 for safety margin?
    float w = 0.281 / 2 + 0.1;

    PathOption value;
    value.curvature = 100.0;
    value.clearance = INFINITY;
    value.free_path_length = 100.0;
    value.closest_point = Eigen::Vector2f(INFINITY, INFINITY);
    value.obstruction = Eigen::Vector2f(INFINITY, INFINITY);
    vector<PathOption> drawings(21);
    fill(drawings.begin(), drawings.end(), value);
    int loopcounter = 0;
    
    for(double i = -1; i <= 1; i += 0.1) {
      // float i = 0.5;
      double radius = 1 / (i + 1e-6);
      // cout << "the radius" << radius << endl;
      // okay never mind we're going from right to left
      cout << endl;

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

      // an alias to the existing struct in the drawings vector
      PathOption& po = drawings.at(loopcounter);

      double robot_x = 0;
      double robot_y = 0;

      double center_x = robot_x; 
      double center_y = robot_y + radius; // right = negative, so if turn radius is neg that's fine

      for (Vector2f point : point_cloud_) {
        visualization::DrawPoint(point, 0xB92348, local_viz_msg_);
        double mag = magnitude(point.x() - center_x, point.y() - center_y);
        double r_1 = radius - w;
        if(radius < 0) r_1 = w - radius;
        double r_2 = magnitude(radius + w, h);
        if(radius < 0) r_2 = magnitude(abs(radius) + w, h);

        double theta = atan2(point.x(), radius - point.y());
        
        // y is negative too
        if(radius < 0) theta = atan2(point.x(), point.y() - radius);

        double phi = theta - atan2(h, radius - w);
        if(radius < 0) phi = theta - atan2(h, abs(radius) + w);
        // visualization::DrawLine(p0, p1, 0x298329, local_viz_msg_);
        // r2 trips up right side
        // 
        // if point will be between r1 and r2 and will impact the front of the car
        if (mag >= r_1 && mag <= r_2 && theta > 0) {
          Eigen::Vector2f p(point.x(), point.y());
          
          temp_fpl = radius * phi;
          cout << "radius: " << radius << "temp_fpl multiplier" <<  theta - atan2(h, radius - w) << endl;

          // larger radius is the ones closer to center that makes sense

          // temp_fpl = 5; 
          // tempfpl is absolute valued anyway as it is passed into draw.
          // but I should definitely absolute value it before I give it to sun

          // for most of them, I want the theta - omega value to be > 1. but for 
          // high radii, I want the theta - omega value to cut it down

          // if (radius < 0) {
          //   cout << "negthetaaaaa" << theta << endl;
          // } else {
          //   cout << "thetaaa" << theta << endl;
          // }
          // cout << "why is omega small for neg?" << atan2(h, -w - radius) << endl;
          // if(radius < 0) temp_fpl = radius * (theta - atan2(h, -w - radius));
          // if(radius < 0) temp_fpl = abs(radius * (theta - atan2(h, radius + w)));
          // cout << "tempfpl less than minimum fpl" << (temp_fpl < po->free_path_length) << endl;
           if (temp_fpl < po.free_path_length) {
            cout << "ultimate choice" <<  theta - atan2(h, radius - w) << endl;

            po.free_path_length = temp_fpl;
            po.curvature = i;
            po.obstruction = p;

            // May not be the best idea to print out now, since there are still more points to go thru
          }
          // If the point lies outside, then it may affect clearance
        } else if ((mag < r_1 && mag > r_2) && theta > 0) { 
        
          // we know the fpl, so we can see if this the closest point
          // need to do some radius checks with mag.
          // old magnitude will just be the po's 

          // the current closest point with which to judge clearance is either
          // less than r1 or greater than r2

          float temp_clear = (mag < r_1) ? fabs(mag) - fabs(r_1) : 
                                           fabs(mag) - fabs(r_2);
          
          if (temp_clear < po.clearance) {
            Eigen::Vector2f p(point.x(), point.y());
            po.clearance = temp_clear;
            po.closest_point = p;
          }
          
        }
      }

      // Path has gone thru all points, safe to print final curve
      visualization::DrawPathOption(
        po.curvature,
        po.free_path_lengthl,
        po.clearance,
        0,
        true,
        local_viz_msg_
        );
      loopcounter++; // remove

      // calc closest_point on fpl to goal and truncate
      // hardcoding goal to 2.5, 2.5 for now
      // b = goal, a = center, c = point on circle

      // have to adjust p_x maybe

      // need to find b in terms of robot frame of reference
      Eigen::Vector2f adj_goal(0, 5);
      visualization::DrawCross(adj_goal, .3, 0x239847, local_viz_msg_);

      // is it possible that the radius is playing w things?
      double mag = magnitude(adj_goal.x() - center_x, adj_goal.y() - center_y);
      // cout << "mag " << mag << endl;
      double p_x = center_x + (adj_goal.x() - center_x) / mag * abs(radius);
      double p_y = center_y + (adj_goal.y() - center_y) / mag * abs(radius);
      Eigen::Vector2f P(p_x, p_y);
      // given two points and center and radius, calculate arc length

      // cout << "pppp " << P << endl;
      visualization::DrawCross(P, .3, 0xab4865, local_viz_msg_);

      visualization::DrawLine(adj_goal, P, 0, local_viz_msg_);

      
      // calculate clearance around obstacle
      // double dtgoal = magnitude(goal.x(), goal.y());
      // arc_score = clearance*1 + po->free_path_length + dtgoal*1;
      // if (arc_score > best_arc_score) {
      //   po->curvature = i/10;
      //   best_arc_score = arc_score;
      // }
          // cout << po->curvature << endl;

    }    

    // for (Vector2f point : drawings) {
    //   // cout << "pointoption" << point.x() << endl;
    //   // cout << point.y() << endl;

    //   if (point.x() != 100) {
    //     visualization::DrawPathOption(
    //                 point.x(),
    //                 point.y(),
    //                 0,
    //                 0,
    //                 true,
    //                 local_viz_msg_
    //                 );
    //   }
      
    
}


}  // namespace navigation

