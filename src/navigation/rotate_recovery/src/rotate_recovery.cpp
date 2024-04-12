/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <rotate_recovery/rotate_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)

namespace rotate_recovery
{

std::vector<geometry_msgs::PoseStamped> clear_poses;
sensor_msgs::PointCloud2 cloud;
pcl::PointCloud<pcl::PointXYZ> pclCloud;

RotateRecovery::RotateRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void RotateRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.39;
    pose.pose.position.y = 2.90;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.127;
    pose.pose.position.y = 2.909;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.407;
    pose.pose.position.y = 2.483;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.0727;
    pose.pose.position.y = 1.685;
    clear_poses.push_back(pose);
    pose.pose.position.x = 0.237;
    pose.pose.position.y = 1.68346;
    clear_poses.push_back(pose);
    pose.pose.position.x = 0.232339;
    pose.pose.position.y = 2.249488115;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.38554;
    pose.pose.position.y = 1.05716;
    clear_poses.push_back(pose);
    pose.pose.position.x = 2.266218;
    pose.pose.position.y = 1.06534;
    clear_poses.push_back(pose);
    pose.pose.position.x = 2.25193119;
    pose.pose.position.y = 2.492686;
    clear_poses.push_back(pose);

    pose.pose.position.x = 2.273493051;
    pose.pose.position.y = 1.960126876;
    clear_poses.push_back(pose);
    pose.pose.position.x = 2.197135925292;
    pose.pose.position.y = 1.5227453;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.392051;
    pose.pose.position.y = 2.01413138;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.373542;
    pose.pose.position.y = 1.44292;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.864102;
    pose.pose.position.y = 0.750898;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.2383728;
    pose.pose.position.y = 0.801893;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.874432921;
    pose.pose.position.y = -0.56531822;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.9072229;
    pose.pose.position.y = 0.793015718;
    clear_poses.push_back(pose);
    pose.pose.position.x = 2.536913156;
    pose.pose.position.y = -0.4763344;
    clear_poses.push_back(pose);
    pose.pose.position.x = 1.24000;
    pose.pose.position.y = -0.45738840;
    clear_poses.push_back(pose);

    pose.pose.position.x = 0.184659;
    pose.pose.position.y = 0.4815458;
    clear_poses.push_back(pose);
    pose.pose.position.x = 0.2556463;
    pose.pose.position.y = 1.15818715;
    clear_poses.push_back(pose);
    pose.pose.position.x = 2.2106778;
    pose.pose.position.y = 0.37473;
    clear_poses.push_back(pose);

    for (const auto& poseStamped : clear_poses)
    {
        pcl::PointXYZ point;
        point.x = poseStamped.pose.position.x;
        point.y = poseStamped.pose.position.y;
        point.z = 0.0;
        pclCloud.points.push_back(point);
    }

    pcl::toROSMsg(pclCloud, cloud);
    cloud.header.frame_id = "map";

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateRecovery::~RotateRecovery()
{
  delete world_model_;
}


double calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    // double dz = pose1.pose.position.z - pose2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy);
}

double findClosestPose(const geometry_msgs::PoseStamped& targetPose, const std::vector<geometry_msgs::PoseStamped>& poses, geometry_msgs::PoseStamped& closestPose)
{
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& pose : poses)
    {
        double distance = calculateDistance(targetPose, pose);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestPose = pose;
        }
    }

    return minDistance;
}

void RotateRecovery::runBehavior()
{
  // if (!initialized_)
  // {
  //   ROS_ERROR("This object must be initialized before runBehavior is called");
  //   return;
  // }

  // if (local_costmap_ == NULL)
  // {
  //   ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
  //   return;
  // }
  // ROS_WARN("Rotate recovery behavior started.");

  // ros::Rate r(frequency_);
  // ros::NodeHandle n;
  // ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // geometry_msgs::PoseStamped global_pose;
  // local_costmap_->getRobotPose(global_pose);

  // double current_angle = tf2::getYaw(global_pose.pose.orientation);
  // double start_angle = current_angle;

  // bool got_180 = false;

  // while (n.ok() &&
  //        (!got_180 ||
  //         std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
  // {
  //   // Update Current Angle
  //   local_costmap_->getRobotPose(global_pose);
  //   current_angle = tf2::getYaw(global_pose.pose.orientation);

  //   // compute the distance left to rotate
  //   double dist_left;
  //   if (!got_180)
  //   {
  //     // If we haven't hit 180 yet, we need to rotate a half circle plus the distance to the 180 point
  //     double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
  //     dist_left = M_PI + distance_to_180;

  //     if (distance_to_180 < tolerance_)
  //     {
  //       got_180 = true;
  //     }
  //   }
  //   else
  //   {
  //     // If we have hit the 180, we just have the distance back to the start
  //     dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
  //   }

  //   double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

  //   // check if that velocity is legal by forward simulating
  //   double sim_angle = 0.0;
  //   while (sim_angle < dist_left)
  //   {
  //     double theta = current_angle + sim_angle;

  //     // make sure that the point is legal, if it isn't... we'll abort
  //     double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
  //     if (footprint_cost < 0.0)
  //     {
  //       ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
  //                 footprint_cost);
  //       return;
  //     }

  //     sim_angle += sim_granularity_;
  //   }

  //   // compute the velocity that will let us stop by the time we reach the goal
  //   double vel = sqrt(2 * acc_lim_th_ * dist_left);

  //   // make sure that this velocity falls within the specified limits
  //   vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  //   geometry_msgs::Twist cmd_vel;
  //   cmd_vel.linear.x = 0.0;
  //   cmd_vel.linear.y = 0.0;
  //   cmd_vel.angular.z = vel;

  //   vel_pub.publish(cmd_vel);

  //   r.sleep();
  // }

  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/clear_poses", 1);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher pub_closest = n.advertise<geometry_msgs::PoseStamped>("/closest_poses", 10);
  geometry_msgs::PoseStamped global_pose;
  geometry_msgs::PoseStamped closest_pose;
  local_costmap_->getRobotPose(global_pose);
  int cnt = 0;
  double dis;
  findClosestPose(global_pose, clear_poses, closest_pose);
  dis = calculateDistance(global_pose, closest_pose);
  while (n.ok() && cnt < 100 && dis > 0.1)
  {
    ROS_INFO("start");
    cnt++;
    // Update Current Angle
    local_costmap_->getRobotPose(global_pose);
    double yaw = tf2::getYaw(global_pose.pose.orientation);
  
    double dx = closest_pose.pose.position.x - global_pose.pose.position.x;
    double dy = closest_pose.pose.position.y - global_pose.pose.position.y;

    Eigen::Vector2d v(cos(yaw) * dx + sin(yaw) * dy,
                    - sin(yaw) * dx + cos(yaw) * dy);
    double norm = v.norm();
    v(0) = v(0) / norm * dis * 2;
    v(1) = v(1) / norm * dis * 2;

    if(fabs(v(0)) > 0.3)
      v *= 0.5 / fabs(v(0));
    if(fabs(v(1)) > 0.3)
      v *= 0.5 / fabs(v(1));

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = v(0);
    cmd_vel.linear.y = v(1);
    cmd_vel.angular.z = 0;
    vel_pub.publish(cmd_vel);

    ROS_INFO("PUB");
    pub.publish(cloud);
    closest_pose.header.frame_id = "map";
    pub_closest.publish(closest_pose);
    r.sleep();
    dis = calculateDistance(global_pose, closest_pose);
  }

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;
  vel_pub.publish(cmd_vel);
}
};  // namespace rotate_recovery
