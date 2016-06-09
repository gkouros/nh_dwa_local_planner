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

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS,
  nav_core::BaseLocalPlanner)

namespace dwa_local_planner
{

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level)
  {
      if (setup_ && config.restore_defaults)
      {
        config = default_config_;
        config.restore_defaults = false;
      }

      if (!setup_)
      {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_trans_vel = config.max_trans_vel;
      limits.min_trans_vel = config.min_trans_vel;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_rot_vel = config.max_rot_vel;
      limits.min_rot_vel = config.min_rot_vel;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_limit_trans = config.acc_limit_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.rot_stopped_vel = config.rot_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false)
  {
  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (! isInitialized())
    {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      // create the actual planner that we'll use
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }

      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb =
        boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  bool DWAPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    if (!isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call "
        "initialize() before using this planner");
      return false;
    }

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached()
  {
    if (!isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call "
        "initialize() before using this planner");
      return false;
    }

    // get current robot pose
    if (!costmap_ros_->getRobotPose(current_pose_))
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    //current_pose_ and goal_pose
    tf::Stamped<tf::Pose> goal_pose;
    if (!planner_util_.getGoal(goal_pose))
    {
      ROS_ERROR("Could not get goal pose");
      return false;
    }

    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();
    double goal_theta = tf::getYaw(goal_pose.getRotation());

    double xy_goal_tolerance =
      planner_util_.getCurrentLimits().xy_goal_tolerance;
    double yaw_goal_tolerance =
      planner_util_.getCurrentLimits().yaw_goal_tolerance;

    double robot_goal_distance = base_local_planner::getGoalPositionDistance(
      current_pose_, goal_x, goal_y);
    double robot_goal_angle_diff =
      base_local_planner::getGoalOrientationAngleDifference(
        current_pose_, goal_theta);

    if( robot_goal_distance <= xy_goal_tolerance
      && fabs(robot_goal_angle_diff) <= yaw_goal_tolerance)
    {
      ROS_INFO("Goal reached");
      return true;
    }
    else
    {
      return false;
    }
  }


  void DWAPlannerROS::publishLocalPlan(
    std::vector<geometry_msgs::PoseStamped>& path)
  {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void DWAPlannerROS::publishGlobalPlan(
    std::vector<geometry_msgs::PoseStamped>& path)
  {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }


  DWAPlannerROS::~DWAPlannerROS()
  {
    //make sure to clean things up
    delete dsrv_;
  }


  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call "
        "initialize() before using this planner");
      return false;
    }

    // get current robot velocity
    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    // get base frame id for drive cmd
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    base_local_planner::Trajectory path =
      dp_->findBestPath(
        current_pose_, robot_vel, drive_cmds,
        costmap_ros_->getRobotFootprint());

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0)
    {
      ROS_WARN_THROTTLE_NAMED(2, "dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions "
          "discarded all candidates. This can mean there is an obstacle too "
          "close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner",
      "A valid velocity command of (%.2f,%.2f,%.2f) was found for this cycle.",
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i)
    {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
        tf::Stamped<tf::Pose>(tf::Pose(
            tf::createQuaternionFromYaw(p_th),
            tf::Point(p_x, p_y, 0.0)),
          ros::Time::now(),
          costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);

    return true;
  }


  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (!costmap_ros_->getRobotPose(current_pose_))
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
    {
      ROS_WARN_NAMED("dwa_local_planner",
        "Received an empty transformed plan.");
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner",
      "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan);

    if (!isGoalReached())
    {
      if (dwaComputeVelocityCommands(cmd_vel))
      {
        publishGlobalPlan(transformed_plan);
      }
      else
      {
        ROS_WARN_NAMED("dwa_local_planner",
          "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> emptyPlan;
        publishGlobalPlan(emptyPlan);
        return false;
      }
    }
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;

      // publish empty global and local plans
      std::vector<geometry_msgs::PoseStamped> emptyPlan;
      publishGlobalPlan(emptyPlan);
      publishLocalPlan(emptyPlan);
    }

    return true;
  }
};
