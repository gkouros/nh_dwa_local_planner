/*********************************************************************
*
* software license agreement (bsd license)
*
*  copyright (c) 2016, George Kouros.
*  all rights reserved.
*
*  redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  this software is provided by the copyright holders and contributors
*  "as is" and any express or implied warranties, including, but not
*  limited to, the implied warranties of merchantability and fitness
*  for a particular purpose are disclaimed. in no event shall the
*  copyright owner or contributors be liable for any direct, indirect,
*  incidental, special, exemplary, or consequential damages (including,
*  but not limited to, procurement of substitute goods or services;
*  loss of use, data, or profits; or business interruption) however
*  caused and on any theory of liability, whether in contract, strict
*  liability, or tort (including negligence or otherwise) arising in
*  any way out of the use of this software, even if advised of the
*  possibility of such damage.
*
* author:  George Kouros
*********************************************************************/

#ifndef NH_DWA_LOCAL_PLANNER_GOAL_ORIENTATION_COST_FUNCTION_H
#define NH_DWA_LOCAL_PLANNER_GOAL_ORIENTATION_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

namespace nh_dwa_local_planner
{

  class GoalOrientationCostFunction : public base_local_planner::TrajectoryCostFunction
  {
    public:

      /**
       * @brief Constructor
       */
      GoalOrientationCostFunction(double distance_scale,
        double orientation_scale, double xy_goal_tolerance):
          distance_scale_(distance_scale),
          orientation_scale_(orientation_scale),
          xy_goal_tolerance_(xy_goal_tolerance) {}

      /**
       * @brief Destructor
       */
      ~GoalOrientationCostFunction(){}

      /**
       * @brief Score trajectories based on final orientation correction
       * @param traj: The trajectory to be evaluated
       */
      double scoreTrajectory(base_local_planner::Trajectory& traj);

      /**
       * @brief Used to update context values (not used, just returns true)
       */
      bool prepare();

      /**
       * @brief Set target pose
       * @param goal_pose: The goal pose
       */
      void setTargetPoses(geometry_msgs::PoseStamped goal_pose);

      /**
       * @brief Set distance scale
       * @param distance_scale: The distance scale
       */
      void setDistanceScale(double distance_scale);

      /**
       * @brief Set orientation scale
       * @param orientation_scale: the orientation scale
       */
      void setOrientationScale(double orientation_scale);

      /**
       * @brief Set xy_goal_tolerance
       * @param xy_goal_tolerance: The xy goal tolerance
       */
      void setXYGoalTolerance(double xy_goal_tolerance);

    private:
      //!< goal pose
      geometry_msgs::PoseStamped goal_pose_;

      //!< distance scale for cost function
      double distance_scale_;

      //!< orientation scale for cost function
      double orientation_scale_;

      //!< xy goal tolerance
      double xy_goal_tolerance_;
  };

}  // namespace nh_dwa_local_planner

#endif  // NH_DWA_LOCAL_PLANNER_GOAL_ORIENTATION_COST_FUNCTION_H
