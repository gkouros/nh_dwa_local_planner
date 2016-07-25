/*********************************************************************
*
* software license agreement (bsd license)
*
*  copyright (c) 2016, George Kouros team.
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

#include "dwa_local_planner/goal_orientation_cost_function.h"
#include <math.h>
#include <tf/tf.h>

namespace dwa_local_planner
{

  double GoalOrientationCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj)
  {
    double cx, cy, cth;  // current pose
    double nx, ny, nth;  // new pose
    double gx, gy, gth;  // goal pose

    traj.getPoint(0, cx, cy, cth);  // get current pose
    traj.getPoint(traj.getPointsSize() - 1, nx, ny, nth);  // get new pose

    // get goal pose
    gx = goal_pose_.pose.position.x;
    gy = goal_pose_.pose.position.y;
    gth = tf::getYaw(goal_pose_.pose.orientation);

    double cg_dist = hypot(cx-gx, cy-gy);  // current distance from goal
    double ng_dist = hypot(nx-gx, ny-gy);  // new distance from goal

    double cg_dth = fabs(cth - gth);  // current orientation error
    double ng_dth = fabs(nth - gth);  // new orientation error

    double score =
      (1 / (activation_factor_ * cg_dist + 1))
      * (distance_scale_ * ng_dist + orientation_scale_ * (ng_dth));

    return (score > 0) ? score : 0;
  }


  void GoalOrientationCostFunction::setTargetPoses(
    geometry_msgs::PoseStamped goal_pose)
  {
    goal_pose_ = goal_pose;
  }


  bool GoalOrientationCostFunction::prepare()
  {
    return true;
  }


  void GoalOrientationCostFunction::setDistanceScale(double distance_scale)
  {
    distance_scale_ = distance_scale;
  }


  void GoalOrientationCostFunction::setOrientationScale(double orientation_scale)
  {
    orientation_scale_ = orientation_scale;
  }


  void GoalOrientationCostFunction::setActivationFactor(double activation_factor)
  {
    activation_factor_ = activation_factor;
  }

}  // namespace dwa_local_planner
