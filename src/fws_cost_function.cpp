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

#include "nh_dwa_local_planner/fws_cost_function.h"
#include <math.h>
#include <ros/ros.h>

namespace nh_dwa_local_planner
{

  double FWSCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj)
  {
    if ((fabs(traj.xv_) < 1e-2)
      || fabs(traj.xv_ * traj.yv_ * traj.thetav_) > 0.0
      || fabs(traj.xv_ / traj.thetav_) < r_min_
      || fabs(traj.yv_ / traj.xv_) > tan(b_max_))
    {
      return -1;
    }
    else
    {
      return 0;
    }
  }

}  // namespace nh_dwa_local_planner
