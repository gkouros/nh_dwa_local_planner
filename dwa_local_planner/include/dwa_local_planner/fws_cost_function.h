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

#ifndef DWA_LOCAL_PLANNER_FWS_COST_FUNCTION_H
#define DWA_LOCAL_PLANNER_FWS_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

namespace dwa_local_planner
{

  class FWSCostFunction : public base_local_planner::TrajectoryCostFunction
  {
    public:

      /**
       * @brief Constructor
       * @param r_min: minimum turning radius of the robot
       * @param b_max: maximum diagonal motion angle
       */
      FWSCostFunction(double r_min, double b_max) : r_min_(r_min), b_max_(b_max)
      {
      }

      /**
       * @brief Destructor
       */
      ~FWSCostFunction()
      {
      }

      /**
       * @brief Punishes inadmissible trajectories based on FWS constraints
       * @param traj: The trajectory to be evaluated
       */
      double scoreTrajectory(base_local_planner::Trajectory& traj);

      /**
       * @brief Used to update context values (not used, just returns true)
       */
      bool prepare() {return true;}

      /**
       * @brief Set minimum turning radius
       */
      void setRMin(double r_min) {r_min_ = r_min;}

      /**
       * @brief Set maximum diagonal motion angle
       */
      void setBMax(double b_max) {b_max_ = b_max;}

    private:
      //!< minimum turning radius of the robot
      double r_min_;

      //!< maximum diagonal motion angle
      double b_max_;
  };

}  // namespace dwa_local_planner

#endif  // DWA_LOCAL_PLANNER_FWS_COST_FUNCTION_H
