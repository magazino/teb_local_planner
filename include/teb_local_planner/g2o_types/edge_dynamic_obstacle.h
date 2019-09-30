/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef EDGE_DYNAMICOBSTACLE_H
#define EDGE_DYNAMICOBSTACLE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/robot_footprint_model.h>

namespace teb_local_planner
{
  
/**
 * @class EdgeDynamicObstacle
 * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
 * 
 * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight \f$. \n
 * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */  
class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, double, VertexPose>
{
public:
  
  /**
   * @brief Construct edge.
   */    
  EdgeDynamicObstacle() : obstacle_(nullptr), robot_model_(nullptr), t_(0)
  {
    int paramOffset = numParameters();
    resizeParameters(paramOffset + 2);
    installParameter(obstacle_, paramOffset++);
    installParameter(robot_model_, paramOffset++);
    _measurement = 0.;
  }
  
  /**
   * @brief Construct edge and specify the time for its associated pose (neccessary for computeError).
   * @param t_ Estimated time until current pose is reached
   */      
  explicit EdgeDynamicObstacle(double t) : obstacle_(nullptr), robot_model_(nullptr), t_(t)
  {
    int paramOffset = numParameters();
    resizeParameters(paramOffset + 2);
    installParameter(obstacle_, paramOffset++);
    installParameter(robot_model_, paramOffset++);
    _measurement = 0.;
  }
  
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must install the parameters for EdgeDynamicObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    
    double dist = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), obstacle_, t_);

    _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.dynamic_obstacle_inflation_dist, 0.0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f\n",_error[0]);
  }
  
    /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    _parameterTypes[0] = typeid(&cfg).name();
    _parameterTypes[1] = typeid(obstacle).name();
    _parameterTypes[2] = typeid(robot_model).name();
    setParameterId(1, obstacle->id());
    setParameterId(2, robot_model->id());
  }

  bool write(std::ostream& os) const
  {
    os << cfg_->id() << " ";
    os << obstacle_->id() << " ";
    os << robot_model_->id() << " ";
    os << t_ << " ";
    os << measurement() << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  bool read(std::istream& is)
  {
    int paramOffset = 0;
    int paramId;
    is >> paramId; // cfg
    setParameterId(paramOffset++, paramId);
    is >> paramId; // obstacle
    setParameterId(paramOffset++, paramId);
    is >> paramId; // robot model
    setParameterId(paramOffset++, paramId);

    is >> t_;
    double meas;
    is >> meas;
    setMeasurement(meas);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
      {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

protected:
  
  Obstacle* obstacle_;  //! store the pointer to the obstacle
  BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  double t_; //!< Estimated time until current pose is reached
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    
 
    

} // end namespace

#endif
