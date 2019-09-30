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
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_VIA_POINT_H_
#define EDGE_VIA_POINT_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include "g2o/core/base_unary_edge.h"


namespace teb_local_planner
{

/**
 * @class EdgeViaPoint
 * @brief Edge defining the cost function for pushing a configuration towards a via point
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min  dist2point \cdot weight \f$. \n
 * \e dist2point denotes the distance to the via point. \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgesViaPoints
 * @remarks Do not forget to call setTebConfig() and setViaPoint()
 */     
class EdgeViaPoint : public BaseTebUnaryEdge<1, Eigen::Vector2d, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeViaPoint()
  {
    _measurement.setZero();
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must install the parameters on EdgeViaPoint()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    _error[0] = (bandpt->position() - _measurement).norm();

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeViaPoint::computeError() _error[0]=%f\n",_error[0]);
  }

  bool write(std::ostream& os) const
  {
    os << cfg_->id() << " ";
    os << measurement().x() << " " << measurement().x() << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  bool read(std::istream& is)
  {
    int paramId;
    is >> paramId; // cfg
    setParameterId(0, paramId);
    Eigen::Vector2d aux;
    is >> aux.x() >> aux.y();
    setMeasurement(aux);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
      {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
