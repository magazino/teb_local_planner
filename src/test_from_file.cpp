#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


using namespace teb_local_planner;

/**
 * sub-classing to make a method public for calling below
 */
class TebOptimalPlannerExt : public TebOptimalPlanner
{
  public:
    using TebOptimalPlanner::initOptimizer;
};

int main( int argc, char** argv )
{
  // TODO update config to a reasonable one
  TebConfig teb_config;
  TebOptimalPlannerExt tebOptimalPlannerExt;

  boost::shared_ptr<g2o::SparseOptimizer> optimizer = tebOptimalPlannerExt.initOptimizer();
  
  const std::string input_filename = "/home/kuemmerle/teb-nan.g2o";
  const std::string output_filename = "/home/kuemmerle/teb-nan-after.g2o";

  // quick test
  optimizer->load(input_filename.c_str());

  ROS_INFO("Loaded graph with %d nodes and %d edges", int(optimizer->vertices().size()), int(optimizer->edges().size()));

  // set the required TEB config for the edges
  for (auto e : optimizer->edges())
  {
    if (dynamic_cast<EdgeInflatedObstacle*>(e))
    {
      EdgeInflatedObstacle* ee = static_cast<EdgeInflatedObstacle*>(e);
      ee->setTebConfig(teb_config);
    }
    else if (dynamic_cast<EdgeVelocity*>(e))
    {
      EdgeVelocity* ee = static_cast<EdgeVelocity*>(e);
      ee->setTebConfig(teb_config);
    }
    else if (dynamic_cast<EdgeAccelerationStart*>(e))
    {
      EdgeAccelerationStart* ee = static_cast<EdgeAccelerationStart*>(e);
      ee->setTebConfig(teb_config);
    }
    else if (dynamic_cast<EdgeAcceleration*>(e))
    {
      EdgeAcceleration* ee = static_cast<EdgeAcceleration*>(e);
      ee->setTebConfig(teb_config);
    }
    else if (dynamic_cast<EdgeAccelerationGoal*>(e))
    {
      EdgeAccelerationGoal* ee = static_cast<EdgeAccelerationGoal*>(e);
      ee->setTebConfig(teb_config);
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(e))
    {
      EdgeTimeOptimal* ee = static_cast<EdgeTimeOptimal*>(e);
      ee->setTebConfig(teb_config);
    }
    else if (dynamic_cast<EdgeKinematicsDiffDrive*>(e))
    {
      EdgeKinematicsDiffDrive* ee = static_cast<EdgeKinematicsDiffDrive*>(e);
      ee->setTebConfig(teb_config);
    }
  }

  optimizer->initializeOptimization();
  optimizer->computeActiveErrors();
  ROS_INFO("Active error %f", optimizer->activeChi2());

  optimizer->setVerbose(true);
  optimizer->optimize(100);
  optimizer->save(output_filename.c_str());

  return 0;
}