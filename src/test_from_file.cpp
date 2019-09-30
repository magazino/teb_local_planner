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
  TebOptimalPlannerExt tebOptimalPlannerExt;
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = tebOptimalPlannerExt.initOptimizer();
  
  const std::string input_filename = "/tmp/teb.g2o";
  const std::string output_filename = "/tmp/teb-after.g2o";

  // quick test
  optimizer->load(input_filename.c_str());

  ROS_INFO("Loaded graph with %d nodes and %d edges", int(optimizer->vertices().size()), int(optimizer->edges().size()));

  optimizer->initializeOptimization();
  optimizer->computeActiveErrors();
  ROS_INFO("Active error %f", optimizer->activeChi2());

  optimizer->setVerbose(true);
  optimizer->optimize(100);
  optimizer->save(output_filename.c_str());

  // workaround for crash on memory cleanup due to overloaded d'tor in base_teb_edges.h
  for (auto& e : optimizer->edges())
  {
    for(std::size_t i=0; i<e->vertices().size(); ++i)
    {
        e->setVertex(i, nullptr);
    }
  }

  return 0;
}