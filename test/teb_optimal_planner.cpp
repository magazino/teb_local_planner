#include <gtest/gtest.h>

#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/optimal_planner.h>

using namespace teb_local_planner;

namespace {

using TebSetup = auto (*)(TebConfig&) -> void;
typedef std::vector<geometry_msgs::PoseStamped> TebInitialPlan;
typedef std::tuple<TebSetup, geometry_msgs::Twist, TebInitialPlan> TebInitialConfiguration;

geometry_msgs::PoseStamped toPoseStamped(double x, double y, double theta)
{
  geometry_msgs::PoseStamped result;
  result.header.frame_id = "/map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = 0.;
  result.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  return result;
}

geometry_msgs::Twist toStartVel(double vx, double vy, double omega)
{
  geometry_msgs::Twist result;
  result.linear.x = vx;
  result.linear.y = vy;
  result.angular.z = omega;
  return result;
}

void setup_diff_drive(TebConfig& teb_config)
{
  teb_config.robot.max_vel_x = 1.;
  teb_config.robot.max_vel_x_backwards = 0.5;
  teb_config.robot.max_vel_y = 0.;
  teb_config.optim.penalty_epsilon = 0.05;
  teb_config.trajectory.control_look_ahead_poses = 3;
}

void setup_omni_drive(TebConfig& teb_config)
{
  teb_config.robot.max_vel_x = 1.;
  teb_config.robot.max_vel_x_backwards = 0.5;
  teb_config.robot.max_vel_y = 1.;
  teb_config.optim.penalty_epsilon = 0.05;
  teb_config.optim.weight_max_vel_absolute = 2.;
  teb_config.optim.weight_straight_velocity = 0.005;
  teb_config.trajectory.control_look_ahead_poses = 3;
}

} // end namespace

/**
 * \brief fixture for general purpose TEB testing
 */
class TebOptimalPlannerFixture : public ::testing::TestWithParam<TebInitialConfiguration>
{
protected:
  virtual void SetUp()
  {
    std::get<0>(GetParam())(teb_config); // setting up the configuration
    optimal_planner.reset(new TebOptimalPlanner(teb_config));
  }

  virtual void TearDown()
  {
    optimal_planner->clearPlanner();
  }

  TebConfig teb_config;
  TebOptimalPlannerPtr optimal_planner;
};

TEST_P(TebOptimalPlannerFixture, TebYieldsVelocityCommands)
{
  const geometry_msgs::Twist& initial_vel = std::get<1>(GetParam());
  const TebInitialPlan &initial_plan = std::get<2>(GetParam());
  optimal_planner->plan(initial_plan);

  // expect a valid command that moves the robot
  double vx, vy, omega;
  optimal_planner->getVelocityCommand(vx, vy, omega, teb_config.trajectory.control_look_ahead_poses);
  EXPECT_TRUE(std::abs(vx) > 0.01 || std::abs(vy) > 0.01 || std::abs(omega) > 0.01);
}

// setting up the test suite
namespace {
  auto drive_types = ::testing::Values(
      setup_diff_drive,
      setup_omni_drive
  );
  auto start_vel = ::testing::Values(
      toStartVel(0, 0, 0),
      toStartVel(0.1, 0, 0),
      toStartVel(-0.1, 0, 0),
      toStartVel(0, 0, 0.1),
      toStartVel(0, 0, -0.1)
  );
  auto plans_to_test = ::testing::Values(
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(0., 0., M_PI_4)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(0., 0., -M_PI_4)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(0., 0., M_PI_2)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(0., 0., -M_PI_2)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(1., 0., 0.)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(-1., 0., 0.)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(1., 1., 0.)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(-1., 1., 0.)}),
      TebInitialPlan({toPoseStamped(0., 0., 0.), toPoseStamped(1e-6, 0., 0.), toPoseStamped(0., 0., -M_PI_4)})
  );
}

INSTANTIATE_TEST_CASE_P(
        TebOptimalPlanner,
        TebOptimalPlannerFixture,
        testing::Combine(drive_types, start_vel, plans_to_test));