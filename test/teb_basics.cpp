#include <gtest/gtest.h>

#include <teb_local_planner/timed_elastic_band.h>

TEST(TEBBasic, autoResizeLargeValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a large timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResizeSmallValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a small timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResize)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // modify the timediff in the middle and add a pose with a smaller timediff as the last one
  teb.TimeDiff(5) = dt + 2*dt_hysteresis;
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, updateAndPruneWithCloserGoal)
{
  const double dt = 0.1;
  teb_local_planner::TimedElasticBand teb;

  // setup a simple TEB (straight line)
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i)
  {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt * i);
  }

  // update with a goal that is now closer to the robot but on the old path
  const teb_local_planner::PoseSE2 new_start(0., 0., 0.);
  const teb_local_planner::PoseSE2 new_goal(7.1, 0., 0.);
  const int min_samples = 3;

  teb.updateAndPruneTEB(new_start, new_goal, min_samples);

  ASSERT_EQ(8, teb.sizePoses()); // expect to remove poses
  ASSERT_FLOAT_EQ(new_goal.position().x(), teb.BackPose().position().x());
  ASSERT_FLOAT_EQ(new_goal.position().y(), teb.BackPose().position().y());
  ASSERT_FLOAT_EQ(new_goal.theta(), teb.BackPose().theta());
  ASSERT_FLOAT_EQ(0.7, teb.BackTimeDiff());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}