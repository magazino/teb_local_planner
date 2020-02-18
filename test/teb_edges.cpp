#include <gtest/gtest.h>

#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/edge_velocity.h>

using namespace teb_local_planner;

TEST(TEBEdges, EdgeVelocityAbsoluteError)
{
  TebConfig teb_config;
  teb_config.robot.max_vel_x = 1.;
  teb_config.robot.max_vel_x_backwards = 0.5;
  teb_config.robot.max_vel_y = 1.;
  teb_config.optim.penalty_epsilon = 0.;

  VertexPose v1;
  v1.setEstimate(PoseSE2(0., 0., 0.));
  VertexPose v2;
  v2.setEstimate(PoseSE2(0., 0., 0.));
  VertexTimeDiff v3;
  v3.setEstimate(0.1);

  EdgeVelocityAbsolute e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setVertex(2, &v3);
  e.setInformation(EdgeVelocityAbsolute::InformationType::Identity());
  e.setTebConfig(teb_config);

  v2.setEstimate(PoseSE2(0., 0., 0.)); // no motion
  e.computeError();
  ASSERT_DOUBLE_EQ(0., e.error()(0));

  v2.setEstimate(PoseSE2(0.1, 0., 0.)); // speed is ok
  e.computeError();
  ASSERT_DOUBLE_EQ(0., e.error()(0));

  v2.setEstimate(PoseSE2(0.2, 0., 0.)); // double speed required
  e.computeError();
  ASSERT_DOUBLE_EQ(1., e.error()(0));

  v2.setEstimate(PoseSE2(-0.1, 0., 0.)); // speed is ok
  e.computeError();
  ASSERT_DOUBLE_EQ(0., e.error()(0));

  v2.setEstimate(PoseSE2(-0.2, 0., 0.)); // double speed required
  e.computeError();
  ASSERT_DOUBLE_EQ(1., e.error()(0));
}

TEST(TEBEdges, EdgeVelocityStraightError)
{
  TebConfig teb_config;
  teb_config.robot.max_vel_x = 1.;
  teb_config.robot.max_vel_x_backwards = 0.5;
  teb_config.robot.max_vel_y = 1.;
  teb_config.optim.penalty_epsilon = 0.;

  VertexPose v1;
  v1.setEstimate(PoseSE2(0., 0., 0.));
  VertexPose v2;
  v2.setEstimate(PoseSE2(0., 0., 0.));

  EdgeVelocityStraight e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(EdgeVelocityStraight::InformationType::Identity());
  e.setTebConfig(teb_config);

  v2.setEstimate(PoseSE2(0., 0., 0.)); // no motion
  e.computeError();
  EXPECT_DOUBLE_EQ(0., e.error()(0));

  v2.setEstimate(PoseSE2(1., 0., 0.)); // straight motion
  e.computeError();
  EXPECT_DOUBLE_EQ(0., e.error()(0));
  v2.setEstimate(PoseSE2(-1., 0., 0.)); // straight motion
  e.computeError();
  EXPECT_DOUBLE_EQ(0., e.error()(0));
  v2.setEstimate(PoseSE2(0., 1., 0.)); // straight motion
  e.computeError();
  EXPECT_DOUBLE_EQ(0., e.error()(0));
  v2.setEstimate(PoseSE2(0., -1., 0.)); // straight motion
  e.computeError();
  EXPECT_DOUBLE_EQ(0., e.error()(0));

  v2.setEstimate(PoseSE2(0.2, 1.0, 0.)); // non straight motion results in some error
  e.computeError();
  EXPECT_NE(0., e.error()(0));

  // evaluating the Jacobian at one concrete instance
  v2.setEstimate(PoseSE2(1., 0., 0.)); // straight motion
  g2o::JacobianWorkspace jacobianWorkspace;
  jacobianWorkspace.updateSize(&e);
  jacobianWorkspace.allocate();
  e.linearizeOplus(jacobianWorkspace);
  EXPECT_NEAR(0, e.jacobianOplusXi()(0, 0), 1e-6);
  EXPECT_NEAR(-1, e.jacobianOplusXi()(0, 1), 1e-6);
  EXPECT_NEAR(-1, e.jacobianOplusXi()(0, 2), 1e-6);
  EXPECT_NEAR(0, e.jacobianOplusXj()(0, 0), 1e-6);
  EXPECT_NEAR(1, e.jacobianOplusXj()(0, 1), 1e-6);
  EXPECT_NEAR(0, e.jacobianOplusXj()(0, 2), 1e-6);
}