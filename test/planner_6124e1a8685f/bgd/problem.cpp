#include <gtest/gtest.h>

#include <mrnav/planner_6124e1a8685f/models/boost_geom_double.hpp>

using Planner = mrnav::planner_6124e1a8685f::PlannerBGD;
using GK = Planner::GeometryKernel;

TEST(problem, default_)
{
  Planner::Problem prob{1};
  ASSERT_EQ(prob.size(), 1);
  EXPECT_FALSE(prob.valid_p());
}

TEST(problem, simple1)
{
  Planner::Problem prob{2};
  ASSERT_EQ(prob.size(), 2);
  EXPECT_FALSE(prob.valid_p());
  Planner::Configuration p1{2}, p2{2};
  p1[0] = GK::Point{1, 1};
  p1[1] = GK::Point{2, 2};
  p2[0] = GK::Point{10, 10};
  p2[1] = GK::Point{20, 20};
  prob.set_conf_init(p1);
  prob.set_conf_goal(p2);
  const Planner::Configuration &p1_ = prob.get_pos_init();
  const Planner::Configuration &p2_ = prob.get_pos_goal();
  EXPECT_EQ(p1_[0], p1[0]);
  EXPECT_EQ(p1_[1], p1[1]);
  EXPECT_EQ(p2_[0], p2[0]);
  EXPECT_EQ(p2_[1], p2[1]);
  Planner::Robot r1{1}, r2{2};
  prob.set_robot(0, r1);
  prob.set_robot(1, r2);
  EXPECT_EQ(prob.get_robot(0).get_radius(), r1.get_radius());
  EXPECT_EQ(prob.get_robot(1).get_radius(), r2.get_radius());
}
