#include <gtest/gtest.h>

#include <mrnav/planner_6124e1a8685f/models/boost_geom_double.hpp>

using Planner = mrnav::planner_6124e1a8685f::PlannerBGD;
using GK = Planner::GeometryKernel;

TEST(configuration, default_)
{
  Planner::Configuration conf{1};
  ASSERT_EQ(conf.size(), 1);
  EXPECT_EQ(conf[0], GK::Point{});
}

TEST(configuration, simple1)
{
  Planner::Configuration conf{2};
  ASSERT_EQ(conf.size(), 2);
  EXPECT_EQ(conf[0], GK::Point{});
  EXPECT_EQ(conf[1], GK::Point{});
  GK::Point p1{1, 1};
  conf[0] = p1;
  EXPECT_EQ(conf[0], p1);
  EXPECT_EQ(conf[1], GK::Point{});
  GK::Point p2{2, 2};
  conf[1] = p2;
  EXPECT_EQ(conf[0], p1);
  EXPECT_EQ(conf[1], p2);
}
