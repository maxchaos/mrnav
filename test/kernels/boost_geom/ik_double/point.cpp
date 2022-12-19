#include <gtest/gtest.h>

#include <mrnav/kernels/boost_geom/ik/double.hpp>

using kern = mrnav::kernels::boost_geom::Kernel_double;

TEST(point, constructor)
{
  kern::Point p_default{};
  kern::Point p1{1.0, 2.0};
  EXPECT_EQ(p1.x(), 1.0);
  EXPECT_EQ(p1.y(), 2.0);
}

TEST(point, assignment)
{
  kern::Point p1{1.0, 2.0};
  p1.x(-1.0);
  p1.y(-2.0);
  EXPECT_EQ(p1.x(), -1.0);
  EXPECT_EQ(p1.y(), -2.0);
}
