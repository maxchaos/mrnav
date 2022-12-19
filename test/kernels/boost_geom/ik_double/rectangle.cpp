#include <gtest/gtest.h>

#include <mrnav/kernels/boost_geom/ik/double.hpp>

using kern = mrnav::kernels::boost_geom::Kernel_double;

TEST(rectangle, simple1)
{
  kern::Rectangle r1{0, 1, 2, 3};
  EXPECT_DOUBLE_EQ(r1.get_xmin(), 0);
  EXPECT_DOUBLE_EQ(r1.get_xmax(), 1);
  EXPECT_DOUBLE_EQ(r1.get_ymin(), 2);
  EXPECT_DOUBLE_EQ(r1.get_ymax(), 3);
  kern::Point min_corner = r1.get_min_corner();
  kern::Point max_corner = r1.get_max_corner();
  EXPECT_DOUBLE_EQ(min_corner.x(), 0);
  EXPECT_DOUBLE_EQ(min_corner.y(), 2);
  EXPECT_DOUBLE_EQ(max_corner.x(), 1);
  EXPECT_DOUBLE_EQ(max_corner.y(), 3);
}
