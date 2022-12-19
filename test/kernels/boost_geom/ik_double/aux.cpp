#include <gtest/gtest.h>

#include <mrnav/kernels/boost_geom/ik/double.hpp>

using kern = mrnav::kernels::boost_geom::Kernel_double;

TEST(aux, buffer)
{
  kern::BaseKernel::BufferStrategy builder{1.0};
  const auto bck =
    kern::BaseKernel::BufferStrategy::get_delta_arc_lenght();
  kern::BaseKernel::BufferStrategy::set_delta_arc_lenght(2 * bck + 1);
  EXPECT_DOUBLE_EQ(
    kern::BaseKernel::BufferStrategy::get_delta_arc_lenght(),
    2*bck + 1);
  kern::BaseKernel::BufferStrategy::set_delta_arc_lenght(10 * bck + 7);
  EXPECT_DOUBLE_EQ(
    kern::BaseKernel::BufferStrategy::get_delta_arc_lenght(),
    10 * bck + 7);
  kern::BaseKernel::BufferStrategy::set_delta_arc_lenght(23.0);
  EXPECT_DOUBLE_EQ(
    kern::BaseKernel::BufferStrategy::get_delta_arc_lenght(),
    23.0);
  kern::BaseKernel::BufferStrategy::set_delta_arc_lenght(17.0);
  EXPECT_DOUBLE_EQ(
    kern::BaseKernel::BufferStrategy::get_delta_arc_lenght(),
    17.0);
  kern::BaseKernel::BufferStrategy::set_delta_arc_lenght(bck);
}
