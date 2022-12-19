#pragma once

#include <mrnav/kernels/boost_geom/ik/double.hpp>

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/config.hpp>
#include <mrnav/planner_6124e1a8685f/robot.hpp>
#include <mrnav/planner_6124e1a8685f/problem.hpp>
#include <mrnav/planner_6124e1a8685f/path.hpp>
#include <mrnav/planner_6124e1a8685f/extended_path.hpp>
#include <mrnav/planner_6124e1a8685f/simple_slice_tree.hpp>
#include <mrnav/planner_6124e1a8685f/simple_slice.hpp>
#include <mrnav/planner_6124e1a8685f/simple_cell.hpp>
#include <mrnav/planner_6124e1a8685f/compound_cell.hpp>
#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

namespace mrnav::planner_6124e1a8685f
{

class Kernel
{
public:
  using GeometryKernel = ::mrnav::kernels::boost_geom::Kernel_double;
};

extern template
class Planner< Kernel >;

using PlannerBGD = Planner< Kernel >;

extern template
class stateful_ptr< PlannerBGD >;
extern template
class stateful_ptr< PlannerBGD::SimpleSliceTree >;
extern template
class stateful_ptr< PlannerBGD::SimpleSlice >;
extern template
class stateful_ptr< PlannerBGD::SimpleCell >;
extern template
class stateful_ptr< PlannerBGD::CompoundCell >;

} // mrnav::planner_6124e1a8685f::models::boost_geom_double
