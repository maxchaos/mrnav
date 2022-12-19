#include <mrnav/planner_6124e1a8685f/models/boost_geom_double.hpp>

#include <mrnav/planner_6124e1a8685f/config_impl.hpp>
#include <mrnav/planner_6124e1a8685f/robot_impl.hpp>
#include <mrnav/planner_6124e1a8685f/problem_impl.hpp>
#include <mrnav/planner_6124e1a8685f/path_impl.hpp>
#include <mrnav/planner_6124e1a8685f/extended_path_impl.hpp>
#include <mrnav/planner_6124e1a8685f/simple_slice_tree_impl.hpp>
#include <mrnav/planner_6124e1a8685f/simple_slice_impl.hpp>
#include <mrnav/planner_6124e1a8685f/simple_cell_impl.hpp>
#include <mrnav/planner_6124e1a8685f/compound_cell_impl.hpp>
#include <mrnav/planner_6124e1a8685f/planner_impl.hpp>
#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr_impl.hpp>

namespace mrnav::planner_6124e1a8685f
{

template
class Planner< Kernel >;

template
class stateful_ptr< PlannerBGD >;
template
class stateful_ptr< PlannerBGD::SimpleSliceTree >;
template
class stateful_ptr< PlannerBGD::SimpleSlice >;
template
class stateful_ptr< PlannerBGD::SimpleCell >;
template
class stateful_ptr< PlannerBGD::CompoundCell >;

} // mrnav::planner_6124e1a8685f
