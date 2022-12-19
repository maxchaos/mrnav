#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class BasicParameters_ >
class Planner< BasicParameters_ >::SimpleSliceTree
{

public:
  using Planner_ = Planner< BasicParameters_ >;
  using PlannerPtr = typename Planner_::PlannerPtr;

  using PolygonRegionSet =
    typename Planner_::GeometryKernel::PolygonRegionSet;
  using PolygonRegionSetSP =
    typename Planner_::GeometryKernel::PolygonRegionSetSP;
  using Rectangle = typename Planner_::Rectangle;

  using SimpleSlice = typename Planner_::SimpleSlice;
  using SimpleSliceSP = typename Planner_::SimpleSliceSP;
  using Robot = typename Planner_::Robot;

  using SimpleSliceTreePtr = stateful_ptr< SimpleSliceTree >;

private:
  SimpleSliceTreePtr _self;

private:
  PlannerPtr _planner;
  size_t _robot_idx;
  PolygonRegionSetSP _ws_aug;
  SimpleSliceSP _root;

private:
  SimpleSliceTree(PlannerPtr planner, size_t robot);

public:
  ~SimpleSliceTree();

public:
  const Robot& get_robot() const;
  const PolygonRegionSet& get_ws_aug() const;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "simple-hierarchy") const;

public:
  friend Planner_;
  friend SimpleSlice;

};

} // mrnav::planner_6124e1a8685f
