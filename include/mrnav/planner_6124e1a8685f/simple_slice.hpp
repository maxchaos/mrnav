#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class BP >
class Planner< BP >::SimpleSlice
{
public:
  using Planner_ = Planner< BP >;

  using SimpleSliceTree = typename Planner_::SimpleSliceTree;
  using SimpleSliceTreePtr = stateful_ptr< SimpleSliceTree >;

  using SimpleCell = typename Planner_::SimpleCell;
  using SimpleCellSP = typename Planner_::SimpleCellSP;

  using Robot = typename Planner_::Robot;

  using CoordNT = typename Planner_::CoordNT;
  using DistNT = typename Planner_::DistNT;
  using Rectangle = typename Planner_::Rectangle;

  using SimpleSliceSP = typename Planner_::SimpleSliceSP;
  using SimpleSlicePtr = stateful_ptr< SimpleSlice >;

private:
  SimpleSlicePtr _self;

private:
  SimpleSliceTreePtr _tree;
  SimpleSlicePtr _parent;
  Rectangle _rect_ideal;
  Rectangle _rect_aug;
  bool _expanded_p;
  std::array< SimpleSliceSP, 4 > _children;
  std::vector< SimpleCellSP > _scells;

private:
  SimpleSlice(SimpleSliceTreePtr tree,
              SimpleSlicePtr parent,
              Rectangle rectangle);

public:
  ~SimpleSlice();

public:
  DistNT get_width() const;
  DistNT get_height() const;
  DistNT get_largest_dimension() const;
  DistNT get_smallest_dimension() const;
  DistNT get_area() const;

  bool expanded_p() const;

  const Robot& get_robot() const;

private:
  void _expand();

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "simple-slice") const;

public:
  friend Planner_;
  friend SimpleSliceTree;

};

} // mrnav::planner_6124e1a8685f
