#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

#include <vector>

namespace mrnav::planner_6124e1a8685f
{

template< class BasicParameters >
class Planner< BasicParameters >::SimpleCell
{

public:
  using Planner_ = Planner< BasicParameters >;

  using SimpleSlice = typename Planner_::SimpleSlice;
  using SimpleSlicePtr = typename SimpleSlice::SimpleSlicePtr;

  using PolygonRegion =
    typename Planner_::GeometryKernel::PolygonRegion;
  using PolygonRegionSP =
    typename Planner_::GeometryKernel::PolygonRegionSP;
  using PolygonRegionIF =
    typename Planner_::GeometryKernel::PolygonRegion::Interface;

  using PolygonRegionSet =
    typename Planner_::GeometryKernel::PolygonRegionSet;
  using PolygonRegionSetSP =
    typename Planner_::GeometryKernel::PolygonRegionSetSP;

  using Point =
    typename Planner_::GeometryKernel::Point;
  using PointSP =
    typename Planner_::GeometryKernel::PointSP;

  using SimpleCellPtr = stateful_ptr< SimpleCell >;
  using SimpleCellSP = typename Planner_::SimpleCellSP;

private:
  SimpleCellPtr _self;

private:
  SimpleSlicePtr _slice;
  SimpleCellPtr _parent;
  bool _expanded_p;
  std::vector< SimpleCellSP > _children;
  std::vector< SimpleCellPtr > _adjacent;

private:
  PolygonRegionSet _shape;
  PolygonRegionSet _rfpoa;
  PolygonRegionSet _rfpua;

private:
  SimpleCell(SimpleSlicePtr slice,
             SimpleCellPtr parent,
             PolygonRegionSP shape,
             const std::vector< SimpleCellPtr > &_adjacent);

  SimpleCell(SimpleSlicePtr slice,
             SimpleCellPtr parent,
             const PolygonRegion &shape,
             const std::vector< SimpleCellPtr > &_adjacent);

  SimpleCell(SimpleSlicePtr slice,
             SimpleCellPtr parent,
             const PolygonRegionIF &shape,
             const std::vector< SimpleCellPtr > &_adjacent);

public:
  ~SimpleCell();

private:
  void _filter_and_register_adjacent(
    const std::vector< SimpleCellPtr > &adjacent);

  void _compute_rfpoa();
  void _compute_rfpua();

public:
  bool expanded_p() const;
  bool adjacent_p(SimpleCellPtr cand) const;
  bool adjacent_p(SimpleCellSP cand) const;
  bool adjacent_p(SimpleCell const *cand) const;
  bool contains_p(const Point &p) const;

private:
  void _expand();

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "simple-cell") const;

public:
  friend Planner_;
  friend SimpleSliceTree;
  friend SimpleSlice;

};

} // mrnav::planner_6124e1a8685f
