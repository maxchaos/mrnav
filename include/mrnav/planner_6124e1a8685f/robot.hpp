#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
class Planner< Kernel >::Robot
{
public:
  using Planner_ = Planner< Kernel >;
  using CoordNT = typename Planner_::CoordNT;
  using DistNT = typename Planner_::DistNT;
  using Point = typename Planner_::GeometryKernel::Point;
  using PointSP = typename Planner_::GeometryKernel::PointSP;
  using PointCSP = typename Planner_::GeometryKernel::PointCSP;
  using Polygon = typename Planner_::GeometryKernel::Polygon;
  using PolygonSP = typename Planner_::GeometryKernel::PolygonSP;

private:
  DistNT _radius;

private:
  Polygon _body;

public:
  Robot(DistNT radius = 0, size_t amount_of_vertices = 32);

public:
  void set_radius(DistNT val, size_t amount_of_vertices = 32);
  DistNT get_radius() const;

public:
  PolygonSP get_body() const;
  PolygonSP get_body(CoordNT cx, CoordNT cy) const;
  PolygonSP get_body(PointCSP p) const;
  PolygonSP get_body(const Point &p) const;

public:
  void recompute_body(const size_t amount_of_vertices);

public:
  friend Planner_;

};

} // mrnav::planner_6124e1a8685f
