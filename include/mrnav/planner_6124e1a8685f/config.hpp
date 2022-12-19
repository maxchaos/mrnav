#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
class Planner< Kernel >::Configuration
{
public:
  using Planner_ = Planner< Kernel >;
  using Point = typename Planner_::GeometryKernel::Point;
  using PointSP = typename Planner_::GeometryKernel::PointSP;
  using PointCSP = typename Planner_::GeometryKernel::PointCSP;

private:
  std::vector< Point > _pos;

private:
  Configuration();

public:
  Configuration(const size_t size);

  Point& get(size_t idx);
  const Point& get(size_t idx) const;

  void set(size_t idx, PointSP p);
  void set(size_t idx, PointCSP p);
  void set(size_t idx, const Point &p);

  size_t size() const;

public:
  Point& operator[](size_t idx);
  const Point& operator[](size_t idx) const;

};

} // mrnav::planner_6124e1a8685f
