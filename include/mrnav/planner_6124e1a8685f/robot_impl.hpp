#pragma once

#include <mrnav/planner_6124e1a8685f/robot.hpp>

#include <cmath>
#include <cassert>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
Planner< Kernel >::Robot::Robot(DistNT radius, size_t amount_of_vertices)
  : _radius{radius}
  , _body{}
{
  assert(amount_of_vertices > 2);
  this->recompute_body(amount_of_vertices);
}

template< class Kernel >
typename Planner< Kernel >::Robot::DistNT
Planner< Kernel >::Robot::get_radius() const
{
  return this->_radius;
}

template< class Kernel >
void
Planner< Kernel >::Robot::set_radius(DistNT val, size_t amount_of_vertices)
{
  this->_radius = val;
  this->recompute_body(amount_of_vertices);
}

template< class Kernel >
typename Planner< Kernel >::Robot::PolygonSP
Planner< Kernel >::Robot::get_body(CoordNT cx, CoordNT cy)
  const
{
  return this->get_body(Point{cx, cy});
}

template< class Kernel >
typename Planner< Kernel >::Robot::PolygonSP
Planner< Kernel >::Robot::get_body(PointCSP p)
  const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  return this->get_body(*p);
}

template< class Kernel >
typename Planner< Kernel >::Robot::PolygonSP
Planner< Kernel >::Robot::get_body(const Point &p)
  const
{
  return this->_body.get_translated(p);
}

template< class Kernel >
void
Planner< Kernel >::Robot::recompute_body(const size_t amount_of_vertices)
{
  double r = this->_radius;
  double theta = 0.0;
  double dtheta = 2 * M_PI / (amount_of_vertices - 1);
  this->_body.clear();
  for(size_t k = 0; k < amount_of_vertices-1; k++)
    {
      this->_body.append_vtx(Point{r*cos(theta), r*sin(theta)});
      theta += dtheta;
    }
  // this->_body.reverse();
}

} // mrnav::planner_6124e1a8685f
