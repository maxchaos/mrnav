#pragma once

#include <mrnav/kernels/boost_geom/point.hpp>

namespace mrnav::kernels::boost_geom
{

template< typename BK >
Point< BK >::Point()
  : _repr{}
{}

template< typename BK >
Point< BK >::Point(CoordNT x, CoordNT y)
  : _repr{x, y}
{}

template< typename BK >
Point< BK >::Point(const BoostPoint &p)
  : _repr{p}
{}

template< typename BK >
typename BK::CoordNT
Point< BK >::x()
  const
{
  return this->_repr.x();
}

template< typename BK >
typename BK::CoordNT
Point< BK >::y()
  const
{
  return this->_repr.y();
}

template< typename BK >
void
Point< BK >::x(CoordNT val)
{
  this->_repr.x(val);
}

template< typename BK >
void
Point< BK >::y(CoordNT val)
{
  this->_repr.y(val);
}

template< typename BK >
bool
operator==(const Point< BK >& l, const Point< BK >& r)
{
  return (l._repr.x() == r._repr.x()) and (l._repr.y() == r._repr.y());
}

template< typename BK >
bool
operator!=(const Point< BK >& l, const Point< BK >& r)
{
  return not (l == r);
}

} // mrnav::kernels::boost_geom
