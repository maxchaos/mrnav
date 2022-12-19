#pragma once

#include <mrnav/kernels/boost_geom/polygon.hpp>

namespace mrnav::kernels::boost_geom {

template< typename BK >
Polygon< BK >::Polygon()
  : PolygonIF< BaseKernel, BoostRing >(&_repr)
  , _repr{}
{}

template< typename BK >
Polygon< BK >::Polygon(const Polygon& other)
  : PolygonIF< BaseKernel, BoostRing >(&_repr)
  , _repr{other._repr}
{}

template< typename BK >
Polygon< BK >::Polygon(const BoostRing &o)
  : PolygonIF< BaseKernel, BoostRing >(&_repr)
  , _repr{o}
{}

template< typename BK >
Polygon< BK >::Polygon(const BoostLineString &o)
  : PolygonIF< BaseKernel, BoostRing >(&_repr)
  , _repr{}
{
  // gtl::model::ring< gtl::model::d2::point_xy<double> > foo;
  _repr.assign(o.cbegin(), o.cend());
}

// template< typename BK >
// Polygon< BK >::Polygon(const typename BoostPolygon::ring_type &o)
//   : PolygonIF< BaseKernel, BoostRing >(&_repr)
//   , _repr{o.cbegin(), o.cend()}
// {}

template< typename BK >
PolygonIF< BK, typename BK::BoostRing >
Polygon< BK >::get_if()
{
  return PolygonIF< BK, typename BK::BoostRing >(&(this->_repr));
}

} // mrnav::kernels::boost_geom
