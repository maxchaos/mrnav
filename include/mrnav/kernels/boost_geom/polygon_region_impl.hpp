#pragma once

#include <mrnav/kernels/boost_geom/polygon_region.hpp>

#include <iterator>

namespace mrnav::kernels::boost_geom
{

template< typename BK >
PolygonRegion< BK >::PolygonRegion()
  : PolygonRegionIF< BaseKernel, BoostPolygon >{&_repr}
  , _repr{}
{}

template< typename BK >
PolygonRegion< BK >::PolygonRegion(const PolygonRegion &other)
  : PolygonRegionIF< BaseKernel, BoostPolygon >{&_repr}
  , _repr{other._repr}
{}

template< typename BK >
typename  PolygonRegion< BK >::Interface
PolygonRegion< BK >::get_if()
{
  return PolygonRegionIF< BK, typename BK::BoostPolygon >(&(this->_repr));
}

template< typename BK >
const typename PolygonRegion< BK >::Interface
PolygonRegion< BK >::get_if() const
{
  return PolygonRegionIF< BK, typename BK::BoostPolygon >(&(this->_repr));
}

} // mrnav::kernels::boost_geom
