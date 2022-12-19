#include <mrnav/kernels/boost_geom/ik/double.hpp>
#include <mrnav/kernels/boost_geom/polygon_region_if_impl.hpp>

namespace mrnav::kernels::boost_geom
{

template class PolygonRegionIF<
  BaseKernel_double,
  BaseKernel_double::BoostPolygon >;

template class PolygonRegionIF<
  BaseKernel_double,
  const BaseKernel_double::BoostPolygon >;

} // mrnav::kernels::boost_geom
