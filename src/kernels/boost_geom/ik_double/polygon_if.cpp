#include <mrnav/kernels/boost_geom/ik/double.hpp>
#include <mrnav/kernels/boost_geom/polygon_if_impl.hpp>

namespace mrnav::kernels::boost_geom
{

template class PolygonIF<
  BaseKernel_double,
  BaseKernel_double::BoostLineString >;
template class PolygonIF<
  BaseKernel_double,
  const BaseKernel_double::BoostLineString >;
template class PolygonIF<
  BaseKernel_double,
  BaseKernel_double::BoostPolygon::ring_type >;
template class PolygonIF<
  BaseKernel_double,
  const BaseKernel_double::BoostPolygon::ring_type >;

} // mrnav::kernels::boost_geom
