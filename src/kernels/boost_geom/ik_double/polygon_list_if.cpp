#include <mrnav/kernels/boost_geom/ik/double.hpp>
#include <mrnav/kernels/boost_geom/polygon_list_if_impl.hpp>

namespace mrnav::kernels::boost_geom
{

template class PolygonListIF<
  BaseKernel_double,
  BaseKernel_double::BoostPolygon::inner_container_type >;
template class PolygonListIF<
  BaseKernel_double,
  const BaseKernel_double::BoostPolygon::inner_container_type >;

} // mrnav::kernels::boost_geom
