#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>

#include <mrnav/kernels/boost_geom/point.hpp>
// #include <mrnav/kernels/boost_geom/point_impl.hpp>
#include <mrnav/kernels/boost_geom/polygon.hpp>
// #include <mrnav/kernels/boost_geom/polygon_impl.hpp>
#include <mrnav/kernels/boost_geom/polygon_region.hpp>
// #include <mrnav/kernels/boost_geom/polygon_region_impl.hpp>
#include <mrnav/kernels/boost_geom/polygon_region_set.hpp>
// #include <mrnav/kernels/boost_geom/polygon_region_set_impl.hpp>
#include <mrnav/kernels/boost_geom/polygon_if.hpp>
// #include <mrnav/kernels/boost_geom/polygon_if_impl.hpp>
#include <mrnav/kernels/boost_geom/polygon_list_if.hpp>
// #include <mrnav/kernels/boost_geom/polygon_list_if_impl.hpp>
#include <mrnav/kernels/boost_geom/polygon_region_if.hpp>
// #include <mrnav/kernels/boost_geom/polygon_region_if_impl.hpp>
#include <mrnav/kernels/boost_geom/rectangle.hpp>
// #include <mrnav/kernels/boost_geom/rectangle_impl.hpp>

#include <mrnav/kernels/boost_geom/aux/buffer.hpp>

namespace mrnav::kernels::boost_geom
{

extern template
class aux::buffer::FixedArcLength< double >;
extern template
class aux::buffer::FixedPointsPerCircle< double >;

typedef
BaseKernel< double >
BaseKernel_double;
typedef Kernel< BaseKernel_double > Kernel_double;

extern template class BaseKernel< double >;
extern template class Kernel< BaseKernel_double >;

extern template class Point< BaseKernel_double >;
extern template class Polygon< BaseKernel_double >;
extern template class PolygonRegion< BaseKernel_double >;
extern template class PolygonRegionSet< BaseKernel_double >;

extern template class PolygonIF<
  BaseKernel_double,
  BaseKernel_double::BoostLineString >;
extern template class PolygonIF<
  BaseKernel_double,
  const BaseKernel_double::BoostLineString >;
extern template class PolygonIF<
  BaseKernel_double,
  BaseKernel_double::BoostPolygon::ring_type >;
extern template class PolygonIF<
  BaseKernel_double,
  const BaseKernel_double::BoostPolygon::ring_type >;

extern template class PolygonListIF<
  BaseKernel_double,
  BaseKernel_double::BoostPolygon::inner_container_type >;
extern template class PolygonListIF<
  BaseKernel_double,
  const BaseKernel_double::BoostPolygon::inner_container_type >;

extern template class PolygonRegionIF<
  BaseKernel_double,
  BaseKernel_double::BoostPolygon >;
extern template class PolygonRegionIF<
  BaseKernel_double,
  const BaseKernel_double::BoostPolygon >;

extern template class Rectangle< BaseKernel_double >;

extern template
bool
operator==(const Point< BaseKernel_double >& l,
           const Point< BaseKernel_double >& r);

extern template
bool
operator!=(const Point< BaseKernel_double >& l,
           const Point< BaseKernel_double >& r);

} // mrnav::kernels::boost_geom
