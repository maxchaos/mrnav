#pragma once

#include <memory>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <mrnav/kernels/boost_geom/aux/buffer.hpp>

namespace mrnav::kernels::boost_geom
{

namespace gtl = boost::geometry;

template< typename CoordNT_,
          typename DistNT_ = CoordNT_,
          bool ccw_orientation_p_ = true,
          bool closed_p_ = false,
          class BufferStrategy_=
          aux::buffer::FixedPointsPerCircle<CoordNT_, DistNT_> >
          // aux::buffer::FixedArcLength<CoordNT_, DistNT_> >
class BaseKernel
{
public:
  static const bool ccw_orientation_p = ccw_orientation_p_;
  static const bool closed_p = closed_p_;

  using CoordNT = CoordNT_;
  using DistNT = DistNT_;
  using BoostPoint =
    boost::geometry::model::d2::point_xy< CoordNT >;
  using BoostLineString =
    boost::geometry::model::linestring< BoostPoint >;
  using BoostRing =
    boost::geometry::model::ring< BoostPoint,
                                  !ccw_orientation_p,
                                  closed_p >;
  using BoostPolygon =
    boost::geometry::model::polygon< BoostPoint,
                                     !ccw_orientation_p,
                                     closed_p >;
  using BoostMultiPolygon =
    boost::geometry::model::multi_polygon< BoostPolygon >;

  using BoostBox =
    boost::geometry::model::box< BoostPoint >;

  using BufferStrategy = BufferStrategy_;
};

template< typename BaseKernel > class Point;
template< typename BaseKernel > class Polygon;
template< typename BaseKernel > class PolygonRegion;
template< typename BaseKernel > class PolygonRegionSet;
template< typename BaseKernel > class Rectangle;

template< typename BaseKernel, typename RingType >
class PolygonIF;
template< typename BaseKernel, typename RingListType >
class PolygonListIF;
template< typename BaseKernel, typename BoostRepr >
class PolygonRegionIF;

template< typename BaseKernel >
using PointSP =
  std::shared_ptr< Point<BaseKernel> >;

template< typename BaseKernel >
using PointCSP =
  std::shared_ptr< const Point<BaseKernel> >;

template< typename BaseKernel >
using PolygonSP =
  std::shared_ptr< Polygon<BaseKernel> >;

template< typename BaseKernel >
using PolygonCSP =
  std::shared_ptr< const Polygon<BaseKernel> >;

template< typename BaseKernel >
using PolygonRegionSP =
  std::shared_ptr< PolygonRegion<BaseKernel> >;

template< typename BaseKernel >
using PolygonRegionCSP =
  std::shared_ptr< const PolygonRegion<BaseKernel> >;

template< typename BaseKernel >
using PolygonRegionSetSP =
  std::shared_ptr< PolygonRegionSet<BaseKernel> >;

template< typename BaseKernel >
using PolygonRegionSetCSP =
  std::shared_ptr< const PolygonRegionSet<BaseKernel> >;

template< typename BaseKernel >
using RectangleSP =
  std::shared_ptr< Rectangle<BaseKernel> >;

template< typename BaseKernel >
using RectangleCSP =
  std::shared_ptr< const Rectangle<BaseKernel> >;



template< typename BaseKernel_,
          typename CoordNT_ = typename BaseKernel_::CoordNT,
          typename DistNT_ = CoordNT_ >
class Kernel
{
public:
  using BaseKernel = BaseKernel_;
  using CoordNT = CoordNT_;
  using DistNT = DistNT_;
  // ----
  using Point =
    ::mrnav::kernels::boost_geom::Point< BaseKernel >;
  using Polygon =
    ::mrnav::kernels::boost_geom::Polygon< BaseKernel >;
  using PolygonRegion =
    ::mrnav::kernels::boost_geom::PolygonRegion< BaseKernel >;
  using PolygonRegionSet =
    ::mrnav::kernels::boost_geom::PolygonRegionSet< BaseKernel >;
  using Rectangle =
    ::mrnav::kernels::boost_geom::Rectangle< BaseKernel >;
  // ----
  using PointSP =
    ::mrnav::kernels::boost_geom::PointSP< BaseKernel >;
  using PointCSP =
    ::mrnav::kernels::boost_geom::PointCSP< BaseKernel >;
  // ----
  using PolygonSP =
    ::mrnav::kernels::boost_geom::PolygonSP< BaseKernel >;
  using PolygonCSP =
    ::mrnav::kernels::boost_geom::PolygonCSP< BaseKernel >;
  // ----
  using PolygonRegionSP =
    ::mrnav::kernels::boost_geom::PolygonRegionSP< BaseKernel >;
  using PolygonRegionCSP =
    ::mrnav::kernels::boost_geom::PolygonRegionCSP< BaseKernel >;
  // ----
  using PolygonRegionSetSP =
    ::mrnav::kernels::boost_geom::PolygonRegionSetSP< BaseKernel >;
  using PolygonRegionSetCSP =
    ::mrnav::kernels::boost_geom::PolygonRegionSetCSP< BaseKernel >;
};

} // mrnav::kernels::boost_geom
