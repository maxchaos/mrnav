#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>
#include <mrnav/kernels/boost_geom/point.hpp>
#include <mrnav/kernels/boost_geom/polygon_if.hpp>

#include <optional>

namespace mrnav::kernels::boost_geom
{

template< typename BaseKernel_ >
class Polygon :
    public PolygonIF< BaseKernel_, typename BaseKernel_::BoostRing >
{
public:
  using BaseKernel = BaseKernel_;

private:
  using BoostRing = typename BaseKernel::BoostRing;
  using BoostLineString = typename BaseKernel::BoostLineString;
  using BoostPolygon = typename BaseKernel::BoostPolygon;

private:
  BoostRing _repr;

public:
  Polygon();
  Polygon(const Polygon& other);
  // Polygon(std::vector< PointCSP<BaseKernel> > vertices);
  // Polygon(const std::vector< Point<BaseKernel> > &vertices);
  // Polygon(const std::vector< const Point<BaseKernel>* > &vertices);
  Polygon(const BoostRing &o);
  Polygon(const BoostLineString &o);
  // Polygon(const typename BoostPolygon::ring_type &o);

public:
  PolygonIF< BaseKernel, typename BaseKernel::BoostRing >
  get_if();

public:
  friend class PolygonRegion< BaseKernel >;
  friend class PolygonRegionSet< BaseKernel >;

  template< typename BK, typename RT >
  friend class PolygonIF;

  template< typename BK, typename RT >
  friend class PolygonListIF;

  template< typename BK, typename RT >
  friend class PolygonRegionIF;

};

} // mrnav::kernels::boost_geom
