#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>
#include <mrnav/kernels/boost_geom/polygon_region_if.hpp>
#include <optional>

namespace mrnav::kernels::boost_geom
{

template< typename BaseKernel_ >
class PolygonRegion :
    public PolygonRegionIF< BaseKernel_,
                            typename BaseKernel_::BoostPolygon >
{
public:
  using BaseKernel = BaseKernel_;
  using Interface = PolygonRegionIF< BaseKernel,
                                     typename BaseKernel::BoostPolygon >;

private:
  using BoostPolygon = typename BaseKernel::BoostPolygon;

public:
  PolygonRegion();
  PolygonRegion(const PolygonRegion &other);

private:
  BoostPolygon _repr;

public:
  Interface get_if();
  const Interface get_if() const;

public:
  friend class Polygon< BaseKernel >;
  friend class PolygonRegionSet< BaseKernel >;

  template< typename BK, typename RT >
  friend class PolygonIF;

  template< typename BK, typename RT >
  friend class PolygonListIF;

  template< typename BK, typename RT >
  friend class PolygonRegionIF;

};

} // mrnav::kernels::boost_geom
