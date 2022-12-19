#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>

namespace mrnav::kernels::boost_geom
{

template< typename BaseKernel_ >
class Point
{
public:
  using BaseKernel = BaseKernel_;
  using CoordNT = typename BaseKernel::CoordNT;
  using BoostPoint = typename BaseKernel::BoostPoint;

private:
  BoostPoint _repr;

public:
  Point();
  Point(CoordNT x, CoordNT y);
  Point(const BoostPoint &p);

public:
  CoordNT x() const;
  CoordNT y() const;
  void x(CoordNT val);
  void y(CoordNT val);

public:
  template< typename BK >
  friend bool operator==(const Point< BK >& l, const Point< BK >& r);
  template< typename BK >
  friend bool operator!=(const Point< BK >& l, const Point< BK >& r);

public:
  friend class Polygon< BaseKernel >;
  friend class PolygonRegion< BaseKernel >;
  friend class PolygonRegionSet< BaseKernel >;
  friend class Rectangle< BaseKernel >;

  template< typename BK, typename RT >
  friend class PolygonIF;

  template< typename BK, typename RT >
  friend class PolygonListIF;

  template< typename BK, typename RT >
  friend class PolygonRegionIF;

};

template< typename BK >
bool operator==(const Point< BK >& l, const Point< BK >& r);

template< typename BK >
bool operator!=(const Point< BK >& l, const Point< BK >& r);

} // mrnav::kernels::boost_geom
