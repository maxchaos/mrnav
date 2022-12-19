#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>

namespace mrnav::kernels::boost_geom
{

template< typename BaseKernel_ >
class Rectangle
{
public:
  using BaseKernel = BaseKernel_;
  using CoordNT = typename BaseKernel::CoordNT;
  using BoostBox = typename BaseKernel::BoostBox;

private:
  BoostBox _repr;

public:
  Rectangle();
  Rectangle(CoordNT x_min, CoordNT x_max, CoordNT y_min, CoordNT y_max);
  Rectangle(const BoostBox &box);

private:
  Rectangle(const Point<BaseKernel> &bot_left,
            const Point<BaseKernel> &top_right);

public:
  Point< BaseKernel > get_min_corner() const;
  // void set_min_corner(const Point< BaseKernel > &val);
  Point< BaseKernel > get_max_corner() const;
  // void set_max_corner(const Point< BaseKernel > &val);

  CoordNT get_xmin() const;
  CoordNT get_xmax() const;
  CoordNT get_ymin() const;
  CoordNT get_ymax() const;

public:
  friend class Polygon< BaseKernel >;
  friend class PolygonRegion< BaseKernel >;
  friend class PolygonRegionSet< BaseKernel >;

};

} // mrnav::kernel::boost_geom
