#pragma once

#include <mrnav/kernels/boost_geom/rectangle.hpp>

namespace mrnav::kernels::boost_geom
{

template< typename BK >
Rectangle< BK >::Rectangle()
  : Rectangle{ Point<BK>{0, 0}, Point<BK>{0, 0} }
{}

template< typename BK >
Rectangle< BK >::Rectangle(CoordNT x_min, CoordNT x_max,
                           CoordNT y_min, CoordNT y_max)
  : Rectangle{ Point<BK>{x_min, y_min}, Point<BK>{x_max, y_max} }
{
  // gtl::model::box< gtl::model::d2::point_xy< double > > foo;
  // foo.
}

template< typename BK >
Rectangle< BK >::Rectangle(const BoostBox &box)
  : _repr{box}
{}

template< typename BK >
Rectangle< BK >::Rectangle(const Point<BK> &bot_left,
                           const Point<BK> &top_right)
  : _repr{ bot_left._repr, top_right._repr }
{}


template< typename BK >
Point< BK >
Rectangle< BK >::get_min_corner() const
{
  return Point< BK >{ this->_repr.min_corner() };
}

template< typename BK >
Point< BK >
Rectangle< BK >::get_max_corner() const
{
  return Point< BK >{ this->_repr.max_corner() };
}

template< typename BK >
typename Rectangle< BK >::CoordNT
Rectangle< BK >::get_xmin() const
{
  return this->_repr.min_corner().x();
}

template< typename BK >
typename Rectangle< BK >::CoordNT
Rectangle< BK >::get_xmax() const
{
  return this->_repr.max_corner().x();
}

template< typename BK >
typename Rectangle< BK >::CoordNT
Rectangle< BK >::get_ymin() const
{
  return this->_repr.min_corner().y();
}

template< typename BK >
typename Rectangle< BK >::CoordNT
Rectangle< BK >::get_ymax() const
{
  return this->_repr.max_corner().y();
}

} // mrnav::kernel::boost_geom
