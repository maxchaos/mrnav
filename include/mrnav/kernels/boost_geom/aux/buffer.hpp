#pragma once

#include <cmath>
#include <type_traits>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace mrnav::kernels::boost_geom::aux::buffer
{

template< typename CoordNT_, typename DistNT_ = CoordNT_ >
class FixedArcLength
{

public:
  using CoordNT = CoordNT_;
  using DistNT = DistNT_;

private:
  static DistNT _delta_arc_length;

public:
  static void set_delta_arc_lenght(DistNT len) {
    _delta_arc_length = len;
  }
  static DistNT get_delta_arc_lenght() {
    return _delta_arc_length;
  }

private:
  size_t _points_per_circle;
  boost::geometry::strategy::buffer::distance_symmetric< CoordNT >
  _stg_distance;
  boost::geometry::strategy::buffer::join_round
  _stg_join;
  boost::geometry::strategy::buffer::end_round
  _stg_end;
  boost::geometry::strategy::buffer::point_circle
  _stg_circle;
  boost::geometry::strategy::buffer::side_straight
  _stg_side;

public:

  FixedArcLength(CoordNT buffer)
    : _points_per_circle{static_cast<size_t>(
      std::ceil( (2 * M_PI * buffer) / _delta_arc_length ))}
    , _stg_distance{buffer}
    , _stg_join{_points_per_circle}
    , _stg_end{_points_per_circle}
    , _stg_circle{_points_per_circle}
    , _stg_side{}
  {}

public:

  size_t points_per_circle() { return this->_points_per_circle; }

  const
  boost::geometry::strategy::buffer::distance_symmetric< CoordNT >&
  stg_distance() {
    return this->_stg_distance;
  }

  const
  boost::geometry::strategy::buffer::join_round&
  stg_join() {
    return this->_stg_join;
  }

  const
  boost::geometry::strategy::buffer::end_round&
  stg_end() {
    return this->_stg_end;
  }

  const
  boost::geometry::strategy::buffer::point_circle&
  stg_circle() {
    return this->_stg_circle;
  }

  const
  boost::geometry::strategy::buffer::side_straight&
  stg_side() {
    return this->_stg_side;
  }

};





template< typename CoordNT_, typename DistNT_ = CoordNT_ >
class FixedPointsPerCircle
{

public:
  using CoordNT = CoordNT_;
  using DistNT = DistNT_;

private:
  static size_t _points_per_circle;

public:
  static void set_points_per_circle(size_t amount) {
    _points_per_circle = amount;
  }
  static DistNT get_points_per_circle() {
    return _points_per_circle;
  }

private:
  boost::geometry::strategy::buffer::distance_symmetric< CoordNT >
  _stg_distance;
  boost::geometry::strategy::buffer::join_round
  _stg_join;
  boost::geometry::strategy::buffer::end_round
  _stg_end;
  boost::geometry::strategy::buffer::point_circle
  _stg_circle;
  boost::geometry::strategy::buffer::side_straight
  _stg_side;

public:

  FixedPointsPerCircle(CoordNT buffer)
    : _stg_distance{buffer}
    , _stg_join{_points_per_circle}
    , _stg_end{_points_per_circle}
    , _stg_circle{_points_per_circle}
    , _stg_side{}
  {}

public:

  const
  boost::geometry::strategy::buffer::distance_symmetric< CoordNT >&
  stg_distance() {
    return this->_stg_distance;
  }

  const
  boost::geometry::strategy::buffer::join_round&
  stg_join() {
    return this->_stg_join;
  }

  const
  boost::geometry::strategy::buffer::end_round&
  stg_end() {
    return this->_stg_end;
  }

  const
  boost::geometry::strategy::buffer::point_circle&
  stg_circle() {
    return this->_stg_circle;
  }

  const
  boost::geometry::strategy::buffer::side_straight&
  stg_side() {
    return this->_stg_side;
  }

};

}
