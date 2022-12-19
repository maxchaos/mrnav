#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>
#include <optional>

namespace mrnav::kernels::boost_geom
{

template< typename BaseKernel_ >
class PolygonRegion
{
public:
  using BaseKernel = BaseKernel_;
  using CoordNT = typename BaseKernel::CoordNT;
  using RingIF = PolygonIF<
    BaseKernel,
    typename BaseKernel::BoostPolygon::ring_type >;
  using RingListIF = PolygonListIF<
    BaseKernel,
    typename BaseKernel::BoostPolygon::inner_container_type >;

private:
  using BoostPolygon = typename BaseKernel::BoostPolygon;

private:
  BoostPolygon _repr;
  mutable std::optional< bool > _cache_simple_p;
  mutable std::optional< bool > _cache_valid_p;

public:
  PolygonRegion();

public:
  const RingIF get_outer() const;
  RingIF get_outer();
  void set_outer(PolygonCSP< BaseKernel > pgn);
  void set_outer(const Polygon< BaseKernel > &pgn);

public:
  const RingListIF get_inners() const;
  RingListIF get_inners();

  template< typename InputIterator >
  std::enable_if_t<
    std::is_same_v< typename std::iterator_traits< InputIterator >::value_type,
                    PolygonCSP >,
    void >
  set_inners(InputIterator begin, InputIterator end);

  template< typename InputIterator >
  std::enable_if_t<
    std::is_same_v< typename std::iterator_traits< InputIterator >::value_type,
                    Polygon >,
    void >
  set_inners(InputIterator begin, InputIterator end);

public:
  bool simple_p();
  bool valid_p();

public:
  bool contains_p(PointCSP< BaseKernel > p) const;
  bool contains_p(const Point< BaseKernel > &p) const;

public:
  void invalidate_cache();

public:
  friend class PolygonRegionSet< BaseKernel >;

  template< typename BK, typename RT >
  friend class PolygonIF;

  template< typename BK, typename RT >
  friend class PolygonListIF;

};

} // mrnav::kernels::boost_geom
