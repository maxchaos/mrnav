#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>

#include <tinyxml2.h>
#include <cstring>

namespace mrnav::kernels::boost_geom {

template< typename BaseKernel_, typename BoostRepr_ >
class PolygonRegionIF
{
public:
  using BaseKernel = BaseKernel_;
  using BoostRepr = typename std::remove_cv_t< BoostRepr_ > ;
  using RingIF = PolygonIF<
    BaseKernel,
    typename BoostRepr::ring_type >;
  using RingListIF = PolygonListIF<
    BaseKernel,
    typename BoostRepr::inner_container_type >;

private:
  BoostRepr *_repr;
  mutable std::optional< bool > _cache_simple_p;
  mutable std::optional< bool > _cache_valid_p;

public:
  PolygonRegionIF(BoostRepr *ptr);
  PolygonRegionIF(const BoostRepr *ptr);

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
  bool simple_p() const;
  bool empty_p() const;
  bool valid_p() const;

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

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "polygon-region");

  void
  from_xml_element(tinyxml2::XMLElement *elt);

};

} // mrnav::kernels::boost_geom
