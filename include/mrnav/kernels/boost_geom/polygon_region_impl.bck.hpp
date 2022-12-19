#pragma once

#include <mrnav/kernels/boost_geom/polygon_region.hpp>

#include <iterator>

namespace mrnav::kernels::boost_geom
{

template< typename BK >
PolygonRegion< BK >::PolygonRegion()
  : _repr{}
  , _cache_simple_p{}
  , _cache_valid_p{}
{}

template< typename BK >
const typename PolygonRegion< BK >::RingIF
PolygonRegion< BK >::get_outer()
  const
{
  return RingIF{ &(this->_repr.outer()) };
}

template< typename BK >
typename PolygonRegion< BK >::RingIF
PolygonRegion< BK >::get_outer()
{
  // Invalidate cache.
  this->invalidate_cache();
  // Return interface to ring.
  return RingIF{ &(this->_repr.outer()) };
}

template< typename BK >
void
PolygonRegion< BK >::set_outer(PolygonCSP< BaseKernel > pgn)
{
  if(pgn == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  this->set_outer(*pgn);
}

template< typename BK >
void
PolygonRegion< BK >::set_outer(const Polygon< BaseKernel > &pgn)
{
  // ::mrnav::kernels::boost_geom::BaseKernel< double, false >::BoostPolygon::ring_type foo;
  // foo.insert()
  auto &outer = this->_repr.outer();
  outer.clear();
  outer.insert(outer.begin(), pgn._repr.cbegin(), pgn._repr.cend());
  // Invalidate cache.
  this->invalidate_cache();
}



template< typename BK >
const typename PolygonRegion< BK >::RingListIF
PolygonRegion< BK >::get_inners()
  const
{
  return RingListIF( &(this->_repr.inners()) );
}

template< typename BK >
typename PolygonRegion< BK >::RingListIF
PolygonRegion< BK >::get_inners()
{
  // Invalidate cache.
  this->invalidate_cache();
  // Return interface to ring list.
  return RingListIF( &(this->_repr.inners()) );
}

template< typename BK >
template< typename InputIterator >
std::enable_if_t<
  std::is_same_v< typename std::iterator_traits< InputIterator >::value_type,
                  PolygonCSP >,
  void >
PolygonRegion< BK >::set_inners(InputIterator begin, InputIterator end)
{
  auto &ringlist = this->_repr.inners();
  ringlist.clear();
  for(auto it = begin; it != end; it++) {
    if(*it != nullptr)
      ringlist.push_back((*it)->_repr);
  }
  // Invalidate cache.
  this->invalidate_cache();
}

template< typename BK >
template< typename InputIterator >
std::enable_if_t<
  std::is_same_v< typename std::iterator_traits< InputIterator >::value_type,
                  Polygon >,
  void >
PolygonRegion< BK >::set_inners(InputIterator begin, InputIterator end)
{
  auto &ringlist = this->_repr.inners();
  ringlist.clear();
  for(auto it = begin; it != end; it++) {
    ringlist.push_back(it->_repr);
  }
  // Invalidate cache.
  this->invalidate_cache();
}



template< typename BK >
bool
PolygonRegion< BK >::simple_p()
{
  if(not this->_cache_simple_p)
    this->_cache_simple_p = gtl::is_simple(this->_repr);
  return this->_cache_simple_p.value();
}

template< typename BK >
bool
PolygonRegion< BK >::valid_p()
{
  if(not this->_cache_valid_p)
    this->_cache_valid_p = gtl::is_valid(this->_repr);
  return this->_cache_valid_p.value();
}


template< typename BK >
bool
PolygonRegion< BK >::contains_p(PointCSP< BK > p)
  const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point in null");
  return this->contains_p(*p);
}

template< typename BK >
bool
PolygonRegion< BK >::contains_p(const Point< BK > &p)
  const
{
  return gtl::within(p._repr, this->_repr);
}



template< typename BK >
void
PolygonRegion< BK >::invalidate_cache()
{
  this->_cache_valid_p.reset();
  this->_cache_valid_p.reset();
}

} // mrnav::kernels::boost_geom
