#pragma once

#include <mrnav/kernels/boost_geom/polygon_list_if.hpp>

namespace mrnav::kernels::boost_geom {


template< typename BK, typename RT >
PolygonListIF< BK, RT >::PolygonListIF(BoostRepr *ringlist)
{
  if(ringlist == nullptr)
    throw std::invalid_argument("ring list type pointer is null");
  this->_repr = ringlist;
  // mrnav::kernels::boost_geom::BaseKernel< double >::BoostPolygon::inner_container_type foo;
  // foo.
}

template< typename BK, typename RT >
PolygonListIF< BK, RT >::PolygonListIF(const BoostRepr *ringlist)
{
  if(ringlist == nullptr)
    throw std::invalid_argument("ring list type pointer is null");
  this->_repr = const_cast<BoostRepr*>(ringlist);
  // mrnav::kernels::boost_geom::BaseKernel< double >::BoostPolygon::inner_container_type foo;
  // foo.
}



template< typename BK, typename RT >
size_t
PolygonListIF< BK, RT >::size()
  const
{
  return this->_repr->size();
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::resize(size_t new_size)
{
  this->_repr->resize(new_size);
}



template< typename BK, typename RT >
const typename PolygonListIF< BK, RT >::RingIF
PolygonListIF< BK, RT >::get_ring(size_t idx)
  const
{
  return RingIF{ &(this->_repr->at(idx)) };
}

template< typename BK, typename RT >
typename PolygonListIF< BK, RT >::RingIF
PolygonListIF< BK, RT >::get_ring(size_t idx)
{
  return RingIF{ &(this->_repr->at(idx)) };
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::set_ring(size_t idx,
                                     PolygonCSP< BaseKernel > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  this->set_ring(idx, *p);
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::set_ring(size_t idx,
                                     const Polygon< BaseKernel > &p)
{
  if(not (idx < this->_repr->size()))
    throw std::out_of_range("polygon index out of range");
  // this->_repr->assign(idx, p._repr);
  auto &ring = this->_repr->at(idx);
  ring.clear();
  ring.assign(p._repr.cbegin(), p._repr.cend());
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::insert_ring(size_t idx, PolygonCSP< BK > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  this->insert_ring(idx, *p);
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::insert_ring(size_t idx, const Polygon< BK > &p)
{
  auto it = this->_idx2iter(idx, true);
  this->_repr->insert(it, p._repr);
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::remove_ring(size_t idx)
{
  auto it = this->_idx2iter(idx);
  this->_repr->erase(it);
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::append_ring(PolygonCSP< BK > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  this->append_ring(*p);
}

template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::append_ring(const Polygon< BK > &p)
{
  this->_repr->push_back(p._repr);
}



template< typename BK, typename RT >
void
PolygonListIF< BK, RT >::clear()
{
  this->_repr->clear();
}



template< typename BK, typename RT >
typename PolygonListIF< BK, RT >::BoostRepr::const_iterator
PolygonListIF< BK, RT >::_idx2iter(size_t idx, bool include_end_p)
  const
{
  // Verify that index is within ranger, i.e.,
  //   0 <= idx <= size(),   if include_end_p
  //   0 <= idx < size(),    if not include_end_p
  if (((include_end_p) and (idx > this->_repr->size())) or
      ((not include_end_p) and (idx >= this->_repr->size())))
    throw std::out_of_range("index out of range");
  // Compute and return the corresponding iterator.
  auto it = this->_repr->cbegin();
  for(size_t k = 0; k < idx; k++)
    it++;
  return it;
}

};
