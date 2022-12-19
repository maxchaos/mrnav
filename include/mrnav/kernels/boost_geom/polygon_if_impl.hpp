#pragma once

#include <mrnav/kernels/boost_geom/polygon_if.hpp>
#include <boost/geometry/strategies/agnostic/point_in_poly_oriented_winding.hpp>

namespace mrnav::kernels::boost_geom {

template< typename BK, typename RT >
PolygonIF< BK, RT >::PolygonIF(BoostRepr *ptr)
  : _repr{}
  , _cache_simple_p{}
  , _cache_valid_p{}
{
  if(ptr == nullptr)
    throw std::invalid_argument("pointer to ring is null");
  this->_repr = ptr;
}

template< typename BK, typename RT >
PolygonIF< BK, RT >::PolygonIF(const BoostRepr *ptr)
  : _repr{}
  , _cache_simple_p{}
  , _cache_valid_p{}
{
  if(ptr == nullptr)
    throw std::invalid_argument("pointer to ring is null");
  this->_repr = const_cast<BoostRepr*>(ptr);
}

template< typename BK, typename RT>
size_t
PolygonIF< BK, RT >::size() const
{
  return this->_repr->size();
}

template< typename BK, typename RT >
bool
PolygonIF< BK, RT >::simple_p()
  const
{
  // If cache holds a value, return it.
  // Otherwise, first compute a new one and cache it.
  if(not this->_cache_simple_p)
    this->_cache_simple_p = gtl::is_simple(*(this->_repr));
  return this->_cache_simple_p.value();
}

template< typename BK, typename RT>
bool
PolygonIF< BK, RT >::empty_p() const
{
  return gtl::is_empty(*(this->_repr));
}

template< typename BK, typename RT >
bool
PolygonIF< BK, RT >::valid_p()
  const
{
  // If cache holds a value, return it.
  // Otherwise, first compute a new one and cache it.
  if(not this->_cache_valid_p)
    this->_cache_valid_p = gtl::is_valid(*(this->_repr));
  return this->_cache_valid_p.value();
}

template< typename BK, typename RT >
PointSP< BK >
PolygonIF< BK, RT >::get_vtx(size_t idx)
  const
{
  return PointSP< BK >{new Point< BK >{this->_repr->at(idx)}};
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::set_vtx(size_t idx, PointCSP< BaseKernel > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  this->set_vtx(idx, *p);
  // Invalidate cache
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::set_vtx(size_t idx, const Point< BaseKernel > &p)
{
  // gtl::model::linestring< gtl::model::d2::point_xy<double> > foo;
  // gtl::model::linestring< gtl::model::d2::point_xy<double> >::value_type bar;
  if(not (idx < this->_repr->size()))
    throw std::out_of_range("index out of range");
  // this->_repr->assign(idx, p._repr);
  auto &pnt = this->_repr->at(idx);
  pnt = p._repr;
  // Invalidate cache
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::insert_vtx(size_t idx, PointCSP< BK > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  this->insert_vtx(idx, *p);
  // Invalidate cache
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::insert_vtx(size_t idx, const Point< BK > &p)
{
  // Convert index to iterator.
  auto it = this->_idx2iter(idx, true);
  // Insert point to place.
  this->_repr->insert(it, p._repr);
  // Invalidate cache.
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::remove_vtx(size_t idx)
{
  // Convert index to iterator.
  auto it = this->_idx2iter(idx);
  // Remove point from place.
  this->_repr->erase(it);
  // Invalidate cache.
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::append_vtx(PointCSP< BK > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  this->append_vtx(*p);
  // Invalidate cache
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::append_vtx(const Point< BK > &p)
{
  // Append point to the end.
  this->_repr->push_back(p._repr);
  // Invalidate cache.
  this->invalidate_cache();
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::clear()
{
  // Remove everything.
  this->_repr->clear();
  // Invalidate cache.
  this->invalidate_cache();
}

template< typename BK, typename RT >
bool
PolygonIF< BK, RT >::contains_p(PointCSP< BK > p)
  const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  return this->contains_p(*p);
}

template< typename BK, typename RT >
bool
PolygonIF< BK, RT >::contains_p(const Point< BK > &p)
  const
{
  gtl::strategy::within::oriented_winding< !BK::ccw_orientation_p,
                                           typename BK::BoostPoint > stg;
  return gtl::within(p._repr, *(this->_repr), stg);
}

template< typename BK, typename RT >
bool
PolygonIF< BK, RT >::intersects_p(const PolygonIF &ring)
{
  return gtl::intersects(*(this->_repr), *(ring._repr));
}

template< typename BK, typename RT >
typename PolygonIF< BK, RT >::BoostRepr::const_iterator
PolygonIF< BK, RT >::_idx2iter(size_t idx, bool include_end_p)
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



template< typename BK, typename RT >
void
PolygonIF< BK, RT >::translate(PointCSP< BK > p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  this->translate(*p);
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::translate(const Point< BK > &p)
{
  for(auto it = this->_repr->begin(); it != this->_repr->end(); it++) {
    typename BK::BoostPoint p_{*it};
    p_.x(p_.x() + p._repr.x());
    p_.y(p_.y() + p._repr.y());
    *it = p_;
  }
}



template< typename BK, typename RT >
PolygonSP< BK >
PolygonIF< BK, RT >::get_translated(PointCSP< BK > p)
  const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  return this->get_translated(*p);
}

template< typename BK, typename RT >
PolygonSP< BK >
PolygonIF< BK, RT >::get_translated(const Point< BK > &p)
  const
{
  PolygonSP< BK > res{new Polygon<BK> {}};
  auto &res_repr = res->_repr;
  for(auto it = this->_repr->cbegin(); it != this->_repr->cend(); it++) {
    typename BK::BoostPoint p_{*it};
    p_.x(p_.x() + p._repr.x());
    p_.y(p_.y() + p._repr.y());
    res_repr.push_back(p_);
  }
  return res;
}



template< typename BK, typename RT >
void
PolygonIF< BK, RT >::reverse()
{
  gtl::reverse(*(this->_repr));
}

template< typename BK, typename RT >
PolygonSP< BK >
PolygonIF< BK, RT >::get_reversed() const
{
  PolygonSP< BK > res{new Polygon< BK >{*(this->_repr)}};
  gtl::reverse(res->_repr);
  return res;
}



template< typename BK, typename RT >
void
PolygonIF< BK, RT >::invalidate_cache()
{
  this->_cache_simple_p.reset();
  this->_cache_valid_p.reset();
}



template< typename BK, typename RT >
tinyxml2::XMLElement*
PolygonIF< BK, RT >::to_xml_element(tinyxml2::XMLDocument *doc,
                                    std::string elt_name)
{
  // Allocate xml element representing this object.
  tinyxml2::XMLElement *xml_pgn = doc->NewElement(elt_name.c_str());
  // Iterate over the ring's vertices.
  for(size_t k = 0; k < this->size(); k++)
    {
      auto vtx = this->_repr->at(k);
      auto xml_vtx = doc->NewElement("vertex");
      xml_vtx->SetAttribute("x", vtx.x());
      xml_vtx->SetAttribute("y", vtx.y());
      xml_pgn->InsertEndChild(xml_vtx);
    }
  // Return the xml representation.
  return xml_pgn;
}

template< typename BK, typename RT >
void
PolygonIF< BK, RT >::from_xml_element(tinyxml2::XMLElement *xml_elt)
{
  // Reset internal state.
  this->_repr->clear();
  // Collect xml elements corresponding to vertices.
  std::vector< tinyxml2::XMLElement* > xml_vertices;
  tinyxml2::XMLElement *xml_vtx = xml_elt->FirstChildElement("vertex");
  while(xml_vtx != nullptr) {
    xml_vertices.push_back(xml_vtx);
    xml_vtx = xml_vtx->NextSiblingElement("vertex");
  }
  // Allocate enough vertices.
  this->_repr->resize(xml_vertices.size());
  // Set vertices.
  for(size_t k = 0; k < xml_vertices.size(); k++) {
    auto &vtx = this->_repr->at(k);
    vtx.x(xml_vertices[k]->DoubleAttribute("x"));
    vtx.y(xml_vertices[k]->DoubleAttribute("y"));
  }
}



} // mrnav::kernels::boost_geom
