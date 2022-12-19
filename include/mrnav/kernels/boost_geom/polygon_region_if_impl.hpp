#pragma once

#include <mrnav/kernels/boost_geom/polygon_region_if.hpp>

#include <iterator>

namespace mrnav::kernels::boost_geom
{

template< typename BK, typename PR >
PolygonRegionIF< BK, PR >::PolygonRegionIF(BoostRepr *ptr)
  : _repr{ptr}
  , _cache_simple_p{}
  , _cache_valid_p{}
{}

template< typename BK, typename PR >
PolygonRegionIF< BK, PR >::PolygonRegionIF(const BoostRepr *ptr)
  : _repr{}
  , _cache_simple_p{}
  , _cache_valid_p{}
{
  if(ptr == nullptr)
    throw std::invalid_argument("pointer to ring is null");
  this->_repr = const_cast<BoostRepr*>(ptr);
}

template< typename BK, typename PR >
const typename PolygonRegionIF< BK, PR >::RingIF
PolygonRegionIF< BK, PR >::get_outer()
  const
{
  return RingIF{ &(this->_repr->outer()) };
}

template< typename BK, typename PR >
typename PolygonRegionIF< BK, PR >::RingIF
PolygonRegionIF< BK, PR >::get_outer()
{
  // Invalidate cache.
  this->invalidate_cache();
  // Return interface to ring.
  return RingIF{ &(this->_repr->outer()) };
}

template< typename BK, typename PR >
void
PolygonRegionIF< BK, PR >::set_outer(PolygonCSP< BaseKernel > pgn)
{
  if(pgn == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  this->set_outer(*pgn);
}

template< typename BK, typename PR >
void
PolygonRegionIF< BK, PR >::set_outer(const Polygon< BaseKernel > &pgn)
{
  // ::mrnav::kernels::boost_geom::BaseKernel< double >::BoostPolygon foo;
  // ::mrnav::kernels::boost_geom::BaseKernel< double, false >::BoostRepr::ring_type foo;
  // foo.insert()
  auto &outer = this->_repr->outer();
  outer.clear();
  outer.insert(outer.begin(), pgn._repr.cbegin(), pgn._repr.cend());
  // Invalidate cache.
  this->invalidate_cache();
}



template< typename BK, typename PR >
const typename PolygonRegionIF< BK, PR >::RingListIF
PolygonRegionIF< BK, PR >::get_inners()
  const
{
  return RingListIF( &(this->_repr->inners()) );
}

template< typename BK, typename PR >
typename PolygonRegionIF< BK, PR >::RingListIF
PolygonRegionIF< BK, PR >::get_inners()
{
  // Invalidate cache.
  this->invalidate_cache();
  // Return interface to ring list.
  return RingListIF( &(this->_repr->inners()) );
}

template< typename BK, typename PR >
template< typename InputIterator >
std::enable_if_t<
  std::is_same_v< typename std::iterator_traits< InputIterator >::value_type,
                  PolygonCSP >,
  void >
PolygonRegionIF< BK, PR >::set_inners(InputIterator begin, InputIterator end)
{
  auto &ringlist = this->_repr->inners();
  ringlist.clear();
  for(auto it = begin; it != end; it++) {
    if(*it != nullptr)
      ringlist.push_back((*it)->_repr);
  }
  // Invalidate cache.
  this->invalidate_cache();
}

template< typename BK, typename PR >
template< typename InputIterator >
std::enable_if_t<
  std::is_same_v< typename std::iterator_traits< InputIterator >::value_type,
                  Polygon >,
  void >
PolygonRegionIF< BK, PR >::set_inners(InputIterator begin, InputIterator end)
{
  auto &ringlist = this->_repr->inners();
  ringlist.clear();
  for(auto it = begin; it != end; it++) {
    ringlist.push_back(it->_repr);
  }
  // Invalidate cache.
  this->invalidate_cache();
}



template< typename BK, typename PR >
bool
PolygonRegionIF< BK, PR >::simple_p() const
{
  if(not this->_cache_simple_p)
    this->_cache_simple_p = gtl::is_simple(*(this->_repr));
  return this->_cache_simple_p.value();
}

template< typename BK, typename PR >
bool
PolygonRegionIF< BK, PR >::empty_p() const
{
  return gtl::is_empty(*(this->_repr));
}

template< typename BK, typename PR >
bool
PolygonRegionIF< BK, PR >::valid_p() const
{
  if(not this->_cache_valid_p)
    this->_cache_valid_p = gtl::is_valid(*(this->_repr));
  return this->_cache_valid_p.value();
}



template< typename BK, typename PR >
bool
PolygonRegionIF< BK, PR >::contains_p(PointCSP< BK > p)
  const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point in null");
  return this->contains_p(*p);
}

template< typename BK, typename PR >
bool
PolygonRegionIF< BK, PR >::contains_p(const Point< BK > &p)
  const
{
  return gtl::within(p._repr, *(this->_repr));
}



template< typename BK, typename PR >
void
PolygonRegionIF< BK, PR >::invalidate_cache()
{
  this->_cache_valid_p.reset();
  this->_cache_valid_p.reset();
}



template< typename BK, typename PR >
tinyxml2::XMLElement*
PolygonRegionIF< BK, PR >::to_xml_element(tinyxml2::XMLDocument *doc,
                                          std::string elt_name)
{
  // Allocate xml element representing this object.
  tinyxml2::XMLElement *xml_reg = doc->NewElement(elt_name.c_str());
  // Create container of outer boundary.
  auto xml_outer_container = doc->NewElement("outer");
  xml_reg->InsertEndChild(xml_outer_container);
  // Convert outer boundary to xml element.
  auto outer = this->get_outer();
  auto xml_outer = outer.to_xml_element(doc, "polygon");
  xml_outer_container->InsertEndChild(xml_outer);
  // Create container of inner boundaries.
  auto xml_inners_container = doc->NewElement("inners");
  xml_reg->InsertEndChild(xml_inners_container);
  // Iterate over inner boundaries and convert each one into an xml element.
  auto inners = this->get_inners();
  for(size_t k = 0; k < inners.size(); k++)
    {
      auto inner = inners.get_ring(k);
      auto xml_inner = inner.to_xml_element(doc, "polygon");
      xml_inners_container->InsertEndChild(xml_inner);
    }
  // Return the xml representation.
  return xml_reg;
}

template< typename BK, typename PR >
void
PolygonRegionIF< BK, PR >::from_xml_element(tinyxml2::XMLElement *xml_elt)
{
  // Reset internal state.
  this->_repr->clear();
  // Get the outer boundary.
  auto xml_outer =
    xml_elt->FirstChildElement("outer")->FirstChildElement("polygon");
  auto outer = this->get_outer();
  outer.from_xml_element(xml_outer);
  // Collect the inner boundaries.
  std::vector< tinyxml2::XMLElement* > xml_inners;
  auto xml_inner =
    xml_elt->FirstChildElement("inners")->FirstChildElement("polygon");
  while(xml_inner != nullptr) {
    xml_inners.push_back(xml_inner);
    xml_inner = xml_inner->NextSiblingElement("polygon");
  }
  // Allocate enough inner boundaries.
  auto &inners = this->_repr->inners();
  inners.resize(xml_inners.size());
  // Iterate over inner boundaries and use interfaces to build them.
  for(size_t k = 0; k < xml_inners.size(); k++)
    {
      RingIF inner{ &(inners.at(k)) };
      inner.from_xml_element(xml_inners[k]);
    }
}



} // mrnav::kernels::boost_geom
