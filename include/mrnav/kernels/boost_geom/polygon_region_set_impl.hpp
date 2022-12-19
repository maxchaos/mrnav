#pragma once

#include <mrnav/kernels/boost_geom/polygon_region_set.hpp>

namespace mrnav::kernels::boost_geom
{

template< typename BK >
PolygonRegionSet< BK >::PolygonRegionSet()
  : _repr{}
{
  // ::mrnav::kernels::boost_geom::BaseKernel< double, false >::BoostMultiPolygon foo;
  // gtl::model::multi_polygon< gtl::model::polygon< gtl::model::d2::point_xy< double > > > foo;
  // foo.
}

template< typename BK >
PolygonRegionSet< BK >::PolygonRegionSet(const RegionIF &rif)
{
  this->_repr.push_back(*(rif._repr));
}

template< typename BK >
size_t
PolygonRegionSet< BK >::size()
  const
{
  return this->_repr.size();
}



template< typename BK >
typename PolygonRegionSet< BK >::RegionIF
PolygonRegionSet< BK >::get_region(size_t idx)
{
  if(idx < this->_repr.size())
    return RegionIF{ &this->_repr.at(idx) };
  throw std::out_of_range("index out of range");
}

template< typename BK >
const typename PolygonRegionSet< BK >::RegionIF
PolygonRegionSet< BK >::get_region(size_t idx)
  const
{
  if(idx < this->_repr.size())
    return RegionIF{ &this->_repr.at(idx) };
  throw std::out_of_range("index out of range");
}



template< typename BK >
void
PolygonRegionSet< BK >::set_region(size_t idx, const RegionIF reg)
{
  // if(idx < this->_repr.size())
  //   this->_repr[idx] = *(reg._repr);
  // throw std::out_of_range("index out of range");
  this->set_region(idx, *(reg._repr));
}

template< typename BK >
void
PolygonRegionSet< BK >::set_region(size_t idx, PolygonRegionCSP< BK > reg)
{
  if(reg == nullptr)
    throw std::invalid_argument("pointer to region in null");
  this->set_region(idx, *reg);
}

template< typename BK >
void
PolygonRegionSet< BK >::set_region(size_t idx, const PolygonRegion< BK > &reg)
{
  // if(idx < this->_repr.size())
  //   this->_repr[idx] = reg._repr;
  // throw std::out_of_range("index out of range");
  this->set_region(idx, reg._repr);
}

template< typename BK >
void
PolygonRegionSet< BK >::set_region(size_t idx, const BoostPolygon &reg)
{
  if(not (idx < this->_repr.size()))
    throw std::out_of_range("index out of range");
  this->_repr[idx] = reg;
}



template< typename BK >
void
PolygonRegionSet< BK >::insert_region(size_t idx, const RegionIF reg)
{
  this->insert_region(idx, *(reg._repr));
}

template< typename BK >
void
PolygonRegionSet< BK >::insert_region(size_t idx,
                                      PolygonRegionCSP< BaseKernel > reg)
{
  if(reg == nullptr)
    throw std::invalid_argument("pointer to region is null");
  this->insert_region(idx, *reg);
}

template< typename BK >
void
PolygonRegionSet< BK >::insert_region(size_t idx,
                                      const PolygonRegion< BaseKernel > &reg)
{
  // // Convert index to iterator.
  // auto it = this->_idx2iter(idx, true);
  // // Insert new region.
  // this->_repr.insert(idx, reg._repr);
  this->insert_region(idx, reg._repr);
}

template< typename BK >
void
PolygonRegionSet< BK >::insert_region(size_t idx, const BoostPolygon &reg)
{
  // Convert index to iterator.
  auto it = this->_idx2iter(idx, true);
  // Insert new region.
  this->_repr.insert(it, reg);
}



template< typename BK >
void
PolygonRegionSet< BK >::remove_region(size_t idx)
{
  // Convert index to iterator.
  auto it = this->_idx2iter(idx);
  // Remove region.
  this->_repr.erase(it);
}



template< typename BK >
void
PolygonRegionSet< BK >::append_region(const RegionIF reg)
{
  this->append_region(*(reg._repr));
}

template< typename BK >
void
PolygonRegionSet< BK >::append_region(PolygonRegionCSP< BaseKernel > reg)
{
  if(reg == nullptr)
    throw std::invalid_argument("pointer to region is null");
  this->append_region(*reg);
}

template< typename BK >
void
PolygonRegionSet< BK >::append_region(const PolygonRegion< BaseKernel > &reg)
{
  // this->push_back(reg._repr);
  this->append_region(reg._repr);
}

template< typename BK >
void
PolygonRegionSet< BK >::append_region(const BoostPolygon &reg)
{
  this->_repr.push_back(reg);
}



template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_buffer(CoordNT radius)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BaseKernel >{} };
  // Alias namespace for brevity.
  namespace gtlb = gtl::strategy::buffer;
  // Compute how many arcs with length less or equal to "buffer_delta_length"
  // are needed to approximate a circle with the given radius.
  // size_t points_per_circle =
  //   std::ceil( (2 * M_PI * radius) / BK::buffer_delta_length );
  // // Define appropriate strategies.
  // gtlb::distance_symmetric< CoordNT > stg_distance(radius);
  // gtlb::join_round stg_join(points_per_circle);
  // gtlb::end_round stg_end(points_per_circle);
  // gtlb::point_circle stg_circle(points_per_circle);
  // gtlb::side_straight stg_side;
  // // Compute the buffer and return it.
  // gtl::buffer(this->_repr, res->_repr,
  //             stg_distance, stg_side, stg_join, stg_end, stg_circle);
  typename BK::BufferStrategy bsb{radius};
  gtl::buffer(this->_repr, res->_repr,
              bsb.stg_distance(), bsb.stg_side(), bsb.stg_join(),
              bsb.stg_end(), bsb.stg_circle());
  return res;
}



template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_union(PolygonCSP< BK > pgn)
  const
{
  if(pgn == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  return this->get_union(*pgn);
}

template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_union(const Polygon< BK > &pgn)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::union_(this->_repr, pgn._repr, res->_repr);
  return res;
}

template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_intersection(PolygonCSP< BK > pgn)
  const
{
  if(pgn == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  return this->get_intersection(*pgn);
}

template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_intersection(const Polygon< BK > &pgn)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::intersection(this->_repr, pgn._repr, res->_repr);
  return res;
}

template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_diff(PolygonCSP< BK > pgn)
  const
{
  if(pgn == nullptr)
    throw std::invalid_argument("pointer to polygon is null");
  return this->get_diff(*pgn);
}

template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_diff(const Polygon< BK > &pgn)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::difference(this->_repr, pgn._repr, res->_repr);
  return res;
}



template< typename  BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_union(PolygonRegionCSP< BK > reg)
const
{
  if(reg == nullptr)
    throw std::invalid_argument("pointer to polygon region is null");
  return this->get_union(*reg);
}

template< typename  BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_union(const PolygonRegion< BK > &reg)
const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::union_(this->_repr, reg._repr, res->_repr);
  return res;
}

template< typename  BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_intersection(PolygonRegionCSP< BK > reg)
const
{
  if(reg == nullptr)
    throw std::invalid_argument("pointer to polygon region is null");
  return this->get_intersection(*reg);
}

template< typename  BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_intersection(const PolygonRegion< BK > &reg)
const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::intersection(this->_repr, reg._repr, res->_repr);
  return res;
}

template< typename  BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_diff(PolygonRegionCSP< BK > reg)
const
{
  if(reg == nullptr)
    throw std::invalid_argument("pointer to polygon region is null");
  return this->get_diff(*reg);
}

template< typename  BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_diff(const PolygonRegion< BK > &reg)
const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::difference(this->_repr, reg._repr, res->_repr);
  return res;
}



template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_union(const Rectangle< BK > &rect)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::union_(this->_repr, rect._repr, res->_repr);
  return res;
}


template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_intersection(const Rectangle< BK > &rect)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::intersection(this->_repr, rect._repr, res->_repr);
  return res;
}

template< typename BK >
PolygonRegionSetSP< BK >
PolygonRegionSet< BK >::get_diff(const Rectangle< BK > &rect)
  const
{
  PolygonRegionSetSP< BK > res{ new PolygonRegionSet< BK >{} };
  gtl::difference(this->_repr, rect._repr, res->_repr);
  return res;
}




template< typename BK >
void
PolygonRegionSet< BK >::buffer(CoordNT radius)
{
  auto buffered = this->get_buffer(radius);
  this->_repr = buffered->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::unite(PolygonCSP< BK > pgn)
{
  auto new_val = this->get_union(pgn);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::unite(const Polygon< BK > &pgn)
{
  auto new_val = this->get_union(pgn);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::intersect(PolygonCSP< BK > pgn)
{
  auto new_val = this->get_intersection(pgn);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::intersect(const Polygon< BK > &pgn)
{
  auto new_val = this->get_intersection(pgn);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::unite(PolygonRegionCSP< BK > reg)
{
  auto new_val = this->get_union(reg);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::unite(const PolygonRegion< BK > &reg)
{
  auto new_val = this->get_union(reg);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::intersect(PolygonRegionCSP< BK > reg)
{
  auto new_val = this->get_intersection(reg);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::intersect(const PolygonRegion< BK > &reg)
{
  auto new_val = this->get_intersection(reg);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::diff(PolygonRegionCSP< BK > reg)
{
  auto new_val = this->get_diff(reg);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::diff(const PolygonRegion< BK > &reg)
{
  auto new_val = this->get_diff(reg);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::unite(const Rectangle< BK > &rect)
{
  auto new_val = this->get_union(rect);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::intersect(const Rectangle< BK > &rect)
{
  auto new_val = this->get_intersection(rect);
  this->_repr = new_val->_repr;
}

template< typename BK >
void
PolygonRegionSet< BK >::diff(const Rectangle< BK > &rect)
{
  auto new_val = this->get_diff(rect);
  this->_repr = new_val->_repr;
}



template< typename BK >
bool
PolygonRegionSet< BK >::empty_p() const
{
  return gtl::is_empty(this->_repr);
}



template< typename BK >
bool
PolygonRegionSet< BK >::contains_p(PointCSP< BK > p) const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  return this->contains_p(*p);
}

template< typename BK >
bool
PolygonRegionSet< BK >::contains_p(const Point< BK > p) const
{
  return gtl::within(p._repr, this->_repr);
}

template< typename BK >
bool
PolygonRegionSet< BK >::intersects_p(PolygonRegionSetSP< BK > set)
  const
{
  if(set == nullptr)
    throw std::invalid_argument("pointer to polygon region set is null");
  return this->intersects_p(*set);
}

template< typename BK >
bool
PolygonRegionSet< BK >::intersects_p(const PolygonRegionSet< BK > &set)
  const
{
  return gtl::intersects(this->_repr, set._repr);
}

template< typename BK >
typename PolygonRegionSet< BK >::DistNT
PolygonRegionSet< BK >::distance(PointCSP< BaseKernel > p) const
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  return this->distance(*p);
}

template< typename BK >
typename PolygonRegionSet< BK >::DistNT
PolygonRegionSet< BK >::distance(const Point< BaseKernel > &p) const
{
  return gtl::distance(this->_repr, p._repr);
}



template< typename BK >
Rectangle< BK >
PolygonRegionSet< BK >::get_envelope() const
{
  gtl::model::box< typename BaseKernel::BoostPoint > mbr;
  gtl::envelope(this->_repr, mbr);
  return Rectangle< BK >{ mbr };
}

template< typename BK >
void
PolygonRegionSet< BK >::get_envelope(CoordNT &x_min, CoordNT &x_max,
                                     CoordNT &y_min, CoordNT &y_max) const
{
  gtl::model::box< typename BaseKernel::BoostPoint > mbr;
  gtl::envelope(this->_repr, mbr);
  auto min_corner = mbr.min_corner();
  auto max_corner = mbr.max_corner();
  x_min = min_corner.x();
  y_min = min_corner.y();
  x_max = max_corner.x();
  y_max = max_corner.y();
}



template< typename BK >
typename PolygonRegionSet< BK >::BoostMultiPolygon::const_iterator
PolygonRegionSet< BK >::_idx2iter(size_t idx, bool include_end_p)
  const
{
  // Verify that index is within ranger, i.e.,
  //   0 <= idx <= size(),   if include_end_p
  //   0 <= idx < size(),    if not include_end_p
  if (((include_end_p) and (idx > this->_repr.size())) or
      ((not include_end_p) and (idx >= this->_repr.size())))
    throw std::out_of_range("index out of range");
  // Compute and return the corresponding iterator.
  auto it = this->_repr.cbegin();
  for(size_t k = 0; k < idx; k++)
    it++;
  return it;
}



template< typename BK >
tinyxml2::XMLElement*
PolygonRegionSet< BK >::to_xml_element(tinyxml2::XMLDocument *doc,
                                       std::string elt_name)
  const
{
  // Allocate xml element representing this object.
  tinyxml2::XMLElement *xml_set = doc->NewElement(elt_name.c_str());
  // Iterate over the regions and convert each one into an xml element.
  for(size_t k = 0; k < this->size(); k++)
    {
      auto reg = this->get_region(k);
      tinyxml2::XMLElement *xml_reg =
        reg.to_xml_element(doc, "polygon-region");
      xml_set->InsertEndChild(xml_reg);
    }
  // Return the xml representation.
  return xml_set;
}

template< typename BK >
void
PolygonRegionSet< BK >::from_xml_element(tinyxml2::XMLElement *xml_elt)
{
  const std::string xml_reg_name = "polygon-region";
  // Reset internal state.
  this->_repr.clear();
  // Collect xml elements corresponding to regions.
  std::vector< tinyxml2::XMLElement* > xml_regions;
  auto xml_reg = xml_elt->FirstChildElement(xml_reg_name.c_str());
  while(xml_reg != nullptr)
    {
      xml_regions.push_back(xml_reg);
      xml_reg = xml_reg->NextSiblingElement(xml_reg_name.c_str());
    }
  // Allocate enough regions.
  this->_repr.resize(xml_regions.size());
  // Iterate over regions and use interfaces to build them.
  for(size_t k = 0; k < xml_regions.size(); k++)
    {
      RegionIF reg{ &this->_repr.at(k) };
      reg.from_xml_element(xml_regions[k]);
    }
}



} // mrnav::kernels::boost_geom
