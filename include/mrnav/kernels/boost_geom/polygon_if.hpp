#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>

#include <tinyxml2.h>
#include <cstring>

namespace mrnav::kernels::boost_geom {

template< typename BaseKernel_, typename BoostRepr_ >
class PolygonIF
{
public:
  using BaseKernel = BaseKernel_;
  using BoostRepr = typename std::remove_cv_t< BoostRepr_ > ;

private:
  BoostRepr *_repr;
  mutable std::optional< bool > _cache_simple_p;
  mutable std::optional< bool > _cache_valid_p;

public:
  PolygonIF(BoostRepr *ring_ptr);
  PolygonIF(const BoostRepr *ring_ptr);

public:
  size_t size() const;
  bool simple_p() const;
  bool empty_p() const;
  bool valid_p() const;

public:
  PointSP< BaseKernel > get_vtx(size_t idx) const;
  void set_vtx(size_t idx, PointCSP< BaseKernel > p);
  void set_vtx(size_t idx, const Point< BaseKernel > &p);
  void insert_vtx(size_t idx, PointCSP< BaseKernel > p);
  void insert_vtx(size_t idx, const Point< BaseKernel > &p);
  void remove_vtx(size_t idx);
  void append_vtx(PointCSP< BaseKernel > p);
  void append_vtx(const Point< BaseKernel > &p);

public:
  void clear();

public:
  bool contains_p(PointCSP< BaseKernel > p) const;
  bool contains_p(const Point< BaseKernel > &p) const;

  bool intersects_p(const PolygonIF &ring);

public:
  void translate(PointCSP< BaseKernel > p);
  void translate(const Point< BaseKernel > &p);
  PolygonSP< BaseKernel > get_translated(PointCSP< BaseKernel > p) const;
  PolygonSP< BaseKernel > get_translated(const Point< BaseKernel > &p) const;

public:
  void reverse();
  PolygonSP< BaseKernel > get_reversed() const;

public:
  void invalidate_cache();

private:
  typename BoostRepr::const_iterator
  _idx2iter(size_t idx, bool include_end_p = false) const;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "polygon");

  void
  from_xml_element(tinyxml2::XMLElement *elt);

};

} // mrnav::kernels::boost_geom
