#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>
#include <mrnav/kernels/boost_geom/polygon_region_if.hpp>

#include <tinyxml2.h>
#include <cstring>

namespace mrnav::kernels::boost_geom
{

template< typename BaseKernel_ >
class PolygonRegionSet
{
public:
  using BaseKernel = BaseKernel_;
  using CoordNT = typename BaseKernel::CoordNT;
  using DistNT = typename BaseKernel::DistNT;

  using RegionIF = PolygonRegionIF< BaseKernel,
                                    typename BaseKernel::BoostPolygon >;

private:
  using BoostPolygon = typename BaseKernel::BoostPolygon;
  using BoostMultiPolygon = typename BaseKernel::BoostMultiPolygon;

private:
  BoostMultiPolygon _repr;

public:
  PolygonRegionSet();
  PolygonRegionSet(const RegionIF &rif);

public:
  size_t size() const;

public:
  RegionIF get_region(size_t idx);
  const RegionIF get_region(size_t idx) const;
  void set_region(size_t idx, const RegionIF reg);
  void set_region(size_t idx, PolygonRegionCSP< BaseKernel > reg);
  void set_region(size_t idx, const PolygonRegion< BaseKernel > &reg);
  void set_region(size_t idx, const BoostPolygon &reg);

  void insert_region(size_t idx, const RegionIF reg);
  void insert_region(size_t idx, PolygonRegionCSP< BaseKernel > reg);
  void insert_region(size_t idx, const PolygonRegion< BaseKernel > &reg);
  void insert_region(size_t idx, const BoostPolygon &reg);

  void remove_region(size_t idx);

  void append_region(const RegionIF reg);
  void append_region(PolygonRegionCSP< BaseKernel > reg);
  void append_region(const PolygonRegion< BaseKernel > &reg);
  void append_region(const BoostPolygon &reg);

public:
  PolygonRegionSetSP< BaseKernel > get_buffer(CoordNT radius) const;

public:
  PolygonRegionSetSP< BaseKernel >
  get_union(PolygonCSP< BaseKernel > pgn) const;

  PolygonRegionSetSP< BaseKernel >
  get_union(const Polygon< BaseKernel > &pgn) const;

  PolygonRegionSetSP< BaseKernel >
  get_intersection(PolygonCSP< BaseKernel > pgn) const;

  PolygonRegionSetSP< BaseKernel >
  get_intersection(const Polygon< BaseKernel > &pgn) const;

  PolygonRegionSetSP< BaseKernel >
  get_diff(PolygonCSP< BaseKernel > pgn) const;

  PolygonRegionSetSP< BaseKernel >
  get_diff(const Polygon< BaseKernel > &pgn) const;

public:
  PolygonRegionSetSP< BaseKernel >
  get_union(PolygonRegionCSP< BaseKernel > reg) const;

  PolygonRegionSetSP< BaseKernel >
  get_union(const PolygonRegion< BaseKernel > &reg) const;

  PolygonRegionSetSP< BaseKernel >
  get_intersection(PolygonRegionCSP< BaseKernel > reg) const;

  PolygonRegionSetSP< BaseKernel >
  get_intersection(const PolygonRegion< BaseKernel > &reg) const;

  PolygonRegionSetSP< BaseKernel >
  get_diff(PolygonRegionCSP< BaseKernel > reg) const;

  PolygonRegionSetSP< BaseKernel >
  get_diff(const PolygonRegion< BaseKernel > &reg) const;

public:
  PolygonRegionSetSP< BaseKernel >
  get_union(const Rectangle< BaseKernel > &rect) const;

  PolygonRegionSetSP< BaseKernel >
  get_intersection(const Rectangle< BaseKernel > &rect) const;

  PolygonRegionSetSP< BaseKernel >
  get_diff(const Rectangle< BaseKernel > &reg) const;

public:
  void buffer(CoordNT radius);

public:
  void unite(PolygonCSP< BaseKernel > pgn);
  void unite(const Polygon< BaseKernel > &pgn);
  void intersect(PolygonCSP< BaseKernel > pgn);
  void intersect(const Polygon< BaseKernel > &pgn);
  void diff(PolygonCSP< BaseKernel > pgn);
  void diff(const Polygon< BaseKernel > &pgn);

public:
  void unite(PolygonRegionCSP< BaseKernel > reg);
  void unite(const PolygonRegion< BaseKernel > &reg);
  void intersect(PolygonRegionCSP< BaseKernel > reg);
  void intersect(const PolygonRegion< BaseKernel > &reg);
  void diff(PolygonRegionCSP< BaseKernel > reg);
  void diff(const PolygonRegion< BaseKernel > &reg);

public:
  void unite(const Rectangle< BaseKernel > &rect);
  void intersect(const Rectangle< BaseKernel > &rect);
  void diff(const Rectangle< BaseKernel > &rect);

public:
  bool empty_p() const;

public:
  bool contains_p(PointCSP< BaseKernel > p) const;
  bool contains_p(const Point< BaseKernel > p) const;

  bool intersects_p(PolygonRegionSetSP< BaseKernel > set) const;
  bool intersects_p(const PolygonRegionSet< BaseKernel > &set) const;

  DistNT distance(PointCSP< BaseKernel > p) const;
  DistNT distance(const Point< BaseKernel > &p) const;

public:
  Rectangle< BaseKernel > get_envelope() const;

  void
  get_envelope(CoordNT &x_min, CoordNT &x_max,
               CoordNT &y_min, CoordNT &y_max) const;

private:
  typename BoostMultiPolygon::const_iterator
  _idx2iter(size_t idx, bool include_end_p = false) const;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "polygon-region-set") const;

  void
  from_xml_element(tinyxml2::XMLElement *elt);

};

} // mrnav::kernels::boost_geom
