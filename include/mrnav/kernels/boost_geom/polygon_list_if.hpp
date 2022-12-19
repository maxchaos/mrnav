#pragma once

#include <mrnav/kernels/boost_geom/core.hpp>

namespace mrnav::kernels::boost_geom {

template< typename BaseKernel_, typename BoostRepr_ >
class PolygonListIF
{
public:
  using BaseKernel = BaseKernel_;
  using BoostRepr = typename std::remove_cv_t< BoostRepr_ >;
  using RingIF =
    PolygonIF< BaseKernel,
               typename BoostRepr_::value_type >;

private:
  BoostRepr *_repr;

public:
  PolygonListIF(BoostRepr *ringlist);
  PolygonListIF(const BoostRepr *ringlist);

public:
  size_t size() const;
  void resize(size_t new_size);

public:
  const RingIF get_ring(size_t idx) const;
  RingIF get_ring(size_t idx);
  void set_ring(size_t idx, PolygonCSP< BaseKernel > p);
  void set_ring(size_t idx, const Polygon< BaseKernel > &p);
  void insert_ring(size_t idx, PolygonCSP< BaseKernel > p);
  void insert_ring(size_t idx, const Polygon< BaseKernel > &p);
  void remove_ring(size_t idx);
  void append_ring(PolygonCSP< BaseKernel > p);
  void append_ring(const Polygon< BaseKernel > &p);

public:
  void clear();

private:
  typename BoostRepr::const_iterator
  _idx2iter(size_t idx, bool include_end_p = false)
    const;

};

} // mrnav::kernels::boost_geom
