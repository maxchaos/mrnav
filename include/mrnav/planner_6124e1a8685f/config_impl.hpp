#pragma once

#include <mrnav/planner_6124e1a8685f/config.hpp>
#include <type_traits>

namespace mrnav::planner_6124e1a8685f
{

template <class Kernel>
Planner<Kernel>::Configuration::Configuration(const size_t size)
  : _pos{size}
{
  if(size < 1)
    throw std::invalid_argument("size must be a positive integer");
}

template< class Kernel >
typename Planner< Kernel >::Configuration::Point&
Planner< Kernel >::Configuration::get(size_t idx)
{
  return this->_pos[idx];
}

template< class Kernel >
const typename Planner< Kernel >::Configuration::Point&
Planner< Kernel >::Configuration::get(size_t idx)
  const
{
  return this->_pos[idx];
}

template< class Kernel >
void
Planner< Kernel >::Configuration::set(size_t idx, PointSP p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  this->set(idx, *p);
}

template< class Kernel >
void
Planner< Kernel >::Configuration::set(size_t idx, PointCSP p)
{
  if(p == nullptr)
    throw std::invalid_argument("pointer to point is null");
  this->set(idx, *p);
}

template< class Kernel >
void
Planner< Kernel >::Configuration::set(size_t idx, const Point &p)
{
  this->_pos[idx] = p;
}

template< class Kernel >
size_t
Planner< Kernel >::Configuration::size() const
{
  return this->_pos.size();
}

template< class Kernel >
typename Planner< Kernel >::Configuration::Point&
Planner< Kernel >::Configuration::operator[](size_t idx)
{
  return _pos[idx];
}

template< class Kernel >
const typename Planner< Kernel >::Configuration::Point&
Planner< Kernel >::Configuration::operator[](size_t idx) const
{
  return _pos[idx];
}

} // mrnav::planner_6124e1a8685f
