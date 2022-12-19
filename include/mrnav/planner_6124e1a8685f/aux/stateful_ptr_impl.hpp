#pragma once

#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class T >
stateful_ptr< T >::stateful_ptr()
  : _state{new state_t{nullptr, false}}
{}

template< class T >
stateful_ptr< T >::stateful_ptr(T *ptr)
  : _state{new state_t{ptr}}
{}

template< class T >
stateful_ptr< T >::stateful_ptr(const stateful_ptr &o)
  : _state{o._state}
{}

template< class T >
T*
stateful_ptr< T >::operator->() const
{
  if(this->_state->_ptr == nullptr)
    throw std::runtime_error("trying to dereference nullptr");
  if(not this->_state->_valid_p)
    throw std::runtime_error(
      "trying to dereference pointer to destroyed object");
  return this->_state->_ptr;
}

template< class T >
T&
stateful_ptr< T >::operator*() const
{
  if(this->_state->_ptr == nullptr)
    throw std::runtime_error("trying to dereference nullptr");
  if(not this->_state->_valid_p)
    throw std::runtime_error(
      "trying to dereference pointer to destroyed object");
  return *(this->_state->_ptr);
}

template< class T >
T*
stateful_ptr< T >::get() const
{
  return this->_state->_ptr;
}

template< class T >
bool
stateful_ptr< T >::valid_p() const
{
  return this->_state->_valid_p;
}

template< class T >
void
stateful_ptr< T >::invalidate() {
  this->_state->_valid_p = false;
}

template< class T >
bool
stateful_ptr< T >::operator==(const stateful_ptr<T> &o) const
{
  return (this->_state->_ptr) == (o._state->_ptr);
}

template< class T >
bool
stateful_ptr< T >::operator==(T *o) const
{
  return this->_state->_ptr == o;
}

template< class T >
bool
stateful_ptr< T >::operator!=(const stateful_ptr<T> &o) const
{
  return (this->_state->_ptr) != (o._state->_ptr);
}

template< class T >
bool
stateful_ptr< T >::operator!=(T* o) const
{
  return this->_state->_ptr != o;
}



template< class T >
stateful_ptr< T >::state_t::state_t(T *ptr, bool valid_p)
  : _ptr{ptr}
  , _valid_p{valid_p}
{}

} // mrnav::planner_6124e1a8685f
