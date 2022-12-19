#pragma once

#include <memory>

namespace mrnav::planner_6124e1a8685f
{

template< typename T >
class stateful_ptr
{

private:
  class state_t;
  std::shared_ptr< state_t > _state;

public:
  stateful_ptr();
  stateful_ptr(T* ptr);
  stateful_ptr(const stateful_ptr &o);

  bool valid_p() const;
  void invalidate();

  T* operator->() const;
  T& operator*() const;
  T* get() const;

public:
  // friend bool operator==(const stateful_ptr<T> &lhs,
  //                        const stateful_ptr<T> &rhs);
  // friend bool operator==(T *lhs, const stateful_ptr<T> &rhs);
  // friend bool operator==(const stateful_ptr<T> &lhs, T *rhs);
  bool operator==(const stateful_ptr<T> &o) const;
  bool operator==(T *o) const;
  bool operator!=(const stateful_ptr<T> &o) const;
  bool operator!=(T* o) const;

};

template< typename T >
class stateful_ptr< T >::state_t
{

private:
  T *_ptr;
  bool _valid_p;

private:
  state_t(T *ptr, bool valid_p = true);

public:
  friend stateful_ptr< T >;

};

} // mrnav::planner_6124e1a8685f
