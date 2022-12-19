#include <mrnav/kernels/boost_geom/ik/double.hpp>
#include <mrnav/kernels/boost_geom/point_impl.hpp>

namespace mrnav::kernels::boost_geom
{

template class Point< BaseKernel_double >;

template
bool
operator==(const Point< BaseKernel_double >& l,
           const Point< BaseKernel_double >& r);

template
bool
operator!=(const Point< BaseKernel_double >& l,
           const Point< BaseKernel_double >& r);

} // mrnav::kernels::boost_geom
