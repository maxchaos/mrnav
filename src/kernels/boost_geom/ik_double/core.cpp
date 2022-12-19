#include <mrnav/kernels/boost_geom/ik/double.hpp>

namespace mrnav::kernels::boost_geom
{

// template class BaseKernel< double, aux::buffer::FixedArcLength< double > >;
template class BaseKernel< double,
                           aux::buffer::FixedPointsPerCircle< double > >;
template class Kernel< BaseKernel_double >;

} // mrnav::kernels::boost_geom
