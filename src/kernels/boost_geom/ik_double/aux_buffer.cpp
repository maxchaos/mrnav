#include <mrnav/kernels/boost_geom/ik/double.hpp>

namespace mrnav::kernels::boost_geom::aux::buffer
{

template class FixedArcLength< double >;

template<>
double FixedArcLength< double >::_delta_arc_length{0.1};
template<>
size_t FixedPointsPerCircle< double >::_points_per_circle{16};

} // mrnav::kernels::boost_geom
