add_library(
  mrnav_kernels_double
  OBJECT
  core.cpp
  point.cpp
  polygon.cpp
  polygon_region.cpp
  polygon_region_set.cpp
  polygon_if.cpp
  polygon_list_if.cpp
  polygon_region_if.cpp
  rectangle.cpp
  aux_buffer.cpp
)

set_property(
  TARGET
  mrnav_kernels_double
  PROPERTY
  POSITION_INDEPENDENT_CODE ON)
