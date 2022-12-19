add_subdirectory(ik_double)

add_library(mrnav_kernels
  SHARED
  $<TARGET_OBJECTS:mrnav_kernels_double>)
