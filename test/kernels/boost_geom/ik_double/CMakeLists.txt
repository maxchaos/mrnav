set(TEST_NAME test-kernels-boost_geom-ik_double)
set(TEST_EXECUTABLE test-kernels-boost_geom-ik_double.exe)

add_executable(
  ${TEST_EXECUTABLE}
  main.cpp
  point.cpp
  polygon.cpp
  polygon_region.cpp
  polygon_region_set.cpp
  rectangle.cpp
  aux.cpp)

target_link_libraries(
  ${TEST_EXECUTABLE}
  gtest_main
  mrnav_kernels
  tinyxml2)

add_test(
  NAME ${TEST_NAME}
  COMMAND ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/${TEST_EXECUTABLE})
