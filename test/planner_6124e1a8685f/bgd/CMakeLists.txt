set(TEST_NAME test-planner_6124e1a8685f)
set(TEST_EXECUTABLE test-planner_6124e1a8685f.exe)

add_executable(
  ${TEST_EXECUTABLE}
  main.cpp
  robot.cpp
  config.cpp
  problem.cpp)

target_link_libraries(
  ${TEST_EXECUTABLE}
  gtest_main
  mrnav_planner_6124e1a8685f
  mrnav_kernels
  tinyxml2)

add_test(
  NAME ${TEST_NAME}
  COMMAND ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/${TEST_EXECUTABLE})
