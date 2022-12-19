add_executable(
  planner_exe
  # Sources:
  exe.cpp
  )

target_link_libraries(planner_exe
  mrnav_planner_6124e1a8685f
  mrnav_kernels
  tinyxml2)
