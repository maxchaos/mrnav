add_subdirectory(models)

add_library(mrnav_planner_6124e1a8685f
  SHARED
  $<TARGET_OBJECTS:mrnav_planner_6124e1a8685f_bgd>)
