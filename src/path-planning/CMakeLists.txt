add_library(path_planning_lib path_planning.cc)
ament_target_dependencies(path_planning_lib rclcpp)

target_include_directories(path_planning_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
