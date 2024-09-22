add_library(path_planning_lib path_planning.cc)
ament_target_dependencies(path_planning_lib rclcpp)
ament_target_dependencies(path_planning_lib dwb_core)
ament_target_dependencies(path_planning_lib nav_msgs)

target_include_directories(path_planning_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")


add_library(state_lib state.cc)
ament_target_dependencies(state_lib rclcpp)
ament_target_dependencies(state_lib dwb_core)
ament_target_dependencies(state_lib nav_msgs)

target_include_directories(state_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")


add_library(ball_lib ball.cc)

target_include_directories(ball_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
