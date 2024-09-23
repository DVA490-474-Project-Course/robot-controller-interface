#===============================================================================
# Path planning

add_library(path_planning_lib path_planning.cc)
ament_target_dependencies(path_planning_lib rclcpp dwb_core nav_msgs)

target_include_directories(path_planning_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
# State

add_library(state_lib state.cc)
ament_target_dependencies(state_lib rclcpp dwb_core nav_msgs)

target_include_directories(state_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
# Ball

add_library(ball_lib ball.cc)

target_include_directories(ball_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
