#===============================================================================
# Path planning

add_library(path_planning_lib path_planning.cc)
ament_target_dependencies(path_planning_lib rclcpp dwb_core nav_msgs)

target_link_libraries(path_planning_lib state_lib)

target_include_directories(path_planning_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
# State

add_library(state_lib state.cc)
ament_target_dependencies(state_lib rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)

target_include_directories(state_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
# Ball

add_library(ball_lib ball.cc)

target_link_libraries(ball_lib state_lib)

target_include_directories(ball_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
# Supporting

add_library(supporting_lib supporting.cc)

target_link_libraries(supporting_lib state_lib)

target_include_directories(supporting_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

#===============================================================================
