#===============================================================================
# Includes

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

enable_testing()

#===============================================================================
# Add

add_executable(
  main_test
  main_test.cc
)

target_link_libraries(
  main_test
  GTest::gtest_main
)

ament_target_dependencies(main_test rclcpp)
ament_target_dependencies(main_test dwb_core)
ament_target_dependencies(main_test nav_msgs)

target_include_directories(main_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test")

#add_subdirectory(individual-robot-behaviour-test)

#===============================================================================
# Must be last, and in this order.

gtest_discover_tests(main_test)
include(GoogleTest)

#===============================================================================
