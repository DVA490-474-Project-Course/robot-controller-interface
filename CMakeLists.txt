cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED_ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

if(Carl)
    project(robot_controller_interface)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(dwb_core REQUIRED)
    find_package(nav_msgs REQUIRED)

    add_subdirectory(src)

else(Communication)
    project(robot_controller_interface)

    find_package(Threads REQUIRED)

    add_subdirectory(src)

    # add .cc files to executable
    file(GLOB_RECURSE SRC_FILES src/simulation-interface/*.cc)
    add_executable(client_demo ${SRC_FILES} src/main.cc)

    # include header files
    target_include_directories(client_demo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation-interface)

    # link libraries
    target_link_libraries(client_demo client_lib)

    #____________________________________________________________
    # Google Test
    #_____________________________________________________________

    # Fetch and Add GoogleTest
    include(FetchContent)

    # Declare GoogleTest as a dependency and download it automatically
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/release-1.11.0.zip
    )

    # Make GoogleTest available
    FetchContent_MakeAvailable(googletest)

    # Add test executable
    file(GLOB_RECURSE TEST_SRC_FILES test/simulation-interface-test/*.cc)
    add_executable(simulation_interface_test ${TEST_SRC_FILES})

    # include header files
    target_include_directories(simulation_interface_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation-interface)

    # Link GoogleTest and GoogleTest main library
    target_link_libraries(simulation_interface_test gtest_main client_lib)
    # Enable testing
    enable_testing()

    # Add a test to be run with `ctest`
    add_test(NAME simulation_interface_test COMMAND simulation_interface_test)

endif()
