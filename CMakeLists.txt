cmake_minimum_required(VERSION 3.10)
set(CMAKe_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED_ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

project(client VERSION 1.0)

# Protobuf
INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
protobuf_generate_cpp(PROTO_CPP PROTO_H
    include/ssl-protobuf/grSim_Commands.proto
    include/ssl-protobuf/grSim_Packet.proto
    include/ssl-protobuf/grSim_Replacement.proto
    include/ssl-protobuf/grSim_Robotstatus.proto
    include/ssl-protobuf/ssl_gc_common.proto
    include/ssl-protobuf/ssl_simulation_config.proto
    include/ssl-protobuf/ssl_simulation_control.proto
    include/ssl-protobuf/ssl_simulation_error.proto
    include/ssl-protobuf/ssl_simulation_robot_control.proto
    include/ssl-protobuf/ssl_simulation_robot_feedback.proto
    include/ssl-protobuf/ssl_simulation_synchronous.proto
    include/ssl-protobuf/ssl_vision_detection.proto
    include/ssl-protobuf/ssl_vision_geometry.proto
    include/ssl-protobuf/ssl_vision_wrapper.proto
)

find_package(Threads REQUIRED)

# Google test (not used yet)
include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)


# add .cc files to executable
file(GLOB_RECURSE SRC_FILES src/*.cc)
add_executable(client ${SRC_FILES} ${PROTO_CPP} ${PROTO_H})

# include header files
target_include_directories(client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation-interface ${CMAKE_CURRENT_SOURCE_DIR}/build)

# link libraries
target_link_libraries(client ${Protobuf_LIBRARIES})
