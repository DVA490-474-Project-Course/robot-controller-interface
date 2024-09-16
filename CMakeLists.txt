cmake_minimum_required(VERSION 3.10)
set(CMAKe_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED_ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

project (client VERSION 1.0)

file(GLOB_RECURSE SRC_FILES src/*.cc)

add_executable(client ${SRC_FILES})
target_include_directories(client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation-interface)
