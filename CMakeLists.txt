cmake_minimum_required(VERSION 3.10)
set(CMAKe_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED_ON)

project (client VERSION 1.0)

find_package(Qt6 REQUIRED COMPONENTS Network)

file(GLOB_RECURSE SRC_FILES src/*.cc)

add_executable(client ${SRC_FILES})
target_link_libraries(client PRIVATE Qt6::Network)
target_include_directories(client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation-interface)
