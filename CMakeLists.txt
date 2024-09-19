cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED_ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

project(client VERSION 1.0)

# Protobuf
INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
file(GLOB_RECURSE PROTO_FILES proto/*.proto)
protobuf_generate_cpp(PROTO_CPP PROTO_H
    ${PROTO_FILES}
)

find_package(Threads REQUIRED)




# add .cc files to executable
file(GLOB_RECURSE SRC_FILES src/*.cc)
add_executable(client ${SRC_FILES} ${PROTO_CPP} ${PROTO_H})

# include header files
target_include_directories(client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation-interface ${CMAKE_CURRENT_SOURCE_DIR}/build)

# link libraries
target_link_libraries(client ${Protobuf_LIBRARIES})
#____________________________________________________________
#googleTest
#_____________________________________________________________
# Add GoogleTest manually
#add_subdirectory(/home/aaiza/googletest ${CMAKE_BINARY_DIR}/googletest-build)


# Fetch and Add GoogleTest
include(FetchContent)

# Declare GoogleTest as a dependency and download it automatically
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.11.0.zip
)

# Make GoogleTest available
FetchContent_MakeAvailable(googletest)

# Add your test executable
add_executable(my_tests test/dummy.cpp test/dummy_test.cpp
)

# Include GoogleTest headers
target_include_directories(my_tests PRIVATE 
    ${gtest_SOURCE_DIR}/include
    ${gmock_SOURCE_DIR}/include
)



# Link GoogleTest and GoogleTest main library
target_link_libraries(my_tests gtest_main)
# Enable testing
enable_testing()

# Add a test to be run with `ctest`
add_test(NAME my_test COMMAND my_tests)
