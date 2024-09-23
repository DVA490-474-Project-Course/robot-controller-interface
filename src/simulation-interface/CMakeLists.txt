set(PROTO_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated)

find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY ${PROTO_GENERATED_DIR})

add_library(client_lib client.cc)

# Generate Protobuf files
file(GLOB_RECURSE PROTO_FILES proto/*.proto)
protobuf_generate_cpp(PROTO_CPP PROTO_H ${PROTO_FILES} PROTOC_OUT_DIR ${PROTO_GENERATED_DIR})
target_include_directories(client_lib PUBLIC ${PROTO_GENERATED_DIR})

target_link_libraries(client_lib ${Protobuf_LIBRARIES})
