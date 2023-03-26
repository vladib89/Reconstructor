cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_subdirectory(realsense)
add_library(depth STATIC ${HEADERS} ${SOURCES})
target_link_libraries(depth rs_depth)
#set_target_properties(depth PROPERTIES LINKER_LANGUAGE CXX)
