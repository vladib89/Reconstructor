cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(rs_depth STATIC ${SOURCES} ${HEADERS})
target_include_directories(rs_depth PUBLIC ${ADDITIONAL_INCLUDE_DIRS})

