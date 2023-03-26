cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(ds STATIC ${SOURCES} ${HEADERS})
target_include_directories(ds PUBLIC ${ADDITIONAL_INCLUDE_DIRS})
target_link_libraries(ds ${REALSENSE2_FOUND})