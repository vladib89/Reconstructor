cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(stepper STATIC ${HEADERS} ${SOURCES})
target_include_directories(stepper PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(stepper ${Open3D_LIBRARIES})