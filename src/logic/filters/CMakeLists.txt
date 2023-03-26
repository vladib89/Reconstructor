cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(filters STATIC ${SOURCES} ${HEADERS})
target_link_libraries(filters ${Open3D_LIBRARIES})
