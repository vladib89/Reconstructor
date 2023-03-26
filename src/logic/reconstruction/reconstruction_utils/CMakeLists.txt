cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(reconstruction_utils STATIC ${HEADERS} ${SOURCES})
target_include_directories(reconstruction_utils PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(reconstruction_utils ${Open3D_LIBRARIES})