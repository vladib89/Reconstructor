cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(opencv STATIC ${HEADERS} ${SOURCES})
target_include_directories(opencv PUBLIC ${OpenCV_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS})
target_link_libraries(opencv ${OpenCV_LIBRARIES} ${Open3D_LIBRARIES})