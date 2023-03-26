cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_subdirectory(opencv)
add_subdirectory(reconstruction_utils)
add_subdirectory(stepper)
add_library(reconstruction STATIC ${SOURCES} ${HEADERS})
target_link_libraries(reconstruction ${Open3D_LIBRARIES} ${REALSENSE2_FOUND} opencv
 reconstruction_utils hal stepper)
target_include_directories(reconstruction PUBLIC ${Open3D_INCLUDE_DIRS} ${ADDITIONAL_INCLUDE_DIRS})
