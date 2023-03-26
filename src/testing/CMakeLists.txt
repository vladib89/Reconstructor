cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

add_library(testing STATIC ${HEADERS} ${SOURCES})
target_include_directories(testing PUBLIC ${Open3D_INCLUDE_DIRS} 
	                                                    ${ADDITIONAL_INCLUDE_DIRS}
	                                                    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(testing ${Open3D_LIBRARIES} ${OpenCV_LIBRARIES})