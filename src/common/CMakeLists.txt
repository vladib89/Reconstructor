cmake_minimum_required(VERSION 3.5.0)

file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.hpp")

if(NOT RS_LIB)
    list(REMOVE_ITEM SOURCES RsDevice.cpp)
    list(REMOVE_ITEM HEADERS RsDevice.hpp)
endif()

add_library(common STATIC ${SOURCES} ${HEADERS})
target_link_libraries(common ${REALSENSE2_FOUND})
target_include_directories(common PUBLIC ${ADDITIONAL_INCLUDE_DIRS})
set_target_properties(common PROPERTIES LINKER_LANGUAGE CXX)
