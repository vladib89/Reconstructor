cmake_minimum_required(VERSION 3.5)
file(GLOB HEADERS "*.hpp")
file(GLOB SOURCES "*.cpp")
add_subdirectory(rs232)

add_library(hal STATIC ${HEADERS} ${SOURCES})
target_link_libraries(hal rs232)
set_target_properties(hal PROPERTIES LINKER_LANGUAGE CXX)