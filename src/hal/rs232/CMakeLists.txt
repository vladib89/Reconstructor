cmake_minimum_required(VERSION 3.5)

add_library(rs232 STATIC rs232.c rs232.h)
target_include_directories(rs232 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
