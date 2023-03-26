cmake_minimum_required(VERSION 3.5.0)

project(HoloScanWrappers)

if(BUILD_CSHARP_BINDINDS)
   add_subdirectory(csharp)
endif()