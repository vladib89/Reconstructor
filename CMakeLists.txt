cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

project(test_registrator)

# Find installed Open3D, this imports
#   Open3D_INCLUDE_DIRS
#   Open3D_LIBRARIES
#   Open3D_LIBRARY_DIRS
#   Open3D_C_FLAGS
#   Open3D_CXX_FLAGSope
#   Open3D_EXE_LINKER_FLAGS
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
    find_package(OpenCV HINTS "E:\\opencv\\build")    

    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")
    else()
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")
    endif()
    
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")
    set(ADDITIONAL_INCLUDE_DIRS "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")
    find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

# Set OS-specific things here
if(WIN32)
    if (ENABLE_GUI)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE")
        # Filament is built with /MT[d], but by default CMake automatically
        # sets "/MD" and "/MDd". They can't coexist, so we need to replace them.
        string(REPLACE "/MD" "/MT" CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
        string(REPLACE "/MDd" "/MTd" CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
        string(REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
        string(REPLACE "/MDd" "/MTd" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
    endif()
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
	add_definitions(-DUNIX)
	add_compile_options(-Wno-deprecated-declarations)
	add_compile_options(-Wno-unused-result)
    add_definitions(-O3)
endif(WIN32)

# Open3D & Realsense
if (Open3D_FOUND)
    OPTION (BUILD_CSHARP_BINDINDS "Build C# wrapper" ON)
    message(STATUS "Found Open3D ${Open3D_VERSION}")

    add_subdirectory(src/logic)
    set(LIBS logic)

    add_subdirectory(src/common)
    set(LIBS ${LIBS} common)

    add_subdirectory(src/hal)
    set(LIBS ${LIBS} hal)

    add_subdirectory(src/depth)
    set(LIBS ${LIBS} depth)

    add_subdirectory(src/testing)
    set(LIBS ${LIBS} testing)

    add_subdirectory(wrappers)
    add_library(holoscan SHARED src/api.hpp src/hs.cpp)
    add_executable(registrator src/main.cpp src/api.hpp src/hs.cpp)
    target_link_libraries(registrator ${LIBS})
    target_link_libraries(holoscan ${LIBS})

    # Hot fix windows dll not found issue, assumming we're using the Release build
    option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)
    set(Open3D_ARCHA "C:\\Program Files (x86)\\Open3D")
    if(WIN32 AND BUILD_SHARED_LIBS)
        message("Will copy Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET registrator POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${Open3D_ARCHA}/bin/Open3D.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
        if(OpenCV_FOUND AND WIN32)
            set(OpenCV_BINARY_DIR "E:\\opencv\\build\\x64\\vc15")
            message("Will copy opencv_world440.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
            add_custom_command(TARGET registrator POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${OpenCV_BINARY_DIR}/bin/opencv_world440.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
        endif()
        if(REALSENSE2_FOUND AND WIN32)
            set(REALSENSE_BINARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")
            message("Will copy realsense2.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
            add_custom_command(TARGET registrator POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${REALSENSE_BINARY_DIR}/realsense2.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
        endif()
    endif()
    if(WIN32 AND ENABLE_GUI)
        message("Will copy SDL2.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET registrator POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${CMAKE_INSTALL_PREFIX}/lib/SDL2.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endif()

else ()
    message(SEND_ERROR "Open3D not found")
endif ()
