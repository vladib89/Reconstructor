cmake_minimum_required(VERSION 3.5.0)

project(LANGUAGES CSharp)
include(CSharpUtilities)
set(CMAKE_CSharp_FLAGS "/langversion:6")

add_subdirectory(HoloArch.HoloScan)
