cmake_minimum_required(VERSION 3.5.0)

project(HoloArch.HoloScan)

set(LRS_DOTNET_TARGET ${PROJECT_NAME})

add_library(${PROJECT_NAME}
    Program.cs
    NativeMethods.cs
    #GlobalSuppressions.cs
    Properties/AssemblyInfo.cs
    #packages.config
    #stylecop.json
)

include(Base/CMakeLists.txt)
include(Hal/CMakeLists.txt)
include(Common/CMakeLists.txt)
