target_sources(${LRS_DOTNET_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/SerialDevice.cs"
        "${CMAKE_CURRENT_LIST_DIR}/stream_config.cs"
	"${CMAKE_CURRENT_LIST_DIR}/Sensor.cs"
	"${CMAKE_CURRENT_LIST_DIR}/Frame.cs")

include(${CMAKE_CURRENT_LIST_DIR}/Stepper/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Depth/CMakeLists.txt)
