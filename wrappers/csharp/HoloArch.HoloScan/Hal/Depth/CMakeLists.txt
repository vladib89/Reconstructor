target_sources(${LRS_DOTNET_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/depth_stream_config.cs"
	"${CMAKE_CURRENT_LIST_DIR}/DepthSensor.cs")
#include(${CMAKE_CURRENT_LIST_DIR}/Realsense/CMakeLists.txt)