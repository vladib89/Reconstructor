message(${CMAKE_CURRENT_LIST_DIR})
target_sources(${LRS_DOTNET_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/Object.cs"
	"${CMAKE_CURRENT_LIST_DIR}/DeleterHandler.cs")