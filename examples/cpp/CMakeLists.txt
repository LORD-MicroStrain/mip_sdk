add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/5_series")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/7_series")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/device_info")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/mip_packet")

if(MIP_ENABLE_METADATA)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/pretty_printer")
endif()

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/threading")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/ublox")
