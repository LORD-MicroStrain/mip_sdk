
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


set(DEVICE_SOURCES
    "${EXAMPLE_DIR}/example_utils.cpp"
    "${EXAMPLE_DIR}/example_utils.hpp"
)

if(WITH_SERIALPORT)

    LIST(APPEND DEVICE_SOURCES
        "${EXAMPLE_DIR}/serial_mip_device.cpp"
        "${EXAMPLE_DIR}/serial_mip_device.hpp"
    )
    set(SERIAL_LIB "serial")
    set(SERIAL_DEFS "MSCL_USE_SERIAL")

endif()

if(WITH_SOCKETS)

    LIST(APPEND DEVICE_SOURCES
        "${EXAMPLE_DIR}/tcp_mip_device.cpp"
        "${EXAMPLE_DIR}/tcp_mip_device.hpp"
    )
    set(SOCKET_LIB "socket")
    set(SOCKET_DEFS "MSCL_USE_SOCKETS")

endif()

add_subdirectory("${EXAMPLE_DIR}/socket")



if(WITH_SERIALPORT OR WITH_SOCKETS)

    if(NOT MSCL_DISABLE_CPP)

        message("Not building C++ examples because MSCL_DISABLE_CPP is set.")

        add_executable(DeviceInfo "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/device_info.cpp" ${DEVICE_SOURCES})
        target_link_libraries(DeviceInfo mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(DeviceInfo PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

        add_executable(WatchImu "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/watch_imu.cpp" ${DEVICE_SOURCES})
        target_link_libraries(WatchImu mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(WatchImu PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

    endif()

    add_executable(WatchImuC "${EXAMPLE_DIR}/watch_imu.c")
    target_link_libraries(WatchImuC mip)

endif()
