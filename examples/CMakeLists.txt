
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


set(DEVICE_SOURCES )

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




if(WITH_SERIALPORT)

    add_executable(PingTest "${EXAMPLE_DIR}/ping.cpp" ${DEVICE_SOURCES})
    target_link_libraries(PingTest mip serial)

endif()


if(WITH_SERIALPORT OR WITH_SOCKETS)

    add_executable(DeviceInfo "${EXAMPLE_DIR}/device_info.cpp" ${DEVICE_SOURCES})
    target_link_libraries(DeviceInfo mip "${SERIAL_LIB}" "${SOCKET_LIB}")
    target_compile_definitions(DeviceInfo PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

endif()
