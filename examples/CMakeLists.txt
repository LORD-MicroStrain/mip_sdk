
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


set(DEVICE_SOURCES
    "${EXAMPLE_DIR}/example_utils.cpp"
    "${EXAMPLE_DIR}/example_utils.hpp"
)

if(WITH_SERIALPORT)
    set(SERIAL_DEFS "MIP_USE_SERIAL")
endif()

if(WITH_SOCKETS)

    LIST(APPEND DEVICE_SOURCES
        "${EXAMPLE_DIR}/tcp_mip_device.cpp"
        "${EXAMPLE_DIR}/tcp_mip_device.hpp"
    )
    set(SOCKET_LIB "socket")
    set(SOCKET_DEFS "MIP_USE_SOCKETS")

endif()

add_subdirectory("${EXAMPLE_DIR}/socket")



if(WITH_SERIALPORT OR WITH_SOCKETS)

    if(NOT MIP_DISABLE_CPP)

        add_executable(DeviceInfo "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/device_info.cpp" ${DEVICE_SOURCES})
        target_link_libraries(DeviceInfo mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(DeviceInfo PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

        add_executable(WatchImu "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/watch_imu.cpp" ${DEVICE_SOURCES})
        target_link_libraries(WatchImu mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(WatchImu PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

        add_executable(GQ7_Example "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/GQ7/GQ7_example.cpp" ${DEVICE_SOURCES})
        target_link_libraries(GQ7_Example mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(GQ7_Example PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

        add_executable(CV7_Example "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/CV7/CV7_example.cpp" ${DEVICE_SOURCES})
        target_link_libraries(CV7_Example mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(CV7_Example PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

        add_executable(GX5_45_Example "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/GX5_45/GX5_45_example.cpp" ${DEVICE_SOURCES})
        target_link_libraries(GX5_45_Example mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(GX5_45_Example PUBLIC "${SERIAL_DEFS}" "${SOCKET_DEFS}")

    endif()

    add_executable(WatchImuC "${EXAMPLE_DIR}/watch_imu.c")
    target_link_libraries(WatchImuC mip)

    add_executable(GQ7_ExampleC "${EXAMPLE_DIR}/GQ7/GQ7_example.c")
    target_link_libraries(GQ7_ExampleC mip)

    add_executable(CV7_ExampleC "${EXAMPLE_DIR}/CV7/CV7_example.c")
    target_link_libraries(CV7_ExampleC mip)

    add_executable(GX5_45_ExampleC "${EXAMPLE_DIR}/GX5_45/GX5_45_example.c")
    target_link_libraries(GX5_45_ExampleC mip)

endif()
