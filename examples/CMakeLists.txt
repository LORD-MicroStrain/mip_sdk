
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


set(DEVICE_SOURCES
    "${EXAMPLE_DIR}/example_utils.cpp"
    "${EXAMPLE_DIR}/example_utils.hpp"
)

if(WITH_SERIAL)
    set(SERIAL_DEFS "MIP_USE_SERIAL")
endif()

if(WITH_TCP)
    set(TCP_DEFS "MIP_USE_TCP")
endif()


if(WITH_SERIAL OR WITH_TCP)

    if(NOT MIP_DISABLE_CPP)

        add_executable(DeviceInfo "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/device_info.cpp" ${DEVICE_SOURCES})
        target_link_libraries(DeviceInfo mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(DeviceInfo PUBLIC "${SERIAL_DEFS}" "${TCP_DEFS}")

        add_executable(WatchImu "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/watch_imu.cpp" ${DEVICE_SOURCES})
        target_link_libraries(WatchImu mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(WatchImu PUBLIC "${SERIAL_DEFS}" "${TCP_DEFS}")

        find_package(Threads REQUIRED)
        add_executable(ThreadingDemo "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/threading.cpp" ${DEVICE_SOURCES})
        target_link_libraries(ThreadingDemo mip "${SERIAL_LIB}" "${SOCKET_LIB}" "${CMAKE_THREAD_LIBS_INIT}")
        target_compile_definitions(ThreadingDemo PUBLIC "${SERIAL_DEFS}" "${TCP_DEFS}")

        add_executable(GQ7_Example "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/GQ7/GQ7_example.cpp" ${DEVICE_SOURCES})
        target_link_libraries(GQ7_Example mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(GQ7_Example PUBLIC "${SERIAL_DEFS}" "${TCP_DEFS}")

        add_executable(CV7_Example "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/CV7/CV7_example.cpp" ${DEVICE_SOURCES})
        target_link_libraries(CV7_Example mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(CV7_Example PUBLIC "${SERIAL_DEFS}" "${TCP_DEFS}")

        add_executable(GX5_45_Example "${EXAMPLE_SOURCES}" "${EXAMPLE_DIR}/GX5_45/GX5_45_example.cpp" ${DEVICE_SOURCES})
        target_link_libraries(GX5_45_Example mip "${SERIAL_LIB}" "${SOCKET_LIB}")
        target_compile_definitions(GX5_45_Example PUBLIC "${SERIAL_DEFS}" "${TCP_DEFS}")

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
