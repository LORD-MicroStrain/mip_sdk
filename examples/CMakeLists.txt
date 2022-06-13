
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


set(DEVICE_SOURCES
)

if(WITH_SERIALPORT)

    LIST(APPEND DEVICE_SOURCES
        "${EXAMPLE_DIR}/serial_mip_device.cpp"
        "${EXAMPLE_DIR}/serial_mip_device.hpp"
    )

endif()




if(WITH_SERIALPORT)

    add_executable(PingTest "${EXAMPLE_DIR}/ping.cpp" ${DEVICE_SOURCES})
    target_link_libraries(PingTest mip serial)
    # target_include_directories(PingTest "${SRC_DIR}" "${SERIAL_INCLUDE_DIRS}")

endif()

