
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


if(WITH_SERIALPORT)

    add_executable(PingTest "${EXAMPLE_DIR}/ping.cpp")
    target_link_libraries(PingTest mip serial)
    # target_include_directories(PingTest "${SRC_DIR}" "${SERIAL_INCLUDE_DIRS}")

endif()
