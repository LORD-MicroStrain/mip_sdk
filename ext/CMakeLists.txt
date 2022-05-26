
option(WITH_SERIALPORT "Use the included serial port library." OFF)

if(WITH_SERIALPORT)

    set(SERIAL_DIR "${CMAKE_CURRENT_LIST_DIR}/serial")

    # The following is adapted from the serial library CMakeLists.txt.
    set(SERIAL_SOURCES
        "${SERIAL_DIR}/src/serial.cc"
        "${SERIAL_DIR}/include/serial/serial.h"
        "${SERIAL_DIR}/include/serial/v8stdint.h"
    )

    set(SERIAL_INCLUDE_DIRS "${SERIAL_DIR}/include" PARENT_SCOPE)

    if(APPLE)
        list(APPEND SERIAL_SOURCES "${SERIAL_DIR}/src/impl/unix.cc")
        list(APPEND SERIAL_SOURCES "${SERIAL_DIR}/src/impl/list_ports/list_ports_osx.cc")
    elseif(UNIX)
        list(APPEND SERIAL_SOURCES "${SERIAL_DIR}/src/impl/unix.cc")
        list(APPEND SERIAL_SOURCES "${SERIAL_DIR}/src/impl/list_ports/list_ports_linux.cc")
    else()
        list(APPEND SERIAL_SOURCES "${SERIAL_DIR}/src/impl/win.cc")
        list(APPEND SERIAL_SOURCES "${SERIAL_DIR}/src/impl/list_ports/list_ports_win.cc")
    endif()

    add_library(serial ${SERIAL_SOURCES})
    target_include_directories(serial PUBLIC "${SERIAL_DIR}/include" )
    if(APPLE)
        find_library(IOKIT_LIBRARY IOKit)
        find_library(FOUNDATION_LIBRARY Foundation)
        target_link_libraries(serial PUBLIC ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
    elseif(UNIX)
        target_link_libraries(serial PUBLIC rt pthread)
    else()
        target_link_libraries(serial PUBLIC setupapi)
    endif()

endif()
