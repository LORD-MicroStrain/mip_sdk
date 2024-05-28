

set(SOURCES
    recording_connection.cpp
    recording_connection.hpp
    ../../common/connection.hpp
)

add_library(microstrain_recording_connection ${SOURCES})

target_link_libraries(microstrain_recording_connection PUBLIC microstrain_common)

#target_include_directories(microstrain_serial PRIVATE ${SRC_DIR})
