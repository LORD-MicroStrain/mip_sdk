

add_library(microstrain_recording_connection
    recording_connection.cpp
    recording_connection.hpp
    ../connection.hpp
)

target_link_libraries(microstrain_recording_connection PRIVATE microstrain_private)
target_link_libraries(microstrain_recording_connection PUBLIC microstrain_common)
target_compile_features(microstrain_recording_connection PUBLIC cxx_std_11)
