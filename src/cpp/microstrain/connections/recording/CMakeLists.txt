set(MICROSTRAIN_RECORDING_CONNECTION_LIBRARY "microstrain_recording_connection" CACHE INTERNAL "Name of the MicroStrain recording connections library")
mark_as_advanced(MICROSTRAIN_RECORDING_CONNECTION_LIBRARY)

add_library(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY}
    recording_connection.cpp
    recording_connection.hpp
    ../connection.hpp
)

target_compile_features(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY} PUBLIC cxx_std_11)
target_link_libraries(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY}
    PRIVATE ${MICROSTRAIN_PRIVATE_LIBRARY}
    PUBLIC ${MICROSTRAIN_COMMON_LIBRARY}
)

target_link_libraries(${MICROSTRAIN_LIBRARIES} INTERFACE ${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY})
