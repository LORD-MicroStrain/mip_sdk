if(MICROSTRAIN_ENABLE_SERIAL)
    add_subdirectory(serial)
endif()

if(MICROSTRAIN_ENABLE_TCP)
    add_subdirectory(tcp)
endif()

# Don't include recording connection if no connections are enabled.
if(MICROSTRAIN_ENABLE_SERIAL OR MICROSTRAIN_ENABLE_TCP)
    add_subdirectory(recording)
endif()
