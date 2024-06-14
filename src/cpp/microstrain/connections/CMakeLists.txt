
if(MIP_USE_SERIAL)
    add_subdirectory(serial)
endif()

if(MIP_USE_TCP)
    add_subdirectory(tcp)
endif()

# Don't include recording connection if no connections are enabled.
if(MIP_USE_SERIAL OR MIP_USE_TCP)
    add_subdirectory(recording)
endif()
