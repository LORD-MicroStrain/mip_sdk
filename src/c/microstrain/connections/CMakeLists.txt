if(MICROSTRAIN_ENABLE_SERIAL)
    add_subdirectory(serial)
endif()

if(MICROSTRAIN_ENABLE_TCP)
    add_subdirectory(tcp)
endif()

set(MICROSTRAIN_LIBRARIES_TMP ${MICROSTRAIN_LIBRARIES_TMP} PARENT_SCOPE)
