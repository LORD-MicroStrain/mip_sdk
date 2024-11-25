add_subdirectory(common)
add_subdirectory(connections)

if(MICROSTRAIN_ENABLE_EXTRAS)
    add_subdirectory(extras)
endif()

set(MICROSTRAIN_LIBRARIES_TMP ${MICROSTRAIN_LIBRARIES_TMP} PARENT_SCOPE)
