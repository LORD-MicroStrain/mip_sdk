
add_subdirectory(common)
add_subdirectory(connections)

if(MICROSTRAIN_ENABLE_EXTRAS)
    add_subdirectory(extras)
endif()
