add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/c")

if(MICROSTRAIN_ENABLE_CPP)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/cpp")
endif()
