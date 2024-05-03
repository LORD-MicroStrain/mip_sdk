

option(MIP_USE_SERIAL "Build serial connection support into the library and examples" ON)
option(MIP_USE_TCP    "Build TCP connection support into the library and exampels"    ON)

if(MIP_USE_SERIAL)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/serial)
endif()

if(MIP_USE_TCP)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tcp)
endif()


