

#option(MICROSTRAIN_ENABLE_SERIAL "Build serial connection support into the library and examples" ON)
#option(MICROSTRAIN_ENABLE_TCP    "Build TCP connection support into the library and exampels"    ON)


if(MICROSTRAIN_ENABLE_SERIAL)
    add_subdirectory(serial)
endif()

if(MICROSTRAIN_ENABLE_TCP)
    add_subdirectory(tcp)
endif()
