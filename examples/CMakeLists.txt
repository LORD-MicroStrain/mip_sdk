
set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")


set(EXAMPLE_SOURCES
    "${EXAMPLE_DIR}/example_utils.c"
    "${EXAMPLE_DIR}/example_utils.h"
    "${EXAMPLE_DIR}/example_utils.cpp"
    "${EXAMPLE_DIR}/example_utils.hpp"
)

set(EXAMPLE_LIBS )

if(MICROSTRAIN_ENABLE_SERIAL)
    set(SERIAL_DEFS "MICROSTRAIN_ENABLE_SERIAL")
    list(APPEND EXAMPLE_LIBS microstrain_serial)
endif()

if(MICROSTRAIN_ENABLE_TCP)
    set(TCP_DEFS "MICROSTRAIN_ENABLE_TCP")
    list(APPEND EXAMPLE_LIBS microstrain_socket)
endif()

if(MICROSTRAIN_ENABLE_SERIAL OR MICROSTRAIN_ENABLE_TCP)
    list(APPEND EXAMPLE_LIBS microstrain_recording_connection)
endif()

if(MIP_ENABLE_EXTRAS)
    set(EXTRAS_DEFS "MIP_ENABLE_EXTRAS")
endif()

set(MIP_EXAMPLE_DEFS ${MIP_DEFINES} ${SERIAL_DEFS} ${TCP_DEFS} ${EXTRAS_DEFS})

macro(add_mip_example name sources)

    add_executable(${name} ${sources})

    target_compile_features(${name} PUBLIC c_std_11 cxx_std_14)
    target_link_libraries(${name} PUBLIC mip ${EXAMPLE_LIBS})
    target_compile_definitions(${name} PUBLIC ${MIP_EXAMPLE_DEFS})
    target_include_directories(${name} PRIVATE ${SRC_DIR} ${EXAMPLE_DIR})

endmacro()

# C++ examples need either serial or TCP support
if(MICROSTRAIN_ENABLE_SERIAL OR MICROSTRAIN_ENABLE_TCP)

    if(NOT MICROSTRAIN_ENABLE_CPP)

        # Generic examples
        add_mip_example(DeviceInfo "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/device_info.cpp")
        add_mip_example(WatchImu   "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/watch_imu.cpp")
        add_mip_example(Threads    "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/threading.cpp")
        if(MIP_ENABLE_METADATA)
            add_mip_example(CsvExample "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CSV/csv.cpp;${EXAMPLE_DIR}/CSV/stringify.cpp;${EXAMPLE_DIR}/CSV/stringify.hpp")
            target_link_libraries(CsvExample PRIVATE mip_metadata)
        endif()

        # Product-specific examples
        add_mip_example(GQ7_Example                  "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/GQ7/GQ7_example.cpp")
        add_mip_example(CV7_Example                  "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CV7/CV7_example.cpp")
        add_mip_example(CV7_INS_Simple_Example       "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CV7_INS/CV7_INS_simple_example.cpp")
        add_mip_example(CV7_INS_Simple_Ublox_Example "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CV7_INS/CV7_INS_simple_ublox_example.cpp")
        add_mip_example(CX5_GX5_45_Example           "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CX5_GX5_45/CX5_GX5_45_example.cpp")
        add_mip_example(CX5_GX5_CV5_15_25_Example    "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CX5_GX5_CV5_15_25/CX5_GX5_CV5_15_25_example.cpp")
    endif()

endif()

# C examples need serial support
if(MICROSTRAIN_ENABLE_SERIAL)

    add_mip_example(WatchImuC                  "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/watch_imu.c")
    add_mip_example(GQ7_ExampleC               "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/GQ7/GQ7_example.c")
    add_mip_example(CV7_ExampleC               "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CV7/CV7_example.c")
    add_mip_example(CX5_GX5_45_ExampleC        "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CX5_GX5_45/CX5_GX5_45_example.c")
    add_mip_example(CX5_GX5_CV5_15_25_ExampleC "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/CX5_GX5_CV5_15_25/CX5_GX5_CV5_15_25_example.c")

endif()
