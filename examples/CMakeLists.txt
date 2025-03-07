set(EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(MIP_EXAMPLE_UTILS_LIBRARY "example_utils")

set(MIP_EXAMPLE_UTILS_SOURCES
    "${EXAMPLE_DIR}/example_utils.c"
    "${EXAMPLE_DIR}/example_utils.h"
    "${EXAMPLE_DIR}/example_utils.cpp"
    "${EXAMPLE_DIR}/example_utils.hpp"
)

add_library(${MIP_EXAMPLE_UTILS_LIBRARY}
    "${MIP_EXAMPLE_UTILS_SOURCES}"
)

if(MSVC)
    source_group(TREE ${EXAMPLE_DIR} FILES ${MIP_EXAMPLE_UTILS_SOURCES})
endif()

set(EXAMPLE_LIBS)

if(MICROSTRAIN_ENABLE_SERIAL)
    set(SERIAL_DEFS "MICROSTRAIN_ENABLE_SERIAL")
    list(APPEND EXAMPLE_LIBS microstrain_serial)
endif()

if(MICROSTRAIN_ENABLE_TCP)
    set(TCP_DEFS "MICROSTRAIN_ENABLE_TCP")
    list(APPEND EXAMPLE_LIBS microstrain_socket)
endif()

if(MICROSTRAIN_ENABLE_SERIAL OR MICROSTRAIN_ENABLE_TCP)
    list(APPEND EXAMPLE_LIBS ${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY})
endif()

if(MIP_ENABLE_EXTRAS)
    set(EXTRAS_DEFS "MIP_ENABLE_EXTRAS")
endif()

set(MIP_EXAMPLE_DEFS ${MIP_DEFINES} ${SERIAL_DEFS} ${TCP_DEFS} ${EXTRAS_DEFS})


target_compile_definitions(${MIP_EXAMPLE_UTILS_LIBRARY}
    PUBLIC
    ${MIP_EXAMPLE_DEFS}
    ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}
)

target_compile_options(${MICROSTRAIN_SERIAL_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

#target_include_directories(${MIP_EXAMPLE_UTILS_LIBRARY} PRIVATE ${SRC_DIR})
target_link_libraries(${MIP_EXAMPLE_UTILS_LIBRARY} PUBLIC ${MIP_LIBRARY} ${EXAMPLE_LIBS})

list(APPEND EXAMPLE_LIBS ${MIP_EXAMPLE_UTILS_LIBRARY})

macro(add_mip_example name sources)

    add_executable(${name} ${sources})

    if(MSVC)
        source_group(TREE ${EXAMPLE_DIR} FILES ${sources})
    endif()

    target_compile_features(${name} PUBLIC c_std_11)
    if(MICROSTRAIN_ENABLE_CPP)  # Technically should only do this for C++ examples but this is simpler
        target_compile_features(${name} PUBLIC cxx_std_11)
    endif()
    target_link_libraries(${name} PUBLIC ${MIP_LIBRARY} ${EXAMPLE_LIBS})
    target_compile_definitions(${name} PUBLIC ${MIP_EXAMPLE_DEFS})
    target_include_directories(${name} PRIVATE ${EXAMPLE_DIR})

endmacro()

add_mip_example(MipPacketExampleC "${EXAMPLE_DIR}/mip_packet_example.c")

if(MICROSTRAIN_ENABLE_CPP)

    add_mip_example(MipPacketExample "${EXAMPLE_DIR}/mip_packet_example.cpp")

    # C++ examples that need either serial or TCP support
    if(MICROSTRAIN_ENABLE_SERIAL OR MICROSTRAIN_ENABLE_TCP)

        # Generic examples
        add_mip_example(DeviceInfo "${EXAMPLE_DIR}/device_info.cpp")
        add_mip_example(WatchImu   "${EXAMPLE_DIR}/watch_imu.cpp")
        add_mip_example(Threads    "${EXAMPLE_DIR}/threading.cpp")
        if(UNIX)
            target_link_libraries(Threads PUBLIC pthread)
        endif()
        if(MIP_ENABLE_METADATA)
            add_mip_example(PrettyPrinter "${EXAMPLE_SOURCES};${EXAMPLE_DIR}/PrettyPrinter/pretty_printer.cpp;${EXAMPLE_DIR}/PrettyPrinter/stringify.cpp;${EXAMPLE_DIR}/PrettyPrinter/stringify.hpp")
            target_link_libraries(PrettyPrinter PRIVATE ${MIP_METADATA_LIBRARY})
        endif()

        # Product-specific examples
        add_mip_example(GQ7_Example                  "${EXAMPLE_DIR}/GQ7/GQ7_example.cpp")
        add_mip_example(CV7_Example                  "${EXAMPLE_DIR}/CV7/CV7_example.cpp")
        add_mip_example(CV7_INS_Simple_Example       "${EXAMPLE_DIR}/CV7_INS/CV7_INS_simple_example.cpp")
        add_mip_example(CV7_INS_Simple_Ublox_Example "${EXAMPLE_DIR}/CV7_INS/CV7_INS_simple_ublox_example.cpp")
        add_mip_example(CX5_GX5_45_Example           "${EXAMPLE_DIR}/CX5_GX5_45/CX5_GX5_45_example.cpp")
        add_mip_example(CX5_GX5_CV5_15_25_Example    "${EXAMPLE_DIR}/CX5_GX5_CV5_15_25/CX5_GX5_CV5_15_25_example.cpp")

    endif()
endif()

# C examples that need serial support
if(MICROSTRAIN_ENABLE_SERIAL)

    add_mip_example(WatchImuC                  "${EXAMPLE_DIR}/watch_imu.c")
    add_mip_example(GQ7_ExampleC               "${EXAMPLE_DIR}/GQ7/GQ7_example.c")
    add_mip_example(CV7_ExampleC               "${EXAMPLE_DIR}/CV7/CV7_example.c")
    add_mip_example(CX5_GX5_45_ExampleC        "${EXAMPLE_DIR}/CX5_GX5_45/CX5_GX5_45_example.c")
    add_mip_example(CX5_GX5_CV5_15_25_ExampleC "${EXAMPLE_DIR}/CX5_GX5_CV5_15_25/CX5_GX5_CV5_15_25_example.c")

endif()
