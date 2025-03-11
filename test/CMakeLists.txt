set(MICROSTRAIN_TEST_LIBRARY "MicrostrainTest")

set(MICROSTRAIN_TEST_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/test.c"
    "${CMAKE_CURRENT_LIST_DIR}/test.h"
)

add_library(${MICROSTRAIN_TEST_LIBRARY}
    "${MICROSTRAIN_TEST_SOURCES}"
)

set(TEST_DIR "${CMAKE_CURRENT_LIST_DIR}")

if(MSVC)
    source_group(TREE ${TEST_DIR} FILES ${MICROSTRAIN_TEST_SOURCES})
endif()

# Note: Don't use this macro to add tests, use the SDK specific macros below
macro(add_test_common NAME SOURCES COMMAND LIBRARIES USE_CPP ARGS)
    set(INCLUDE_DIRS "${MICROSTRAIN_SRC_C_DIR}")

    if(${USE_CPP})
        list(APPEND INCLUDE_DIRS "${MICROSTRAIN_SRC_CPP_DIR}")
    endif()

    add_executable(${NAME} ${SOURCES})

    if(MSVC)
        source_group(TREE ${TEST_DIR} FILES ${SOURCES})
    endif()

    target_include_directories(${NAME} PRIVATE
        "${TEST_DIR}"
        "${INCLUDE_DIRS}"
    )

    target_link_libraries(${NAME} PRIVATE
        ${MICROSTRAIN_TEST_LIBRARY}
        "${LIBRARIES}"
    )

    add_test(${NAME} ${COMMAND} ${ARGS})
endmacro()

set(OPTIONS USE_CPP)
set(SINGLE_VALUES NAME COMMAND)
set(MULTI_VALUES SOURCES ADDL_LINK_LIBRARIES ARGS)

macro(add_mip_test)
    set(TEST_ARG_PREFIX "MIP_TEST_ARG")
    cmake_parse_arguments(${TEST_ARG_PREFIX}
        "${OPTIONS}"
        "${SINGLE_VALUES}"
        "${MULTI_VALUES}"
        "${ARGN}"
    )

    set(MIP_TEST_LINK_LIBRARIES
        ${MIP_LIBRARY}
        "${${TEST_ARG_PREFIX}_ADDL_LINK_LIBRARIES}"
    )

    add_test_common(
        "${${TEST_ARG_PREFIX}_NAME}"
        "${${TEST_ARG_PREFIX}_SOURCES}"
        "${${TEST_ARG_PREFIX}_COMMAND}"
        "${MIP_TEST_LINK_LIBRARIES}"
        ${${TEST_ARG_PREFIX}_USE_CPP}
        "${${TEST_ARG_PREFIX}_ARGS}"
    )
endmacro()

macro(add_microstrain_test)
    set(TEST_ARG_PREFIX "MICROSTRAIN_TEST_ARG")
    cmake_parse_arguments(${TEST_ARG_PREFIX}
        "${OPTIONS}"
        "${SINGLE_VALUES}"
        "${MULTI_VALUES}"
        "${ARGN}"
    )

    set(MICROSTRAIN_TEST_LINK_LIBRARIES
        ${MICROSTRAIN_LIBRARY}
        "${${TEST_ARG_PREFIX}_ADDL_LINK_LIBRARIES}"
    )

    add_test_common(
        "${${TEST_ARG_PREFIX}_NAME}"
        "${${TEST_ARG_PREFIX}_SOURCES}"
        "${${TEST_ARG_PREFIX}_COMMAND}"
        "${MICROSTRAIN_TEST_LINK_LIBRARIES}"
        ${${TEST_ARG_PREFIX}_USE_CPP}
        "${${TEST_ARG_PREFIX}_ARGS}"
    )
endmacro()

set(MIP_TEST_DATA_BIN "${CMAKE_CURRENT_LIST_DIR}/data/mip_data.bin")

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/c")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/cpp")

#if(MICROSTRAIN_BUILD_EXAMPLES)
#    set(OUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/MipPacketExampleOutput.txt")
#    add_test(
#        NAME TestMipPacketExample
#        COMMAND diff -u "${CMAKE_CURRENT_LIST_DIR}/data/packet_example_cpp_check.txt" <(examples/MipPacketExample)
#    )
#endif()
