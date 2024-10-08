#
# MIP extras library
#

macro(add_mip_extras_test name sources command)
    add_executable(${name} ${sources})

    target_include_directories(${name} PRIVATE ${MICROSTRAIN_SRC_DIR})
    target_link_libraries(${name} ${MIP_EXTRAS_LIBRARY})

    add_test(${name} ${command} ${ARGN})
endmacro()

add_mip_extras_test(TestTimeUtils "${CMAKE_CURRENT_LIST_DIR}/test_time_utils.cpp" TestTimeUtils)
