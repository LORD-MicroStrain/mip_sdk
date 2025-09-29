add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/cmocka")

set(MIP_UNIT_TEST_TARGET_C "${MIP_UNIT_TEST_TARGET}_c")
set(MIP_INTEGRATION_TEST_TARGET_C "${MIP_INTEGRATION_TEST_TARGET}_c")

if(NOT TARGET "cmocka::cmocka")
    message(WARNING "No cmocka")
else()
    message(WARNING "cmocka!!")
endif()
include(AddCMockaTest)

add_cmocka_test(
    "${MIP_UNIT_TEST_TARGET_C}"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/mip_unit_tests.c"
    LINK_LIBRARIES "${MIP_LIBRARIES}" "cmocka::cmocka"
)

target_include_directories("${MIP_UNIT_TEST_TARGET_C}" PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/cmocka
)

add_subdirectory("microstrain")
#add_subdirectory("mip")
