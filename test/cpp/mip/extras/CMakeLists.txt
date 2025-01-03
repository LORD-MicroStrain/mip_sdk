add_mip_test(
    NAME TestTimeUtils
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_time_utils.cpp"
    COMMAND TestTimeUtils
    ADDL_LINK_LIBRARIES ${MIP_EXTRAS_LIBRARY}
    USE_CPP TRUE
)
