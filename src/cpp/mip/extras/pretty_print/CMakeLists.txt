
list(APPEND MIP_EXTRAS_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/formatter.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/formatter.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/basic_formatter.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/basic_formatter.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/packet_printer.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/packet_printer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/debug_print.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/debug_print.hpp"
)

set(MIP_EXTRAS_SOURCES ${MIP_EXTRAS_SOURCES} PARENT_SCOPE)
