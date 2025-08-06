
set(MIP_PRETTY_PRINTER_LIBRARY mip_pretty_print)

add_library("${MIP_PRETTY_PRINTER_LIBRARY}" STATIC
    "formatter.cpp"
    "formatter.hpp"
    "basic_formatter.hpp"
    "basic_formatter.cpp"
    "packet_printer.hpp"
    "packet_printer.cpp"
    "debug_print.cpp"
    "debug_print.hpp"
)

target_include_directories(${MIP_PRETTY_PRINTER_LIBRARY} PRIVATE )

target_link_libraries(${MIP_PRETTY_PRINTER_LIBRARY} PUBLIC ${MIP_LIBRARY} ${MIP_METADATA_LIBRARY})

set(MIP_PRETTY_PRINTER_LIBRARY ${MIP_PRETTY_PRINTER_LIBRARY} PARENT_SCOPE)
