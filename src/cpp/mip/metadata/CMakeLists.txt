set(MIP_METADATA_LIBRARY "mip_metadata" CACHE INTERNAL "Name of the MIP metadata library")
mark_as_advanced(MIP_METADATA_LIBRARY)

# Get the mipdef list from the C version
get_target_property(MIP_DEFS ${MIP_LIBRARY} definitions)

set(MIP_DEF_HEADERS ${MIP_DEFS})
list(TRANSFORM MIP_DEF_HEADERS APPEND ".hpp")
list(TRANSFORM MIP_DEF_HEADERS PREPEND "definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    message("C++ metadata definitions = ${MIP_DEF_HEADERS}")
endif()

add_library(${MIP_METADATA_LIBRARY}
    "mip_all_definitions.hpp"
    "mip_definitions.hpp"
    "mip_definitions.cpp"
    "mip_formatter.hpp"
    "mip_formatter.cpp"
    "mip_meta_utils.hpp"
    "mip_metadata.hpp"
    "mip_format_bytes.hpp"
    "mip_format_bytes.cpp"
    "mip_structures.hpp"
    "mip_meta_utils.hpp"
    "definitions/common.hpp"
    ${MIP_DEF_HEADERS}
)

target_compile_features(${MIP_METADATA_LIBRARY} PUBLIC cxx_std_20)
target_link_libraries(${MIP_METADATA_LIBRARY}
    PRIVATE ${MICROSTRAIN_PRIVATE_LIBRARY}
    PUBLIC ${MIP_LIBRARY}
)
target_compile_definitions(${MIP_METADATA_LIBRARY} PUBLIC "MIP_ENABLE_METADATA=1")

target_link_libraries(${MIP_LIBRARIES} INTERFACE ${MIP_METADATA_LIBRARY})
