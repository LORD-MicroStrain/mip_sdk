

set(META_SOURCES
    "mip_all_definitions.hpp"
    "mip_definitions.hpp"
    "mip_definitions.cpp"
    "mip_metadata.hpp"
    "mip_structures.hpp"
    "mip_meta_utils.hpp"
)

# Get the mipdef list from the C version
get_target_property(MIPDEFS mip definitions)

set(MIPDEF_HEADERS ${MIPDEFS})
list(TRANSFORM MIPDEF_HEADERS APPEND ".hpp")
list(TRANSFORM MIPDEF_HEADERS PREPEND "definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    message("C++ metadata definitions = ${MIPDEF_HEADERS}")
endif()

add_library(mip_metadata ${META_SOURCES} ${MIPDEF_HEADERS})
target_compile_features(mip_metadata PUBLIC cxx_std_20)
target_link_libraries(mip_metadata PUBLIC mip)
target_compile_definitions(mip_metadata PUBLIC "MIP_ENABLE_METADATA=1")
