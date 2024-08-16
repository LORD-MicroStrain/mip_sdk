
set(SOURCES
    composite_result.hpp
    descriptor_id.hpp
    version.cpp
    version.hpp
)

add_library(mip_extras ${SOURCES})
target_compile_features(mip_extras PUBLIC cxx_std_11)
target_include_directories(mip_extras INTERFACE ${MIP_INCLUDE_DIR})

