
set(SOURCES
    composite_result.hpp
    descriptor_id.hpp
    firmware_version.cpp
    firmware_version.hpp
)

add_library(mip_extras ${SOURCES})
target_link_libraries(mip_extras PRIVATE microstrain_private)
target_compile_features(mip_extras PUBLIC cxx_std_11)
target_link_libraries(mip_extras PUBLIC mip microstrain_extras)
target_compile_definitions(mip_extras PUBLIC "MIP_ENABLE_EXTRAS=1")
