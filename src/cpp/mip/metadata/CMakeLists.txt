

set(META_SOURCES
    mip_all_definitions.hpp
    mip_definitions.hpp
    mip_definitions.cpp
    mip_metadata.hpp
    mip_structures.hpp
    mip_meta_utils.hpp
    definitions/commands_base.hpp
    definitions/commands_3dm.hpp
    definitions/data_sensor.hpp
    definitions/common.hpp
)

add_library(mip_metadata ${META_SOURCES})
target_compile_features(mip_metadata PUBLIC cxx_std_20)
