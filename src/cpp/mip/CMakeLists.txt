
target_sources(mip PRIVATE
    mip.hpp
    mip_all.hpp
    mip_cmdqueue.hpp
    mip_descriptors.hpp
    mip_field.hpp
    mip_interface.hpp
    mip_interface.cpp
    mip_packet.hpp
    mip_parser.hpp
    mip_result.hpp
    mip_serialization.hpp
)

target_compile_features(mip PUBLIC cxx_std_11)

get_target_property(MIPDEFS mip definitions)

set(MIPDEF_HEADERS ${MIPDEFS})
set(MIPDEF_SOURCES ${MIPDEFS})
list(TRANSFORM MIPDEF_HEADERS APPEND ".hpp")
list(TRANSFORM MIPDEF_SOURCES APPEND ".cpp")
list(APPEND MIPDEF_SOURCES ${MIPDEF_HEADERS})
list(TRANSFORM MIPDEF_SOURCES PREPEND "definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    message("C++ definitions = ${MIPDEF_SOURCES}")
endif()

target_sources(mip PRIVATE ${MIPDEF_SOURCES})

if(MIP_ENABLE_METADATA)
    add_subdirectory(metadata)
endif()

if(MIP_ENABLE_EXTRAS)
    add_subdirectory(extras)
endif()
