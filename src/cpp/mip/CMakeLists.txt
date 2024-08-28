set(MIP_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

target_sources(${MIP_LIBRARY} PRIVATE
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

target_compile_features(${MIP_LIBRARY} PUBLIC cxx_std_11)
target_include_directories(${MIP_LIBRARY} PRIVATE ${MIP_INCLUDE_DIR})

# Get the mipdef list from the C version
get_target_property(MIP_DEFS ${MIP_LIBRARY} definitions)

set(MIP_DEF_HEADERS ${MIP_DEFS})
set(MIP_DEF_SOURCES ${MIP_DEFS})
list(TRANSFORM MIP_DEF_HEADERS APPEND ".hpp")
list(TRANSFORM MIP_DEF_SOURCES APPEND ".cpp")
list(APPEND MIP_DEF_SOURCES ${MIP_DEF_HEADERS})
list(TRANSFORM MIP_DEF_SOURCES PREPEND "definitions/")

if(MICROSTRAIN_CMAKE_DEBUG)
    message("C++ definitions = ${MIP_DEF_SOURCES}")
endif()

target_sources(${MIP_LIBRARY} PRIVATE ${MIP_DEF_SOURCES})

if(MIP_ENABLE_METADATA)
    add_subdirectory(metadata)
endif()

if(MIP_ENABLE_EXTRAS)
    add_subdirectory(extras)
endif()
