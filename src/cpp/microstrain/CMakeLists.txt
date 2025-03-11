
set(MICROSTRAIN_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/embedded_time.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/index.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/logging.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/platform.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/serialization.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/span.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/serialization/readwrite.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/serialization/serializer.hpp"
)

# The MicroStrain library is created by the C code. Here we add the C++ files to it.
target_sources(${MICROSTRAIN_LIBRARY} PRIVATE ${MICROSTRAIN_SOURCES})

target_compile_features(${MICROSTRAIN_LIBRARY} PUBLIC cxx_std_11)
target_include_directories(${MICROSTRAIN_LIBRARY} INTERFACE ${MICROSTRAIN_SRC_CPP_DIR})

if(MICROSTRAIN_USE_STD_SPAN)
    target_compile_definitions(${MICROSTRAIN_LIBRARY} PUBLIC "MICROSTRAIN_USE_STD_SPAN=1")
endif()

if(MICROSTRAIN_USE_STD_ENDIAN)
    target_compile_definitions(${MICROSTRAIN_LIBRARY} PUBLIC "MICROSTRAIN_USE_STD_ENDIAN=1")
endif()

#
# Components
#

add_subdirectory(connections)

if(MICROSTRAIN_ENABLE_EXTRAS)
    add_subdirectory(extras)
endif()

set(MICROSTRAIN_LIBRARIES_TMP ${MICROSTRAIN_LIBRARIES_TMP} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_LIBRARY} ${MICROSTRAIN_SRC_DIR})
