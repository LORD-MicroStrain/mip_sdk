# microstrain_common is defined by the C api.
set(MICROSTRAIN_COMMON_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/embedded_time.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/index.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/logging.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/platform.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/serialization.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/serialization/readwrite.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/serialization/serializer.hpp"
)

target_sources(${MICROSTRAIN_COMMON_LIBRARY} PRIVATE
    "${MICROSTRAIN_COMMON_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_COMMON_SOURCES})
endif()

target_compile_features(${MICROSTRAIN_COMMON_LIBRARY} PUBLIC cxx_std_11)
target_include_directories(${MICROSTRAIN_COMMON_LIBRARY} INTERFACE ${MICROSTRAIN_SRC_CPP_DIR})


if(MICROSTRAIN_USE_STD_SPAN)
    target_compile_definitions(${MICROSTRAIN_COMMON_LIBRARY} PUBLIC "MICROSTRAIN_USE_STD_SPAN=1")
endif()

if(MICROSTRAIN_USE_STD_ENDIAN)
    target_compile_definitions(${MICROSTRAIN_COMMON_LIBRARY} PUBLIC "MICROSTRAIN_USE_STD_ENDIAN=1")
endif()

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_COMMON_LIBRARY} ${MICROSTRAIN_SRC_DIR})
