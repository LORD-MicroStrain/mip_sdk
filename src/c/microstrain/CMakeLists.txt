set(MICROSTRAIN_LIBRARY "microstrain" CACHE INTERNAL "Name of the MicroStrain library.")
mark_as_advanced(MICROSTRAIN_LIBRARY)

set(MICROSTRAIN_LOGGING_MAX_LEVEL "MICROSTRAIN_LOG_LEVEL_WARN" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any call to logging functions with a higher level than this will be compiled out.")
set(MICROSTRAIN_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")

set(MICROSTRAIN_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/embedded_time.h"
    "${CMAKE_CURRENT_LIST_DIR}/logging.h"
    "${CMAKE_CURRENT_LIST_DIR}/platform.h"
    "${CMAKE_CURRENT_LIST_DIR}/serialization.c"
    "${CMAKE_CURRENT_LIST_DIR}/serialization.h"
)

add_library(${MICROSTRAIN_LIBRARY} ${MICROSTRAIN_SOURCES})

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_SOURCES})
endif()

target_compile_features(${MICROSTRAIN_LIBRARY} PUBLIC c_std_11)

target_compile_definitions(${MICROSTRAIN_LIBRARY}
    PUBLIC "MICROSTRAIN_TIMESTAMP_TYPE=${MICROSTRAIN_TIMESTAMP_TYPE}"
    PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}
)

target_compile_options(${MICROSTRAIN_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

# Logging is a little weird since we need to install the header no matter what
if(MICROSTRAIN_ENABLE_LOGGING)
    if(MICROSTRAIN_LOGGING_MAX_LEVEL STREQUAL "")
        message(FATAL_ERROR "MICROSTRAIN_LOGGING_MAX_LEVEL must be MICROSTRAIN_LOG_LEVEL_* or a number")
    endif()

    set(MICROSTRAIN_LOGGING_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/logging.c"
    )

    target_sources(${MICROSTRAIN_LIBRARY} PRIVATE
        "${MICROSTRAIN_LOGGING_SOURCES}"
    )

    if(MSVC)
        source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_LOGGING_SOURCES})
    endif()

    message("MicroStrain logging is enabled. Max level = '${MICROSTRAIN_LOGGING_MAX_LEVEL}'")

    target_compile_definitions(${MICROSTRAIN_LIBRARY} PUBLIC
        "MICROSTRAIN_ENABLE_LOGGING" "MICROSTRAIN_LOGGING_MAX_LEVEL=${MICROSTRAIN_LOGGING_MAX_LEVEL}"
    )
endif()

target_include_directories(${MICROSTRAIN_LIBRARY} INTERFACE ${MICROSTRAIN_SRC_C_DIR})

#
# Component libraries
#

add_subdirectory(connections)

set(MICROSTRAIN_LIBRARIES_TMP ${MICROSTRAIN_LIBRARIES_TMP} ${MICROSTRAIN_LIBRARY} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MICROSTRAIN_LIBRARY} ${MICROSTRAIN_SRC_DIR})
