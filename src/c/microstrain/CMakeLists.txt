set(MICROSTRAIN_LIBRARY "microstrain" CACHE INTERNAL "Name of the MicroStrain library.")
mark_as_advanced(MICROSTRAIN_LIBRARY)

set(MICROSTRAIN_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")
set(MICROSTRAIN_LOGGING_MAX_LEVEL "MICROSTRAIN_LOGGING_LEVEL_TRACE_" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any call to logging functions with a higher level than this will be compiled out.")

set(MICROSTRAIN_LOGGING_ENABLED OFF)
if(MICROSTRAIN_LOGGING_MAX_LEVEL)
    if(NOT "${MICROSTRAIN_LOGGING_MAX_LEVEL}" MATCHES "^MICROSTRAIN_LOGGING_LEVEL_[A-Z][A-Z][A-Z][A-Z]?[A-Z]?_$" AND NOT "${MICROSTRAIN_LOGGING_MAX_LEVEL}" MATCHES "^[0-9]$")
        message(FATAL_ERROR "MICROSTRAIN_LOGGING_MAX_LEVEL must be MICROSTRAIN_LOGGING_LEVEL_*_ or a positive number")
    endif()

    if(NOT "${MICROSTRAIN_LOGGING_MAX_LEVEL}" MATCHES "^MICROSTRAIN_LOGGING_LEVEL_OFF_$" AND NOT "${MICROSTRAIN_LOGGING_MAX_LEVEL}" MATCHES "^0$")
        set(MICROSTRAIN_LOGGING_ENABLED ON)

        message("MicroStrain logging is enabled. Max level = '${MICROSTRAIN_LOGGING_MAX_LEVEL}'")
    endif()
endif()

# Add the logging level to the compile definitions
# Needs to be here since the cache variable is defined here
list(APPEND MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS
    "MICROSTRAIN_LOGGING_MAX_LEVEL=${MICROSTRAIN_LOGGING_MAX_LEVEL}"
)
set(MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS "${MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS}" PARENT_SCOPE)

set(MICROSTRAIN_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/embedded_time.h"
    "${CMAKE_CURRENT_LIST_DIR}/logging.h"
    "${CMAKE_CURRENT_LIST_DIR}/platform.h"
    "${CMAKE_CURRENT_LIST_DIR}/serialization.c"
    "${CMAKE_CURRENT_LIST_DIR}/serialization.h"
    "${CMAKE_CURRENT_LIST_DIR}/strings.c"
    "${CMAKE_CURRENT_LIST_DIR}/strings.h"
)

add_library(${MICROSTRAIN_LIBRARY}
    ${MICROSTRAIN_SOURCES}
)

# Configure the project in an examples sub-folder for IDEs that support it
file(RELATIVE_PATH PROJECT_RELATIVE_PATH "${MICROSTRAIN_SRC_DIR}" "${CMAKE_CURRENT_LIST_DIR}")
set_target_properties(${MICROSTRAIN_LIBRARY} PROPERTIES
    FOLDER "${PROJECT_RELATIVE_PATH}"
)

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
if(MICROSTRAIN_LOGGING_ENABLED)
    set(MICROSTRAIN_LOGGING_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/logging.c"
    )

    target_sources(${MICROSTRAIN_LIBRARY} PRIVATE
        "${MICROSTRAIN_LOGGING_SOURCES}"
    )

    if(MSVC)
        source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_LOGGING_SOURCES})
    endif()
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
