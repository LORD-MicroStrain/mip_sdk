
#option(MICROSTRAIN_ENABLE_LOGGING "Build with logging functions enabled" ON)
set(MICROSTRAIN_LOGGING_MAX_LEVEL "MICROSTRAIN_LOG_LEVEL_WARN" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any call to logging functions with a higher level than this will be compiled out.")
set(MICROSTRAIN_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")


set(SOURCES
    logging.h
    platform.h
    serialization.c
    serialization.h
)

add_library(microstrain_common ${SOURCES})
target_compile_features(microstrain_common PUBLIC c_std_11)

target_compile_definitions(microstrain_common PUBLIC "MICROSTRAIN_TIMESTAMP_TYPE=${MICROSTRAIN_TIMESTAMP_TYPE}")  # TODO rename

# Logging is a little weird since we need to install the header no matter what
if(MICROSTRAIN_ENABLE_LOGGING)
    if(MICROSTRAIN_LOGGING_MAX_LEVEL STREQUAL "")
        message(ERROR "MICROSTRAIN_LOGGING_MAX_LEVEL must be MICROSTRAIN_LOG_LEVEL_* or a number")
    endif()
    target_sources(microstrain_common PRIVATE "../../../c/microstrain/common/logging.c")
    message("MicroStrain logging is enabled. Max level = '${MICROSTRAIN_LOGGING_MAX_LEVEL}'")
    target_compile_definitions(microstrain_common PUBLIC "MICROSTRAIN_ENABLE_LOGGING" "MICROSTRAIN_LOGGING_MAX_LEVEL=${MICROSTRAIN_LOGGING_MAX_LEVEL}")  # TODO rename
endif()

#target_include_directories(microstrain_common PUBLIC ${CMAKE_CURRENT_LIST_DIR})
