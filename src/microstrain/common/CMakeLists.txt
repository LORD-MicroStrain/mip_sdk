
option(MIP_ENABLE_LOGGING "Build with logging functions enabled" ON)
set(MICROSTRAIN_LOGGING_MAX_LEVEL "MICROSTRAIN_LOG_LEVEL_WARN" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any call to logging functions with a higher level than this will be compiled out.")


set(SOURCES
    serialization.hpp
    connection.hpp
    descriptor_id.hpp
    index.hpp
    logging.h
    platform.h
    serialization.c
    serialization.h
)


add_library(microstrain_common ${SOURCES})


# Logging is a little weird since we need to install the header no matter what
if(MIP_ENABLE_LOGGING)
    if(MICROSTRAIN_LOGGING_MAX_LEVEL STREQUAL "")
        message(ERROR "MICROSTRAIN_LOGGING_MAX_LEVEL must be MIP_LOG_LEVEL_* or a number")
    endif()
    target_sources(microstrain_common PRIVATE "logging.c")
    message("MicroStrain logging is enabled. Max level = '${MICROSTRAIN_LOGGING_MAX_LEVEL}'")
    target_compile_definitions(microstrain_common PUBLIC "MIP_ENABLE_LOGGING" "MICROSTRAIN_LOGGING_MAX_LEVEL=${MICROSTRAIN_LOGGING_MAX_LEVEL}")
endif()

target_include_directories(microstrain_common PUBLIC ${SRC_DIR})
