
option(MIP_ENABLE_LOGGING "Build with logging functions enabled" ON)
set(MIP_LOGGING_MAX_LEVEL "MIP_LOG_LEVEL_WARN" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any log level logged at a higher level than this will result in a noop regardless of runtime configuration.")


set(SOURCES
    platform.h
    descriptor_id.hpp
    index.hpp
    mip_logging.h
)


add_library(microstrain_common ${SOURCES})


# Logging is a little weird since we need to install the header no matter what
if(MIP_ENABLE_LOGGING)
    if(MIP_LOGGING_LEVEL_MAX STREQUAL "")
        message(FATAL_ERROR "MIP_LOGGING_MAX_LEVEL must be MIP_LOG_LEVEL_* or a number")
    endif()
    target_sources(microstrain_common PRIVATE "mip_logging.c")
    message("Logging enabled. Max level = ${MIP_LOGGING_MAX_LEVEL}")
    target_compile_definitions(microstrain_common PUBLIC "MIP_ENABLE_LOGGING" "MIP_LOGGING_MAX_LEVEL=${MIP_LOGGING_MAX_LEVEL}")
endif()

target_include_directories(microstrain_common PUBLIC ${SRC_DIR})
