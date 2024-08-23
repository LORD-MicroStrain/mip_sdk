
#option(MICROSTRAIN_ENABLE_LOGGING "Build with logging functions enabled" ON)
set(MICROSTRAIN_LOGGING_MAX_LEVEL "MICROSTRAIN_LOG_LEVEL_WARN" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any call to logging functions with a higher level than this will be compiled out.")
set(MICROSTRAIN_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")

add_library(microstrain_common
    "logging.h"
    "platform.h"
    "serialization.c"
    "serialization.h"
)

target_compile_features(microstrain_common PUBLIC c_std_11)
target_compile_definitions(microstrain_common PUBLIC "MICROSTRAIN_TIMESTAMP_TYPE=${MICROSTRAIN_TIMESTAMP_TYPE}")

# Logging is a little weird since we need to install the header no matter what
if(MICROSTRAIN_ENABLE_LOGGING)
    if(MICROSTRAIN_LOGGING_MAX_LEVEL STREQUAL "")
        message(FATAL_ERROR "MICROSTRAIN_LOGGING_MAX_LEVEL must be MICROSTRAIN_LOG_LEVEL_* or a number")
    endif()
    target_sources(microstrain_common PRIVATE "${CMAKE_CURRENT_LIST_DIR}/logging.c")
    message("MicroStrain logging is enabled. Max level = '${MICROSTRAIN_LOGGING_MAX_LEVEL}'")
    target_compile_definitions(microstrain_common PUBLIC "MICROSTRAIN_ENABLE_LOGGING" "MICROSTRAIN_LOGGING_MAX_LEVEL=${MICROSTRAIN_LOGGING_MAX_LEVEL}")
endif()

target_include_directories(microstrain_common INTERFACE ${SRC_C_DIR})

# Disable windows defined min/max
# Set Windows header version (0x0501 is _WIN32_WINNT_WINXP, required for TCP)
if(WIN32)
    target_compile_definitions(microstrain_common
        PUBLIC
        "NOMINMAX=1"
        "_WIN32_WINNT=_WIN32_WINNT_WINXP"
    )
endif()
