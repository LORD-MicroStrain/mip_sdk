#pragma once

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_logging  Mip Logging [C]
///
///@brief High-level C functions for logging information from within the MIP SDK
///
/// This module contains functions that allow the MIP SDK to log information
/// and allows users to override the logging functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Logging level enum
///
typedef enum mip_log_level
{
    MIP_LOG_LEVEL_OFF   = 0,  ///< Signifies that the log is turned off
    MIP_LOG_LEVEL_FATAL = 1,  ///< Fatal logs are logged when an unrecoverable error occurs
    MIP_LOG_LEVEL_ERROR = 2,  ///< Error logs are logged when an error occurs
    MIP_LOG_LEVEL_WARN  = 3,  ///< Warning logs are logged when something concerning happens that may or not be a mistake
    MIP_LOG_LEVEL_INFO  = 4,  ///< Info logs are logged when some general info needs to be conveyed to the user
    MIP_LOG_LEVEL_DEBUG = 5,  ///< Debug logs are logged for debug purposes.
    MIP_LOG_LEVEL_TRACE = 6,  ///< Trace logs are logged in similar cases to debug logs but can be logged in tight loops
} mip_log_level;

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom logging behavior.
///
///@param level   The log level that this log should be logged at
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
typedef void (*mip_log_callback)(void* user, mip_log_level level, const char* fmt, va_list args);

extern mip_log_callback _mip_log_callback;
extern mip_log_level _mip_log_level;
extern void* _mip_log_user_data;

void mip_logging_init(mip_log_callback callback, mip_log_level level, void* user);

mip_log_callback mip_logging_callback();
mip_log_level mip_logging_level();
void* mip_logging_user_data();

void mip_logging_log(mip_log_level level, const char* fmt, ...);

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK. Prefer specific
///       log level functions like MIP_LOG_INFO, etc. when possible.
///@copydetails mip::C::mip_log_callback
///
#ifdef MIP_ENABLE_LOGGING
#define MIP_LOG_INIT(callback, level, user) mip_logging_init(callback, level, user)
#define MIP_LOG_LOG(level, fmt, ...) mip_logging_log(level, fmt, __VA_ARGS__)
#else
#define MIP_LOG_INIT(callback, level, user) (void)0
#define MIP_LOG_LOG(level, fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at fatal level
///
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= 1
#define MIP_LOG_FATAL(fmt, ...) MIP_LOG_LOG(MIP_LOG_LEVEL_FATAL, fmt, __VA_ARGS__)
#else
#define MIP_LOG_FATAL(fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at error level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= 2
#define MIP_LOG_ERROR(fmt, ...) MIP_LOG_LOG(MIP_LOG_LEVEL_ERROR, fmt, __VA_ARGS__)
#else
#define MIP_LOG_ERROR(fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at warn level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= 3
#define MIP_LOG_WARN(fmt, ...) MIP_LOG_LOG(MIP_LOG_LEVEL_WARN, fmt, __VA_ARGS__)
#else
#define MIP_LOG_WARN(fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at info level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= 4
#define MIP_LOG_INFO(fmt, ...) MIP_LOG_LOG(MIP_LOG_LEVEL_INFO, fmt, __VA_ARGS__)
#else
#define MIP_LOG_INFO(fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at debug level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= 5
#define MIP_LOG_DEBUG(fmt, ...) MIP_LOG_LOG(MIP_LOG_LEVEL_DEBUG, fmt, __VA_ARGS__)
#else
#define MIP_LOG_DEBUG(fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at trace level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= 6
#define MIP_LOG_TRACE(fmt, ...) MIP_LOG_LOG(MIP_LOG_LEVEL_TRACE, fmt, __VA_ARGS__)
#else
#define MIP_LOG_DEBUG(fmt, ...) (void)0
#endif

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif