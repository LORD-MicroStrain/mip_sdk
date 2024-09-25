#pragma once

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_c
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup microstrain_logging  MicroStrain Logging [C]
///
///@brief High-level C functions for logging information from within the MicroStrain SDK
///
/// This module contains functions that allow the MicroStrain SDK to log information
/// and allows users to override the logging functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Logging level enum
///
typedef uint8_t microstrain_log_level;
#define MICROSTRAIN_LOG_LEVEL_OFF   0  ///< Signifies that the log is turned off
#define MICROSTRAIN_LOG_LEVEL_FATAL 1  ///< Fatal logs are logged when an unrecoverable error occurs
#define MICROSTRAIN_LOG_LEVEL_ERROR 2  ///< Error logs are logged when an error occurs
#define MICROSTRAIN_LOG_LEVEL_WARN  3  ///< Warning logs are logged when something concerning happens that may or not be a mistake
#define MICROSTRAIN_LOG_LEVEL_INFO  4  ///< Info logs are logged when some general info needs to be conveyed to the user
#define MICROSTRAIN_LOG_LEVEL_DEBUG 5  ///< Debug logs are logged for debug purposes.
#define MICROSTRAIN_LOG_LEVEL_TRACE 6  ///< Trace logs are logged in similar cases to debug logs but can be logged in tight loops

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom logging behavior.
///
///@param level   The log level that this log should be logged at
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
typedef void (*microstrain_log_callback)(void* user, microstrain_log_level level, const char* fmt, va_list args);

void microstrain_logging_init(microstrain_log_callback callback, microstrain_log_level level, void* user);

microstrain_log_callback microstrain_logging_callback(void);
microstrain_log_level microstrain_logging_level(void);
void* microstrain_logging_user_data(void);

void microstrain_logging_log(microstrain_log_level level, const char* fmt, ...);

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to initialize the MicroStrain logger.
///       This function does not need to be called unless the user wants logging
///
///@param callback The callback to execute when there is data to log
///@param level    The level that the MicroStrain SDK should log at
///@param user     User data that will be passed to the callback every time it is excuted
///
#ifdef MICROSTRAIN_ENABLE_LOGGING
#define MICROSTRAIN_LOG_INIT(callback, level, user) microstrain_logging_init(callback, level, user)
#else
#define MICROSTRAIN_LOG_INIT(callback, level, user) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK. Prefer specific
///       log level functions like MICROSTRAIN_LOG_INFO, etc. when possible.
///@copydetails microstrain_log_callback
///
#ifdef MICROSTRAIN_ENABLE_LOGGING
#define MICROSTRAIN_LOG_LOG(level, ...) microstrain_logging_log(level, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_LOG(level, ...) (void)0
#endif

#ifndef MICROSTRAIN_LOGGING_MAX_LEVEL
#define MICROSTRAIN_LOGGING_MAX_LEVEL MICROSTRAIN_LOG_LEVEL_WARN
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at fatal level
///
///@param context Context of what called this function. Could be a mip device, serial connection, etc.
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOG_LEVEL_FATAL
#define MICROSTRAIN_LOG_FATAL(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_FATAL, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_FATAL(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at error level
///@copydetails MICROSTRAIN_LOG_FATAL
#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOG_LEVEL_ERROR
#define MICROSTRAIN_LOG_ERROR(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_ERROR, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_ERROR(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at warn level
///@copydetails MICROSTRAIN_LOG_FATAL
#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOG_LEVEL_WARN
#define MICROSTRAIN_LOG_WARN(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_WARN, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_WARN(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at info level
///@copydetails MICROSTRAIN_LOG_FATAL
#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOG_LEVEL_INFO
#define MICROSTRAIN_LOG_INFO(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_INFO, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_INFO(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at debug level
///@copydetails MICROSTRAIN_LOG_FATAL
#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOG_LEVEL_DEBUG
#define MICROSTRAIN_LOG_DEBUG(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_DEBUG(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at trace level
///@copydetails MICROSTRAIN_LOG_FATAL
#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOG_LEVEL_TRACE
#define MICROSTRAIN_LOG_TRACE(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_TRACE, __VA_ARGS__)
#else
#define MICROSTRAIN_LOG_TRACE(...) (void)0
#endif

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
#endif
