#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup microstrain_logging  MicroStrain Logging
///
///@brief High-level functions for logging information from within the
///       MicroStrain SDK
///
/// This module contains functions that allow the MicroStrain SDK to log
/// information and allows users to override the logging functions
///
///@{
///@defgroup microstrain_logging_c   MicroStrain Logging [C]
///@defgroup microstrain_logging_cpp MicroStrain Logging [CPP]
///@}
///

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_logging_c
///
///@brief Logging functions in C.
///
///@{

#define MICROSTRAIN_LOGGING_LEVEL_OFF_   0
#define MICROSTRAIN_LOGGING_LEVEL_FATAL_ 1
#define MICROSTRAIN_LOGGING_LEVEL_ERROR_ 2
#define MICROSTRAIN_LOGGING_LEVEL_WARN_  3
#define MICROSTRAIN_LOGGING_LEVEL_INFO_  4
#define MICROSTRAIN_LOGGING_LEVEL_DEBUG_ 5
#define MICROSTRAIN_LOGGING_LEVEL_TRACE_ 6

#ifndef MICROSTRAIN_LOGGING_MAX_LEVEL
// Define a default logging level
#define MICROSTRAIN_LOGGING_MAX_LEVEL MICROSTRAIN_LOGGING_LEVEL_OFF_
#endif // !MICROSTRAIN_LOGGING_MAX_LEVEL

#if MICROSTRAIN_LOGGING_MAX_LEVEL > MICROSTRAIN_LOGGING_LEVEL_OFF_
#define MICROSTRAIN_LOGGING_ENABLED
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL > MICROSTRAIN_LOGGING_LEVEL_OFF_

#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_FATAL_
#define MICROSTRAIN_LOGGING_ENABLED_FATAL
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_FATAL_

#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_ERROR_
#define MICROSTRAIN_LOGGING_ENABLED_ERROR
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_ERROR_

#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_WARN_
#define MICROSTRAIN_LOGGING_ENABLED_WARN
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_WARN_

#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_INFO_
#define MICROSTRAIN_LOGGING_ENABLED_INFO
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_INFO_

#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_DEBUG_
#define MICROSTRAIN_LOGGING_ENABLED_DEBUG
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_DEBUG_

#if MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_TRACE_
#define MICROSTRAIN_LOGGING_ENABLED_TRACE
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL >= MICROSTRAIN_LOGGING_LEVEL_TRACE_

#define MICROSTRAIN_LOG_UNUSED_V(...) do { if (0) { (void)sizeof(()int[]){__VA_ARGS__}); } } while (0)

////////////////////////////////////////////////////////////////////////////////
///@brief Logging level enum
///
typedef enum microstrain_log_level
{
    MICROSTRAIN_LOG_LEVEL_OFF   = MICROSTRAIN_LOGGING_LEVEL_OFF_,   ///< Signifies that the log is turned off
    MICROSTRAIN_LOG_LEVEL_FATAL = MICROSTRAIN_LOGGING_LEVEL_FATAL_, ///< Fatal logs are logged when an unrecoverable error occurs
    MICROSTRAIN_LOG_LEVEL_ERROR = MICROSTRAIN_LOGGING_LEVEL_ERROR_, ///< Error logs are logged when an error occurs
    MICROSTRAIN_LOG_LEVEL_WARN  = MICROSTRAIN_LOGGING_LEVEL_WARN_,  ///< Warning logs are logged when something concerning happens that may or not be a mistake
    MICROSTRAIN_LOG_LEVEL_INFO  = MICROSTRAIN_LOGGING_LEVEL_INFO_,  ///< Info logs are logged when some general info needs to be conveyed to the user
    MICROSTRAIN_LOG_LEVEL_DEBUG = MICROSTRAIN_LOGGING_LEVEL_DEBUG_, ///< Debug logs are logged for debug purposes.
    MICROSTRAIN_LOG_LEVEL_TRACE = MICROSTRAIN_LOGGING_LEVEL_TRACE_, ///< Trace logs are logged in similar cases to debug logs but can be logged in tight loops
} microstrain_log_level;

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom logging behavior.
///
///@param user  User data pointer
///@param level The log level that this log should be logged at
///@param fmt   printf style format string
///@param args  Variadic args used to populate the fmt string
///
typedef void (*microstrain_log_callback)(void* user, const microstrain_log_level level, const char* fmt, va_list args);

void microstrain_logging_init(const microstrain_log_callback callback, const microstrain_log_level level, void* user);

microstrain_log_callback microstrain_logging_callback(void);

microstrain_log_level microstrain_logging_level(void);

void* microstrain_logging_user_data(void);

void microstrain_logging_log_v(const microstrain_log_level level, const char* fmt, va_list args);

void microstrain_logging_log(const microstrain_log_level level, const char* fmt, ...);

const char* microstrain_logging_level_name(const microstrain_log_level level);

void microstrain_log_bytes(const microstrain_log_level level, const char* msg, const uint8_t* data, size_t length);

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to initialize the MicroStrain logger.
///       This function does not need to be called unless the user wants
///       logging
///
///@param callback The callback to execute when there is data to log
///@param level    The level that the MicroStrain SDK should log at
///@param user     User data that will be passed to the callback every time it
///                is executed
///
#ifdef MICROSTRAIN_LOGGING_ENABLED
#define MICROSTRAIN_LOG_INIT(callback, level, user) microstrain_logging_init(callback, level, user)
#else // !MICROSTRAIN_LOGGING_ENABLED
#define MICROSTRAIN_LOG_INIT(callback, level, user) (void)0; (void)callback; (void)level; (void)user
#endif // MICROSTRAIN_LOGGING_ENABLED

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK. Prefer
///       specific log level functions like MICROSTRAIN_LOG_INFO, etc. when
///       possible.
///@copydetails microstrain_log_callback
///
#ifdef MICROSTRAIN_LOGGING_ENABLED
#define MICROSTRAIN_LOG_LOG(level, ...) microstrain_logging_log(level, __VA_ARGS__)
#define MICROSTRAIN_LOG_LOG_V(level, fmt, args) microstrain_logging_log_v(level, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED
#define MICROSTRAIN_LOG_LOG(level, ...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_LOG_V(level, fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at fatal
///       level
///
///@param context Context of what called this function. Could be a mip device,
///               serial connection, etc.
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_FATAL
#define MICROSTRAIN_LOG_FATAL(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_FATAL, __VA_ARGS__)
#define MICROSTRAIN_LOG_FATAL_V(fmt, args) MICROSTRAIN_LOG_LOG_V(MICROSTRAIN_LOG_LEVEL_FATAL, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED_FATAL
#define MICROSTRAIN_LOG_FATAL(...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_FATAL_V(fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED_FATAL

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at error
///       level
///@copydetails MICROSTRAIN_LOG_FATAL
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_ERROR
#define MICROSTRAIN_LOG_ERROR(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_ERROR, __VA_ARGS__)
#define MICROSTRAIN_LOG_ERROR_V(fmt, args) MICROSTRAIN_LOG_LOG_V(MICROSTRAIN_LOG_LEVEL_ERROR, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED_ERROR
#define MICROSTRAIN_LOG_ERROR(...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_ERROR_V(fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED_ERROR

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at warn level
///@copydetails MICROSTRAIN_LOG_FATAL
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_WARN
#define MICROSTRAIN_LOG_WARN(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_WARN, __VA_ARGS__)
#define MICROSTRAIN_LOG_WARN_V(fmt, args) MICROSTRAIN_LOG_LOG_V(MICROSTRAIN_LOG_LEVEL_WARN, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED_WARN
#define MICROSTRAIN_LOG_WARN(...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_WARN_V(fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED_WARN

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at info level
///@copydetails MICROSTRAIN_LOG_FATAL
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_INFO
#define MICROSTRAIN_LOG_INFO(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_INFO, __VA_ARGS__)
#define MICROSTRAIN_LOG_INFO_V(fmt, args) MICROSTRAIN_LOG_LOG_V(MICROSTRAIN_LOG_LEVEL_INFO, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED_INFO
#define MICROSTRAIN_LOG_INFO(...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_INFO_V(fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED_INFO

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at debug
///       level
///@copydetails MICROSTRAIN_LOG_FATAL
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_DEBUG
#define MICROSTRAIN_LOG_DEBUG(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define MICROSTRAIN_LOG_DEBUG_V(fmt, args) MICROSTRAIN_LOG_LOG_V(MICROSTRAIN_LOG_LEVEL_DEBUG, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED_DEBUG
#define MICROSTRAIN_LOG_DEBUG(...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_DEBUG_V(fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED_DEBUG

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MicroStrain SDK at trace
///       level
///@copydetails MICROSTRAIN_LOG_FATAL
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_TRACE
#define MICROSTRAIN_LOG_TRACE(...) MICROSTRAIN_LOG_LOG(MICROSTRAIN_LOG_LEVEL_TRACE, __VA_ARGS__)
#define MICROSTRAIN_LOG_TRACE_V(fmt, args) MICROSTRAIN_LOG_LOG_V(MICROSTRAIN_LOG_LEVEL_TRACE, fmt, args)
#else // !MICROSTRAIN_LOGGING_ENABLED_TRACE
#define MICROSTRAIN_LOG_TRACE(...) MICROSTRAIN_LOG_UNUSED_V(__VA_ARGS__)
#define MICROSTRAIN_LOG_TRACE_V(fmt, args) (void)0; (void)fmt; (void)args
#endif // MICROSTRAIN_LOGGING_ENABLED_TRACE

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log bytes as a hex sting
///
#ifdef MICROSTRAIN_LOGGING_ENABLED
#define MICROSTRAIN_LOG_BYTES(level, msg, data, length) microstrain_log_bytes(level, msg, data, length)
#else // !MICROSTRAIN_LOGGING_ENABLED
#define MICROSTRAIN_LOG_BYTES(level, msg, data, length) (void)0; (void)level; (void)msg; (void)data; (void)length
#endif // MICROSTRAIN_LOGGING_ENABLED

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log bytes as a hex sting at trace level
///
/// This optimizes out to a NO-OP when MICROSTRAIN_LOGGING_ENABLED_TRACE is not
/// defined.
///
#ifdef MICROSTRAIN_LOGGING_ENABLED_TRACE
#define MICROSTRAIN_LOG_BYTES_TRACE(msg, data, length) microstrain_log_bytes(MICROSTRAIN_LOG_LEVEL_TRACE, msg, data, length)
#else // !MICROSTRAIN_LOGGING_ENABLED_TRACE
#define MICROSTRAIN_LOG_BYTES_TRACE(msg, data, length) (void)0; (void)msg; (void)data; (void)length
#endif // MICROSTRAIN_LOGGING_ENABLED_TRACE


////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log an error message from a syscall.
///@param msg A plain C-string without any printf-style formatters.
/// The resulting log message will be
/// "<message here>: <error-code> <error-description>\n".
///
#define MICROSTRAIN_LOG_ERROR_WITH_ERRNO(msg) MICROSTRAIN_LOG_ERROR(msg ": %d %s\n", errno, strerror(errno))

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log an error message from a syscall.
///@param msg A plain C-string which includes one or more printf-style
///           formatters.
///@param ... Arguments corresponding to the format codes in msg.
/// The resulting log message will be
/// "<message here>: <error-code> <error-description>\n".
///
#define MICROSTRAIN_LOG_ERROR_WITH_ERRNO_EX(msg, ...) MICROSTRAIN_LOG_ERROR(msg ": %d %s\n", __VA_ARGS__, errno, strerror(errno))

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
