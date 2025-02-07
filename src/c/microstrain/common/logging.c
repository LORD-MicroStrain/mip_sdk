
#include "logging.h"

#include <stddef.h>

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging callback. Do not access directly
microstrain_log_callback microstrain_log_callback_ = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging level. Do not access directly
microstrain_log_level microstrain_log_level_ = MICROSTRAIN_LOG_LEVEL_OFF;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging user data. Do not access directly
void* microstrain_log_user_data_ = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the logger with a callback and user data.
///       Call MICROSTRAIN_LOG_INIT instead of using this function directly.
///       This function does not have to be called unless the user wants logging
///
///@param callback The callback to execute when there is data to log
///@param level    The level that the MicroStrain SDK should log at
///@param user     User data that will be passed to the callback every time it
///                is executed
///
void microstrain_logging_init(const microstrain_log_callback callback, const microstrain_log_level level, void* user)
{
    microstrain_log_callback_  = callback;
    microstrain_log_level_     = level;
    microstrain_log_user_data_ = user;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging callback
///
///@return The currently active logging callback
///
microstrain_log_callback microstrain_logging_callback(void)
{
    return microstrain_log_callback_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging level
///
///@return The currently active logging level
///
microstrain_log_level microstrain_logging_level(void)
{
    return microstrain_log_level_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging user data
///
///@return The currently active logging user data
///
void* microstrain_logging_user_data(void)
{
    return microstrain_log_user_data_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Internal log function called by variadic logging macros.
///       Call MICROSTRAIN_LOG_*_V macros instead of using this function
///       directly
///@copydetails microstrain_log_callback
///
void microstrain_logging_log_v(const microstrain_log_level level, const char* fmt, va_list args)
{
    const microstrain_log_callback callback = microstrain_logging_callback();
    if(callback != NULL && microstrain_logging_level() >= level)
    {
        callback(microstrain_logging_user_data(), level, fmt, args);
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Internal log function called by logging macros.
///       Call MICROSTRAIN_LOG_* macros instead of using this function directly
///@copydetails microstrain_log_callback
///
void microstrain_logging_log(const microstrain_log_level level, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    microstrain_logging_log_v(level, fmt, args);
    va_end(args);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a string representing the given log level.
///@param level
///
/// The returned string is statically-allocated and shall not be freed nor
/// modified. The strings are padded to a uniform length for consistent
/// alignment in log files.
///
const char* microstrain_logging_level_name(const microstrain_log_level level)
{
    switch(level)
    {
    case MICROSTRAIN_LOG_LEVEL_OFF:   return "OFF  ";
    case MICROSTRAIN_LOG_LEVEL_FATAL: return "FATAL";
    case MICROSTRAIN_LOG_LEVEL_ERROR: return "ERROR";
    case MICROSTRAIN_LOG_LEVEL_WARN:  return "WARN ";
    case MICROSTRAIN_LOG_LEVEL_INFO:  return "INFO ";
    case MICROSTRAIN_LOG_LEVEL_DEBUG: return "DEBUG";
    case MICROSTRAIN_LOG_LEVEL_TRACE: return "TRACE";
    default: return "INVALID";
    }
}
