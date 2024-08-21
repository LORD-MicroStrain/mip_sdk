
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
///@param user     User data that will be passed to the callback every time it is excuted
///
void microstrain_logging_init(microstrain_log_callback callback, microstrain_log_level level, void* user)
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
microstrain_log_callback microstrain_logging_callback()
{
  return microstrain_log_callback_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging level
///
///@return The currently active logging level
///
microstrain_log_level microstrain_logging_level()
{
  return microstrain_log_level_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging user data
///
///@return The currently active logging user data
///
void* microstrain_logging_user_data()
{
  return microstrain_log_user_data_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Internal log function called by logging macros.
///       Call MICROSTRAIN_LOG_* macros instead of using this function directly
///@copydetails microstrain::C::microstrain_log_callback
///
void microstrain_logging_log(microstrain_log_level level, const char* fmt, ...)
{
  const microstrain_log_callback logging_callback = microstrain_logging_callback();
  const microstrain_log_level            logging_level    = microstrain_logging_level();
  if (logging_callback != NULL && logging_level >= level)
  {
    va_list args;
    va_start(args, fmt);
    logging_callback(microstrain_logging_user_data(), level, fmt, args);
    va_end(args);
  }
}