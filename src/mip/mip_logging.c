
#include "mip_logging.h"

#include <stddef.h>

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging callback. Do not access directly
mip_log_callback _mip_log_callback = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging level. Do not access directly
mip_log_level _mip_log_level = MIP_LOG_LEVEL_OFF;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging user data. Do not access directly
void* _mip_log_user_data = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the logger with a callback and user data.
///       Call MIP_LOG_INIT instead of using this function directly
///
///@param callback The callback to execute when there is data to log
///@param level    The level that the MIP SDK should log at
///@param user     User data that will be passed to the callback every time it is excuted
///
void _mip_logging_init(mip_log_callback callback, mip_log_level level, void* user)
{
  _mip_log_callback = callback;
  _mip_log_level = level;
  _mip_log_user_data = user;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging callback
///
///@return The currently active logging callback
///
mip_log_callback _mip_logging_callback()
{
  return _mip_log_callback;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging level
///
///@return The currently active logging level
///
mip_log_level _mip_logging_level()
{
  return _mip_log_level;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging user data
///
///@return The currently active logging user data
///
void* _mip_logging_user_data()
{
  return _mip_log_user_data;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Internal log function called by logging macros.
///       Call MIP_LOG_* macros instead of using this function directly
///@copydetails mip::C::mip_log_callback
///
void mip_logging_log(mip_log_level level, const char* fmt, ...)
{
  const mip_log_callback logging_callback = _mip_logging_callback();
  const mip_log_level logging_level = _mip_logging_level();
  if (logging_callback != NULL && logging_level >= level)
  {
    va_list args;
    va_start(args, fmt);
    logging_callback(_mip_logging_user_data(), level, fmt, args);
    va_end(args);
  }
}