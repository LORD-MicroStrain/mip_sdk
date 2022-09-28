
#include "mip_logger.h"

#include <stddef.h>

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging callback. Do not access directly
mip_logger_callback _mip_logger_callback  = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging user data. Do not access directly
void* _mip_logger_user_data = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the logger with a callback and user data
///
///@param callback The callback to execute when there is data to log
///@param user     User data that will be passed to the callback every time it is excuted
///
void mip_logger_init(mip_logger_callback callback, void* user)
{
  _mip_logger_callback = callback;
  _mip_logger_user_data = user;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logger callback
///
///@return The currently active logger callback
///
mip_logger_callback mip_logger_get_callback()
{
  return _mip_logger_callback;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logger user data
///
///@return The currently active logger user data
///
void* mip_logger_get_user_data()
{
  return _mip_logger_user_data;
}