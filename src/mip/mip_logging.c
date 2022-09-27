
#include "mip_logging.h"

#include <stddef.h>

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging callback. Do not access directly
mip_logging_callback _mip_logging_callback  = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging user data. Do not access directly
void* _mip_logging_user_data = NULL;

void mip_logging_init(mip_logging_callback callback, void* user)
{
  _mip_logging_callback = callback;
  _mip_logging_user_data = user;
}

mip_logging_callback mip_logging_get_callback()
{
  return _mip_logging_callback;
}

void* mip_logging_get_user_data()
{
  return _mip_logging_user_data;
}