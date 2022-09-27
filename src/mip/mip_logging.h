#pragma once

#include <stdint.h>

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
///@brief High-level C functions for logging information from the MIP SDK
///
/// This module contains functions that allow the MIP SDK to log information
/// and allows users to override the logging functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Logging level enum
///
typedef enum mip_logging_level
{
    MIP_LOGGING_LEVEL_FATAL = 0,  ///< Fatal logs are logged when an unrecoverable error occurs
    MIP_LOGGING_LEVEL_ERROR = 1,  ///< Error logs are logged when an error occurs
    MIP_LOGGING_LEVEL_WARN  = 2,  ///< Warning logs are logged when something concerning happens that may or not be a mistake
    MIP_LOGGING_LEVEL_INFO  = 3,  ///< Info logs are logged when some general info needs to be conveyed to the user
    MIP_LOGGING_LEVEL_DEBUG = 4,  ///< Debug logs are logged for debug purposes.
    MIP_LOGGING_LEVEL_TRACE = 5,  ///< Trace logs are logged in similar cases to debug logs but can be logged in tight loops
} mip_logging_level;

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom logging behavior.
///
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param level   The log level that this log should be logged at
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
typedef void (*mip_logging_callback)(const void* context, void* user, mip_logging_level level, const char* fmt, ...);

extern mip_logging_callback _mip_logging_callback;
extern void* _mip_logging_user_data;

void mip_logging_init(mip_logging_callback callback, void* user);
mip_logging_callback mip_logging_get_callback();
void* mip_logging_get_user_data();

////////////////////////////////////////////////////////////////////////////////
///@def mip_logging_log
///@copydoc mip_logging_callback
#define mip_logging_log(context, level, fmt, ...) \
    if (mip_logging_get_callback() != NULL) \
        mip_logging_get_callback()(context, mip_logging_get_user_data(), level, fmt, __VA_ARGS__)
#define mip_logging_fatal(context, fmt, ...) mip_logging_log(context, MIP_LOGGING_LEVEL_FATAL, fmt, __VA_ARGS__)
#define mip_logging_error(context, fmt, ...) mip_logging_log(context, MIP_LOGGING_LEVEL_ERROR, fmt, __VA_ARGS__)
#define mip_logging_warn(context, fmt, ...) mip_logging_log(context, MIP_LOGGING_LEVEL_WARN, fmt, __VA_ARGS__)
#define mip_logging_info(context, fmt, ...) mip_logging_log(context, MIP_LOGGING_LEVEL_INFO, fmt, __VA_ARGS__)
#define mip_logging_debug(context, fmt, ...) mip_logging_log(context, MIP_LOGGING_LEVEL_DEBUG, fmt, __VA_ARGS__)
#define mip_logging_trace(context, fmt, ...) mip_logging_log(context, MIP_LOGGING_LEVEL_TRACE, fmt, __VA_ARGS__)

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C

using LoggingLevel = C::mip_logging_level;

class LoggingInterface
{
protected:
    template<class... Args>
    void log(LoggingLevel level, const char* fmt, Args&&... args) const
    {
        using mip::C::mip_logging_get_callback;
        using mip::C::mip_logging_get_user_data;
        mip_logging_log(this, level, fmt, args...);
    }

    template<class... Args>
    void fatal(const char* fmt, Args&&... args) const { log(LoggingLevel::MIP_LOGGING_LEVEL_FATAL, fmt, args...); }
};

inline void initLogging(mip::C::mip_logging_callback callback, void* user = nullptr) { mip_logging_init(callback, user); }

} // namespace mip
#endif