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
///@defgroup mip_logger  Mip Logging [C]
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
typedef enum mip_logger_level
{
    MIP_LOGGER_LEVEL_OFF   = 0,  ///< Signifies that the log is turned off
    MIP_LOGGER_LEVEL_FATAL = 1,  ///< Fatal logs are logged when an unrecoverable error occurs
    MIP_LOGGER_LEVEL_ERROR = 2,  ///< Error logs are logged when an error occurs
    MIP_LOGGER_LEVEL_WARN  = 3,  ///< Warning logs are logged when something concerning happens that may or not be a mistake
    MIP_LOGGER_LEVEL_INFO  = 4,  ///< Info logs are logged when some general info needs to be conveyed to the user
    MIP_LOGGER_LEVEL_DEBUG = 5,  ///< Debug logs are logged for debug purposes.
    MIP_LOGGER_LEVEL_TRACE = 6,  ///< Trace logs are logged in similar cases to debug logs but can be logged in tight loops
} mip_logger_level;

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom logging behavior.
///
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param level   The log level that this log should be logged at
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
typedef void (*mip_logger_callback)(const void* context, void* user, mip_logger_level level, const char* fmt, ...);

extern mip_logger_callback _mip_logger_callback;
extern mip_logger_level _mip_logger_level;
extern void* _mip_logger_user_data;

void mip_logger_init(mip_logger_callback callback, mip_logger_level level, void* user);
mip_logger_callback mip_logger_get_callback();
mip_logger_level mip_logger_get_level();
void* mip_logger_get_user_data();

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK. Prefer specific
///       log level functions like mip_logger_info, etc. when possible.
///@copydetails mip::C::mip_logger_callback
#ifdef MIP_ENABLE_LOGGING
#define mip_logger_log(context, level, fmt, ...) \
    if (mip_logger_get_callback() != NULL && mip_logger_get_level() >= level) \
        mip_logger_get_callback()(context, mip_logger_get_user_data(), level, fmt, __VA_ARGS__)
#else
#define mip_logger_log(context, level, fmt, ...) do {} while (1)
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at fatal level
///
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
#define mip_logger_fatal(context, fmt, ...) mip_logger_log(context, MIP_LOGGER_LEVEL_FATAL, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at error level
///@copydetails mip::C::mip_logger_fatal
#define mip_logger_error(context, fmt, ...) mip_logger_log(context, MIP_LOGGER_LEVEL_ERROR, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at warn level
///@copydetails mip::C::mip_logger_fatal
#define mip_logger_warn(context, fmt, ...) mip_logger_log(context, MIP_LOGGER_LEVEL_WARN, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at info level
///@copydetails mip::C::mip_logger_fatal
#define mip_logger_info(context, fmt, ...) mip_logger_log(context, MIP_LOGGER_LEVEL_INFO, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at debug level
///@copydetails mip::C::mip_logger_fatal
#define mip_logger_debug(context, fmt, ...) mip_logger_log(context, MIP_LOGGER_LEVEL_DEBUG, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at trace level
///@copydetails mip::C::mip_logger_fatal
#define mip_logger_trace(context, fmt, ...) mip_logger_log(context, MIP_LOGGER_LEVEL_TRACE, fmt, __VA_ARGS__)

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_logger_cpp  Mip Logging [C++]
///
///@brief High-level C++ functions for logging information from within the MIP SDK
///
/// This module contains functions that allow the MIP SDK to log information
/// and allows users to override the logging functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief C++ alias for mip::C::mip_logger_level
using LoggerLevel = C::mip_logger_level;

////////////////////////////////////////////////////////////////////////////////
///@copydoc mip::C::mip_logger_init
inline void initLogging(mip::C::mip_logger_callback callback, LoggerLevel level = LoggerLevel::MIP_LOGGER_LEVEL_INFO, void* user = nullptr) { mip_logger_init(callback, level, user); }

////////////////////////////////////////////////////////////////////////////////
///@brief C++ class usable by other MIP SDK classes to log information
///
class Logger
{
public:
    ///@brief Creates a logger with a null context.
    ///
    Logger() = default;

    ///@brief Creates a logger with a context. The context should usually be the this pointer of whatever class created it
    ///
    Logger(const void* context) : mContext(context) {}

    ///@brief Helper function to log a string with varargs using the MIP SDK logger.
    ///       Prefer using specific functions like fatal, info, etc.
    ///@param level   The log level that this log should be logged at
    ///@param fmt     printf style format string
    ///@param ...args Variadic args used to populate the fmt string
    template<class... Args>
    void log(LoggerLevel level, const char* fmt, Args&&... args) const
    {
        using mip::C::mip_logger_get_callback;
        using mip::C::mip_logger_get_level;
        using mip::C::mip_logger_get_user_data;
        mip_logger_log(this, level, fmt, args...);
    }

    ///@brief Helper function to log a string with varargs at fatal level
    ///@param fmt     printf style format string
    ///@param ...args Variadic args used to populate the fmt string
    template<class... Args>
    void fatal(const char* fmt, Args&&... args) const { log(LoggerLevel::MIP_LOGGER_LEVEL_FATAL, fmt, args...); }

    ///@brief Helper function to log a string with varargs at error level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    void error(const char* fmt, Args&&... args) const { log(LoggerLevel::MIP_LOGGER_LEVEL_ERROR, fmt, args...); }

    ///@brief Helper function to log a string with varargs at warn level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    void warn(const char* fmt, Args&&... args) const { log(LoggerLevel::MIP_LOGGER_LEVEL_WARN, fmt, args...); }

    ///@brief Helper function to log a string with varargs at info level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    void info(const char* fmt, Args&&... args) const { log(LoggerLevel::MIP_LOGGER_LEVEL_INFO, fmt, args...); }

    ///@brief Helper function to log a string with varargs at debug level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    void debug(const char* fmt, Args&&... args) const { log(LoggerLevel::MIP_LOGGER_LEVEL_DEBUG, fmt, args...); }

    ///@brief Helper function to log a string with varargs at trace level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    void trace(const char* fmt, Args&&... args) const { log(LoggerLevel::MIP_LOGGER_LEVEL_TRACE, fmt, args...); }

private:
    const void* mContext = nullptr;
};

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace mip
#endif