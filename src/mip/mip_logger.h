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
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param level   The log level that this log should be logged at
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
typedef void (*mip_log_callback)(const void* context, void* user, mip_log_level level, const char* fmt, va_list args);

extern mip_log_callback _mip_log_callback;
extern mip_log_level _mip_log_level;
extern void* _mip_log_user_data;

void mip_logging_init(mip_log_callback callback, mip_log_level level, void* user);

mip_log_callback mip_logging_callback();
mip_log_level mip_logging_level();
void* mip_logging_user_data();

void mip_logging_log(const void* context, mip_log_level level, const char* fmt, ...);

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK. Prefer specific
///       log level functions like MIP_LOG_INFO, etc. when possible.
///@copydetails mip::C::mip_log_callback
///
#ifdef MIP_ENABLE_LOGGING
#define MIP_LOG_LOG(context, level, fmt, ...) mip_logging_log(context, level, fmt, __VA_ARGS__)
#else
#define MIP_LOG_LOG(context, level, fmt, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at fatal level
///
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
#define MIP_LOG_FATAL(context, fmt, ...) MIP_LOG_LOG(context, MIP_LOG_LEVEL_FATAL, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at error level
///@copydetails mip::C::MIP_LOG_FATAL
#define MIP_LOG_ERROR(context, fmt, ...) MIP_LOG_LOG(context, MIP_LOG_LEVEL_ERROR, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at warn level
///@copydetails mip::C::MIP_LOG_FATAL
#define MIP_LOG_WARN(context, fmt, ...) MIP_LOG_LOG(context, MIP_LOG_LEVEL_WARN, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at info level
///@copydetails mip::C::MIP_LOG_FATAL
#define MIP_LOG_INFO(context, fmt, ...) MIP_LOG_LOG(context, MIP_LOG_LEVEL_INFO, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at debug level
///@copydetails mip::C::MIP_LOG_FATAL
#define MIP_LOG_DEBUG(context, fmt, ...) MIP_LOG_LOG(context, MIP_LOG_LEVEL_DEBUG, fmt, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at trace level
///@copydetails mip::C::MIP_LOG_FATAL
#define MIP_LOG_TRACE(context, fmt, ...) MIP_LOG_LOG(context, MIP_LOG_LEVEL_TRACE, fmt, __VA_ARGS__)

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
///@brief C++ alias for mip::C::mip_log_level
using LoggingLevel = C::mip_log_level;



////////////////////////////////////////////////////////////////////////////////
///@brief C++ class usable by other MIP SDK classes to log information
///
class Logging
{
private:
    ///@brief Helper struct to mark function parameters as unused.
    ///       Required for parameters that are variadic template arguments
    struct unused
    {
        template<typename ...Args>
        unused(Args const & ... ) {}
    };

public:
    ///@copydoc mip::C::mip_logging_init
    static inline void initialize(mip::C::mip_log_callback callback, LoggingLevel level = LoggingLevel::MIP_LOG_LEVEL_INFO, void* user = nullptr) { mip_logging_init(callback, level, user); }

    ///@brief Helper function to log a string with varargs using the MIP SDK logger.
    ///       Prefer using specific functions like fatal, info, etc.
    ///@param context The context of the object doing the logging
    ///@param level   The log level that this log should be logged at
    ///@param fmt     printf style format string
    ///@param ...args Variadic args used to populate the fmt string
    template<class... Args>
    static void log(const void* context, LoggingLevel level, const char* fmt, Args&&... args)
    {
        using mip::C::mip_logging_log;
        MIP_LOG_LOG(context, level, fmt, args...);
        unused{context, level, fmt, args...};  // If logging is turned off
    }

    ///@brief Helper function to log a string with varargs at fatal level
    ///@param fmt     printf style format string
    ///@param ...args Variadic args used to populate the fmt string
    template<class... Args>
    static void fatal(const void* context, const char* fmt, Args&&... args) { log(context, LoggingLevel::MIP_LOG_LEVEL_FATAL, fmt, args...); }

    ///@brief Helper function to log a string with varargs at error level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    static void error(const void* context, const char* fmt, Args&&... args) { log(context, LoggingLevel::MIP_LOG_LEVEL_ERROR, fmt, args...); }

    ///@brief Helper function to log a string with varargs at warn level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    static void warn(const void* context, const char* fmt, Args&&... args) { log(context, LoggingLevel::MIP_LOG_LEVEL_WARN, fmt, args...); }

    ///@brief Helper function to log a string with varargs at info level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    static void info(const void* context, const char* fmt, Args&&... args) { log(context, LoggingLevel::MIP_LOG_LEVEL_INFO, fmt, args...); }

    ///@brief Helper function to log a string with varargs at debug level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    static void debug(const void* context, const char* fmt, Args&&... args) { log(context, LoggingLevel::MIP_LOG_LEVEL_DEBUG, fmt, args...); }

    ///@brief Helper function to log a string with varargs at trace level
    ///@copydetails mip::Logger::fatal
    template<class... Args>
    static void trace(const void* context, const char* fmt, Args&&... args) { log(context, LoggingLevel::MIP_LOG_LEVEL_TRACE, fmt, args...); }
};

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace mip
#endif