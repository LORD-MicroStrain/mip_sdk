#pragma once

#include <microstrain/logging.h>

#include <microstrain/span.hpp>

#include <string>


namespace microstrain {
namespace logging {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_logging_cpp
///@{

    enum class LogLevel : ::microstrain::C::microstrain_log_level
    {
        OFF   = MICROSTRAIN_LOG_LEVEL_OFF,
        FATAL = MICROSTRAIN_LOG_LEVEL_FATAL,
        ERROR = MICROSTRAIN_LOG_LEVEL_ERROR,
        WARN  = MICROSTRAIN_LOG_LEVEL_WARN,
        INFO  = MICROSTRAIN_LOG_LEVEL_INFO,
        DEBUG = MICROSTRAIN_LOG_LEVEL_DEBUG,
        TRACE = MICROSTRAIN_LOG_LEVEL_TRACE,
    };

    using Callback = ::microstrain::C::microstrain_log_callback;

    inline void init(Callback callback, LogLevel max_level, void* user=nullptr)
    {
        return MICROSTRAIN_LOG_INIT(
            callback,
            static_cast<::microstrain::C::microstrain_log_level>(max_level),
            user
        );
    }

    inline Callback get_callback()
    {
#if MICROSTRAIN_ENABLE_LOGGING
        return ::microstrain::C::microstrain_logging_callback();
#else
        return nullptr;
#endif
    }

    inline LogLevel max_level()
    {
    #if MICROSTRAIN_ENABLE_LOGGING
       return static_cast<LogLevel>(
           ::microstrain::C::microstrain_logging_level()
       );
    #else
       return LogLevel::OFF;
    #endif
    }

    inline void* user_pointer()
    {
#if MICROSTRAIN_ENABLE_LOGGING
        return ::microstrain::C::microstrain_logging_user_data();
#else
        return nullptr;
#endif
    }

    inline void log(LogLevel level, const char* fmt, ...)
    {
#if MICROSTRAIN_ENABLE_LOGGING
        va_list args;
        va_start(args, fmt);
        ::microstrain::C::microstrain_logging_log_v(
            static_cast<::microstrain::C::microstrain_log_level>(level),
            fmt, args
        );
        va_end(args);
#endif
    }

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace logging
} // namespace microstrain
