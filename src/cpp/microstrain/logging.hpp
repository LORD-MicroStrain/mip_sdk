#pragma once

#include <microstrain/logging.h>

/*
namespace microstrain {
namespace logging {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_logging_cpp
///@{

    // enum LogLevel : ::microstrain::C::microstrain_log_level
    // {
    //     LEVEL_NONE  = MICROSTRAIN_LOG_LEVEL_OFF,
    //     LEVEL_FATAL = MICROSTRAIN_LOG_LEVEL_FATAL,
    //     LEVEL_ERROR = MICROSTRAIN_LOG_LEVEL_ERROR,
    //     LEVEL_WARN  = MICROSTRAIN_LOG_LEVEL_WARN,
    //     LEVEL_INFO  = MICROSTRAIN_LOG_LEVEL_INFO,
    //     LEVEL_DEBUG = MICROSTRAIN_LOG_LEVEL_DEBUG,
    //     LEVEL_TRACE = MICROSTRAIN_LOG_LEVEL_TRACE,
    // };
    using Level = ::microstrain::C::microstrain_log_level;

    using Callback = ::microstrain::C::microstrain_log_callback;

    inline void init(Callback callback, Level max_level, void* user=nullptr)
    {
        return MICROSTRAIN_LOG_INIT(callback, max_level, user);
    }

    inline Callback get_callback()
    {
#ifdef MICROSTRAIN_LOGGING_ENABLED
        return ::microstrain::C::microstrain_logging_callback();
#else // !MICROSTRAIN_LOGGING_ENABLED
        return nullptr;
#endif // MICROSTRAIN_LOGGING_ENABLED
    }

    inline Level max_level()
    {
#ifdef MICROSTRAIN_LOGGING_ENABLED
       return ::microstrain::C::microstrain_logging_level();
#else // !MICROSTRAIN_LOGGING_ENABLED
       return LogLevel::OFF;
#endif // MICROSTRAIN_LOGGING_ENABLED
    }

    inline void* user_pointer()
    {
#ifdef MICROSTRAIN_LOGGING_ENABLED
        return ::microstrain::C::microstrain_logging_user_data();
#else // !MICROSTRAIN_LOGGING_ENABLED
        return nullptr;
#endif // MICROSTRAIN_LOGGING_ENABLED
    }

    inline void log(Level level, const char* fmt, ...)
    {
#ifdef MICROSTRAIN_LOGGING_ENABLED
        va_list args;
        va_start(args, fmt);
        ::microstrain::C::microstrain_logging_log_v(level, fmt, args);
        va_end(args);
#endif // MICROSTRAIN_LOGGING_ENABLED
    }

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace logging
} // namespace microstrain
*/
