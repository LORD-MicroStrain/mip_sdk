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
#if MICROSTRAIN_ENABLE_LOGGING
        return ::microstrain::C::microstrain_logging_callback();
#else
        return nullptr;
#endif
    }

    inline Level max_level()
    {
    #if MICROSTRAIN_ENABLE_LOGGING
       return ::microstrain::C::microstrain_logging_level();
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

    inline void log(Level level, const char* fmt, ...)
    {
#if MICROSTRAIN_ENABLE_LOGGING
        va_list args;
        va_start(args, fmt);
        ::microstrain::C::microstrain_logging_log_v(level, fmt, args);
        va_end(args);
#endif
    }

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace logging
} // namespace microstrain
*/