#pragma once

#include <microstrain/logging.h>


namespace microstrain
{

//enum class LogLevel
//{
//    OFF   = MICROSTRAIN_LOG_LEVEL_OFF,
//    FATAL = MICROSTRAIN_LOG_LEVEL_FATAL,
//    ERROR = MICROSTRAIN_LOG_LEVEL_ERROR,
//    WARN  = MICROSTRAIN_LOG_LEVEL_WARN,
//    INFO  = MICROSTRAIN_LOG_LEVEL_INFO,
//    DEBUG = MICROSTRAIN_LOG_LEVEL_DEBUG,
//    TRACE = MICROSTRAIN_LOG_LEVEL_TRACE,
//};

//namespace logging
//{
//    using Callback = ::microstrain_log_callback;
//
//    void init(Callback callback, LogLevel max_level, void* user=nullptr);
//
//    Callback get_callback() { return ::microstrain_logging_callback(); }
//    LogLevel max_level() { return static_cast<LogLevel>(::microstrain_logging_level()); }
//    void* user_pointer() { return ::microstrain_logging_user_data(); }
//
//    //void log(LogLevel level, const char* fmt, ...) { ::microstrain_logging_log(level, fmt, args); }
//}

} // namespace microstrain
