#pragma once

#include <microstrain/logging.h>

#include <microstrain/span.hpp>

#include <string>


namespace microstrain
{

namespace logging
{
    enum class LogLevel : C::microstrain_log_level
    {
        OFF   = MICROSTRAIN_LOG_LEVEL_OFF,
        FATAL = MICROSTRAIN_LOG_LEVEL_FATAL,
        ERROR = MICROSTRAIN_LOG_LEVEL_ERROR,
        WARN  = MICROSTRAIN_LOG_LEVEL_WARN,
        INFO  = MICROSTRAIN_LOG_LEVEL_INFO,
        DEBUG = MICROSTRAIN_LOG_LEVEL_DEBUG,
        TRACE = MICROSTRAIN_LOG_LEVEL_TRACE,
    };

    using Callback = C::microstrain_log_callback;

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

    void* user_pointer()
    {
#if MICROSTRAIN_ENABLE_LOGGING
        return ::microstrain::C::microstrain_logging_user_data();
#else
        return nullptr;
#endif
    }

    void log(LogLevel level, const char* fmt, ...)
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
}



///@copydoc microstrain::C::microstrain_strcat_n
///
inline bool strcat_n(char* buffer, size_t buffer_size, size_t* index, const char* str, size_t len)
{
    return C::microstrain_strcat_n(buffer, buffer_size, index, str, len);
}

///@brief Strcat overload taking a span for the buffer.
///
inline bool strcat_n(Span<char> buffer, size_t* index, const char* str, size_t len)
{
    return strcat_n(buffer.data(), buffer.size(), index, str, len);
}

///@brief strcat overload taking a span for the appended string.
///
inline bool strcat(Span<char> buffer, size_t* index, Span<const char> str)
{
    return strcat_n(buffer, index, str.data(), str.size());
}

///@brief strcat overload taking a std::string_view.
///
inline bool strcat(Span<char> buffer, size_t* index, std::string_view str)
{
    return strcat_n(buffer, index, str.data(), str.size());
}

///@brief strcat overload taking a std::string.
///
inline bool strcat(Span<char> buffer, size_t* index, const std::string& str)
{
    return strcat_n(buffer, index, str.data(), str.size());
}

///@brief strcat overload taking a string literal or array.
///
/// Use this by passing a string literal directly so that the compiler
/// is able to deduce the size of the string. This avoids the need to
/// call std::strlen.
///
template<size_t N>
bool strcat_l(Span<char> buffer, size_t* index, const char(&str)[N])
{
    return strcat_n(buffer, index, str, N-1);
}



///@copydoc microstrain::C::microstrain_strfmt_v
///
inline bool strfmt_v(char* buffer, size_t buffer_size, size_t* index, const char* fmt, va_list args)
{
    return C::microstrain_strfmt_v(buffer, buffer_size, index, fmt, args);
}

///@brief strfmt_v overload taking a span for the buffer.
///
inline bool strfmt_v(Span<char> buffer, size_t* index, const char* fmt, va_list args)
{
    return strfmt_v(buffer.data(), buffer.size(), index, fmt, args);
}

///@copydoc microstrain::C::microstrain_strfmt
///
inline bool strfmt(char* buffer, size_t buffer_size, size_t* index, const char* fmt, ...)
{
    va_list args;
        va_start(args, fmt);
    bool ok = strfmt_v(buffer, buffer_size, index, fmt, args);
        va_end(args);
    return ok;
}

///@brief strfmt overload taking a span for the buffer.
///
inline bool strfmt(Span<char> buffer, size_t* index, const char* fmt, ...)
{
    va_list args;
        va_start(args, fmt);
    bool ok = strfmt_v(buffer.data(), buffer.size(), index, fmt, args);
        va_end(args);
    return ok;
}


///@copydoc microstrain::C::microstrain_strfmt_bytes
///
inline bool strfmt_bytes(char* buffer, size_t buffer_size, size_t* index, const uint8_t* data, size_t len, unsigned int byte_grouping)
{
    return C::microstrain_strfmt_bytes(buffer, buffer_size, index, data, len, byte_grouping);
}

///@brief strfmt overload taking a span for the buffer.
///
inline bool strfmt_bytes(Span<char> buffer, size_t* index, const uint8_t* data, size_t len, unsigned int byte_grouping)
{
    return strfmt_bytes(buffer.data(), buffer.size(), index, data, len, byte_grouping);
}

///@brief strfmt overload taking a span for the buffer and data.
///
inline bool strfmt_bytes(Span<char> buffer, size_t* index, Span<const uint8_t> data, unsigned int byte_grouping)
{
    return strfmt_bytes(buffer.data(), buffer.size(), index, data.data(), data.size(), byte_grouping);
}

} // namespace microstrain
