#pragma once

#include <microstrain/strings.h>
#include <microstrain/span.hpp>

#if MICROSTRAIN_HAS_STD_STRING
#if __cpp_lib_string_view >= 201606L
#include <string_view>
#endif
#include <string>
#endif

namespace microstrain {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_strings_cpp
///
///@brief String manipulation in C++.
///
///@{


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

#if MICROSTRAIN_HAS_STD_STRING
#if __cpp_lib_string_view >= 201606L
///@brief strcat overload taking a std::string_view.
///
/// You must define MICROSTRAIN_HAS_STD_STRING to enable this overload.
/// Your compiler must also support the string view feature test macro.
///
inline bool strcat(Span<char> buffer, size_t* index, std::string_view str)
{
    return strcat_n(buffer, index, str.data(), str.size());
}
#endif // __cpp_lib_string_view

///@brief strcat overload taking a std::string.
///
/// You must define MICROSTRAIN_HAS_STD_STRING to enable this overload.
///
inline bool strcat(Span<char> buffer, size_t* index, const std::string& str)
{
    return strcat_n(buffer, index, str.data(), str.size());
}
#endif // MICROSTRAIN_HAS_STD_STRING

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

#if MICROSTRAIN_ENABLE_LOGGING

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

#endif // MICROSTRAIN_ENABLE_LOGGING


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


///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace microstrain
