#pragma once

#include <microstrain/strings.h>
#include <microstrain/array_view.hpp>

#if MICROSTRAIN_HAS_STD_STRING
#if __cpp_lib_string_view >= 201606L
#include <string_view>
#endif
#include <string>
#endif

#include <cstring>


namespace microstrain {
namespace strings {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_strings_cpp
///
///@brief String manipulation in C++.
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a string into a buffer.
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. Cannot be NULL unless str_len is 0. Does NOT
///       require NULL termination, and any such termination is ignored. NULL
///       characters will be appended just like any other character.
///@param str_len
///       Length of string (number of characters to copy). Usually you would
///       set this to strlen(str). This overrides any NULL terminator in str.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
inline bool concat(CharArrayView buffer, size_t* index, const char* str, size_t len)
{
    return ::microstrain::C::microstrain_string_concat(buffer.data(), buffer.size(), index, str, len);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate an array of characters into a buffer.
///
/// Equivalent to `concat(buffer, buffer_size, index, str.data(), str.size());`.
/// This version accepts a Span of characters.
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. Embedded NULL chars are allowed; Use
///       concat_z for strings which are terminated before the end of the span.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
inline bool concat(CharArrayView buffer, size_t* index, ConstCharArrayView str)
{
    return concat(buffer, index, str.data(), str.size());
}

#if MICROSTRAIN_HAS_STD_STRING
#if __cpp_lib_string_view >= 201606L

////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a std::string_view into a buffer.
///
/// Equivalent to `concat(buffer, buffer_size, index, str.data(), str.size());`.
/// This version accepts a std::string_view.
///
///@note You must define MICROSTRAIN_HAS_STD_STRING to use this overload.
///@note You must have a compiler that supports std::string_view.
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. Embedded NULL chars are allowed.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
inline bool concat(CharArrayView buffer, size_t* index, std::string_view str)
{
    return concat(buffer, index, str.data(), str.size());
}
#endif // __cpp_lib_string_view

////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a std::string into a buffer.
///
/// Equivalent to `concat(buffer, buffer_size, index, str.data(), str.size());`.
/// This version accepts a std::string_view.
///
///@note You must define MICROSTRAIN_HAS_STD_STRING to use this overload.
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. Embedded NULL chars are allowed.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
inline bool concat(CharArrayView buffer, size_t* index, const std::string& str)
{
    return concat(buffer, index, str.data(), str.size());
}
#endif // MICROSTRAIN_HAS_STD_STRING


////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a NULL-terminated C string into a buffer.
///
/// Equivalent to `concat(buffer, buffer_size, index, str, strlen(str));`.
///
/// Copies up to `maxLen` characters, or the first NULL character is reached,
/// whichever comes first. maxLen can be used when it's uncertain if the source
/// buffer is properly terminated.
///
///Examples:
///@code{.cpp}
///  const char* str = "To be appended";
///  concat_z(buffer, &index, str);    // NULL-terminated, no maxlen
///  concat_z(buffer, &index, str, 5); // Only copies "To be"
///@endcode
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. NULL-termination is required unless maxLen
///       is given.
///@param maxLen
///       If given, limit to this many characters. Defaults to unlimited.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
inline bool concat_cstr(CharArrayView buffer, size_t* index, const char* str, size_t maxLen=size_t(-1))
{
    const size_t len = std::min(maxLen, std::strlen(str));
    return concat(buffer, index, ConstCharArrayView{str, len});
}

////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a string literal into a buffer.
///
/// Equivalent to `concat(buffer, buffer_size, index, str, sizeof(str)-1);`.
///
/// Use this by passing a string literal directly so that the compiler
/// is able to deduce the size of the string. This avoids the need to
/// call std::strlen.
///
///Example: `concat_l(buffer, &index, "append this string");`
///
/// Note that this also works with C arrays, in which case the entire array is
/// appended (NULLs and all).
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. Embedded NULL chars are allowed. Use
///       concat_z for strings which are terminated before N characters.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
template<size_t N>
bool concat_l(CharArrayView buffer, size_t* index, const char(&str)[N])
{
    return concat(buffer, index, str, N-1);
}

#if MICROSTRAIN_ENABLE_LOGGING

////////////////////////////////////////////////////////////////////////////////
///@copybrief microstrain::C::microstrain_string_format_v
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///@param fmt
///       Format string similar to printf.
///@param args
///       List of formatting arguments similar to vprintf.
///
///@returns True if successful
///@returns False if an encoding error occurs (see snprintf). The index is
///         unchanged in this case.
///@returns False if insufficient space is available, unless buffer is NULL.
///
inline bool format_v(CharArrayView buffer, size_t* index, const char* fmt, va_list args)
{
    return ::microstrain::C::microstrain_string_format_v(buffer.data(), buffer.size(), index, fmt, args);
}

////////////////////////////////////////////////////////////////////////////////
///@copybrief microstrain::C::microstrain_string_format
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///@param fmt
///       Format string similar to printf.
///
///@returns True if successful
///@returns False if an encoding error occurs (see snprintf). The index is
///         unchanged in this case.
///@returns False if insufficient space is available, unless buffer is NULL.
///
inline bool format(CharArrayView buffer, size_t* index, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    bool ok = format_v(buffer, index, fmt, args);
    va_end(args);
    return ok;
}

#endif // MICROSTRAIN_ENABLE_LOGGING


////////////////////////////////////////////////////////////////////////////////
///@brief Formats a byte array to a text buffer in hexadecimal.
///
/// No additional characters are printed other than the hex values and spaces
/// (if byte_grouping is positive). No leading or trailing space is printed.
///
/// Examples:
///@code{.cpp}
///
///@endcode
///
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///@param data
///       Data to be formatted. Can be NULL if data_size is 0.
///@param data_size
///       Number of bytes from data to print. Must be 0 if data is NULL.
///@param byte_grouping
///       If greater than zero, a space will be printed every byte_grouping
///       bytes. E.g. a group of 2 will print pairs of bytes separated by
///       spaces.
///
///@returns True if successful
///@returns False if an encoding error occurs (see snprintf). The index is
///         unchanged in this case.
///@returns False if insufficient space is available, unless buffer is NULL.
///
inline bool bytesToHexStr(CharArrayView buffer, size_t* index, ConstU8ArrayView data, unsigned int byte_grouping)
{
    return ::microstrain::C::microstrain_string_bytes_to_hex_str(buffer.data(), buffer.size(), index, data.data(), data.size(), byte_grouping);
}


///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace strings
} // namespace microstrain
