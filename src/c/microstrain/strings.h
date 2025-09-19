#pragma once

// TODO: Find better way to check for this
// Used to check if IO operations are supported
#include "logging.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
namespace microstrain {
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup microstrain_strings  MicroStrain Strings
///
///@brief High-level functions for manipulating %C strings
///
/// These functions wrap the standard str(n)cat and snprintf(_v) functions to
/// provide a more uniform and robust interface.
///
/// All of these functions work on a character buffer using an index value. The
/// index value is used to track the current position in the buffer. The
/// boolean return value allows easy error checking. These features make it easy
/// to chain multiple format calls in series without having to check for various
/// error conditions between steps.
///
///@{
///@defgroup microstrain_strings_c   MicroStrain Strings [C]
///@defgroup microstrain_strings_cpp MicroStrain Strings [CPP]
///@}
///

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_strings_c
///
///@brief String manipulation in C.
///
///@{

bool microstrain_string_concat(char* buffer, size_t buffer_size, size_t* index, const char* str, size_t str_len);
bool microstrain_string_concat_z(char* buffer, size_t buffer_size, size_t* index, const char* str);

#define microstrain_string_concat_l(buffer, buffer_size, index, str_lit) microstrain_string_concat(buffer, buffer_size, index, str_lit, sizeof(str_lit)-1)

// TODO: Use better check for this or move to extras
#ifdef MICROSTRAIN_LOGGING_ENABLED
// sprintf is too big for some embedded systems.
// Disable it when logging is disabled.
bool microstrain_string_format_v(char* buffer, size_t buffer_size, size_t* index, const char* fmt, va_list args);
bool microstrain_string_format(char* buffer, size_t buffer_size, size_t* index, const char* fmt, ...);
#endif // MICROSTRAIN_LOGGING_ENABLED

bool microstrain_string_bytes_to_hex_str(char* buffer, size_t buffer_size, size_t* index, const uint8_t* data, size_t data_size, unsigned int byte_grouping);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace microstrain
#endif
