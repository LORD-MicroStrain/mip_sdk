#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
namespace microstrain {
namespace C {
extern "C" {
#endif


bool microstrain_strcat_n(char* buffer, size_t buffer_size, size_t* index, const char* str, size_t str_len);

#if MICROSTRAIN_ENABLE_LOGGING
// sprintf is too bloated for some embedded systems.
// Disable it when logging is disabled.
bool microstrain_strfmt_v(char* buffer, size_t buffer_size, size_t* index, const char* fmt, va_list args);
bool microstrain_strfmt(char* buffer, size_t buffer_size, size_t* index, const char* fmt, ...);
#endif // MICROSTRAIN_ENABLE_LOGGING

bool microstrain_strfmt_bytes(char* buffer, size_t buffer_size, size_t* index, const uint8_t* data, size_t data_size, unsigned int byte_grouping);

#define microstrain_strcat_l(buffer, buffer_size, index, str_lit) microstrain_strcat_n(buffer, buffer_size, index, str_lit, sizeof(str_lit)-1)


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace microstrain
#endif
