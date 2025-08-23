
#include "logging.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Global logging callback. Do not access directly
microstrain_log_callback microstrain_log_callback_ = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging level. Do not access directly
microstrain_log_level microstrain_log_level_ = MICROSTRAIN_LOG_LEVEL_OFF;

////////////////////////////////////////////////////////////////////////////////
///@brief Global logging user data. Do not access directly
void* microstrain_log_user_data_ = NULL;

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the logger with a callback and user data.
///       Call MICROSTRAIN_LOG_INIT instead of using this function directly.
///       This function does not have to be called unless the user wants logging
///
///@param callback The callback to execute when there is data to log
///@param level    The level that the MicroStrain SDK should log at
///@param user     User data that will be passed to the callback every time it
///                is executed
///
void microstrain_logging_init(const microstrain_log_callback callback, const microstrain_log_level level, void* user)
{
    microstrain_log_callback_  = callback;
    microstrain_log_level_     = level;
    microstrain_log_user_data_ = user;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging callback
///
///@return The currently active logging callback
///
microstrain_log_callback microstrain_logging_callback(void)
{
    return microstrain_log_callback_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging level
///
///@return The currently active logging level
///
microstrain_log_level microstrain_logging_level(void)
{
    return microstrain_log_level_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the currently active logging user data
///
///@return The currently active logging user data
///
void* microstrain_logging_user_data(void)
{
    return microstrain_log_user_data_;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Internal log function called by variadic logging macros.
///       Call MICROSTRAIN_LOG_*_V macros instead of using this function
///       directly
///@copydetails microstrain_log_callback
///
void microstrain_logging_log_v(const microstrain_log_level level, const char* fmt, va_list args)
{
    const microstrain_log_callback callback = microstrain_logging_callback();
    if(callback != NULL && microstrain_logging_level() >= level)
    {
        callback(microstrain_logging_user_data(), level, fmt, args);
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Internal log function called by logging macros.
///       Call MICROSTRAIN_LOG_* macros instead of using this function directly
///@copydetails microstrain_log_callback
///
void microstrain_logging_log(const microstrain_log_level level, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    microstrain_logging_log_v(level, fmt, args);
    va_end(args);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a string representing the given log level.
///@param level
///
/// The returned string is statically-allocated and shall not be freed nor
/// modified. The strings are padded to a uniform length for consistent
/// alignment in log files.
///
const char* microstrain_logging_level_name(const microstrain_log_level level)
{
    switch(level)
    {
    case MICROSTRAIN_LOG_LEVEL_OFF:   return "OFF  ";
    case MICROSTRAIN_LOG_LEVEL_FATAL: return "FATAL";
    case MICROSTRAIN_LOG_LEVEL_ERROR: return "ERROR";
    case MICROSTRAIN_LOG_LEVEL_WARN:  return "WARN ";
    case MICROSTRAIN_LOG_LEVEL_INFO:  return "INFO ";
    case MICROSTRAIN_LOG_LEVEL_DEBUG: return "DEBUG";
    case MICROSTRAIN_LOG_LEVEL_TRACE: return "TRACE";
    default: return "INVALID";
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a string into a buffer.
///@param buffer
///       Pointer to buffer where str will be appended. Use NULL to just compute
///       the required size (buffer_size must be 0 in that case).
///@param buffer_size
///       Size of buffer. Must be 0 if buffer is NULL.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. Cannot be NULL unless str_len is 0. Does NOT
///       require NULL termination, and any NULL characters are ignored.
///@param str_len
///       Length of string (number of characters to copy). Usually you would
///       set this to strlen(str). This overrides any NULL terminator in str.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
bool microstrain_strcat_n(char* buffer, size_t buffer_size, size_t* index, const char* str, size_t str_len)
{
    const size_t offset = *index;

    *index += str_len;  // Index is always updated.

    // Case 1: Buffer already overrun (includes offset==buffer_size)
    // xxxxxxxxxxxxxxxxxxxxxxxxxx0###############
    // |                          |   |      |
    // 0                buffer_size  ofs  ofs+len
    //
    // Case 2: Buffer overruns if str_len too long
    // xxxxxxxxxxxxxxx0___________###############
    // |              |           |        |
    // 0             ofs    buffer_size  ofs+len
    //
    // Case 3: String fits entirely
    // xxxxxxxxxxxx0______________########
    // |           |      |       |
    // 0          ofs  ofs+len  buffer_size
    //

    // Not case 1: Do nothing if already overflowed.
    if(offset < buffer_size)
    {
        // At least one byte exists for the NULL terminator.
        assert(offset < buffer_size);

        // Handle case 2, where appending would cause overflow.
        // Convert to case 3 by limiting str_len to what will fit in the buffer.
        if(offset + str_len >= buffer_size)
            str_len = (buffer_size - 1) - offset;

        memcpy(buffer + offset, str, str_len);
        buffer[offset + str_len] = '\0';
    }

    return (buffer == NULL) || (*index < buffer_size);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Concatenate a string into a buffer.
///
/// Equivalent to `microstrain_strcat_n(buffer, buffer_size, index, str, strlen(str));`.
///
///@param buffer
///       Pointer to buffer where str will be appended. Use NULL to just compute
///       the required size (buffer_size must be 0 in that case).
///@param buffer_size
///       Size of buffer. Must be 0 if buffer is NULL.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index in all cases.
///@param str
///       String to be appended. NULL-termination is required.
///
///@returns True if sufficient buffer space exists or if buffer is NULL.
///@returns False if buffer is not NULL and insufficient space is available.
///
bool microstrain_strcat_c(char* buffer, size_t buffer_size, size_t* index, const char* str)
{
    return microstrain_strcat_n(buffer, buffer_size, index, str, strlen(str));
}

////////////////////////////////////////////////////////////////////////////////
///@brief Wrapper for std::snprintf with a better interface.
///
/// Example:
///@code{.cpp}
/// size_t format_dec_hex(char* buffer, size_t buffer_size, int value, bool dec, bool hex)
/// {
///   size_t index = 0;
///   bool ok = true;
///   if(dec) ok &= microstrain_strfmt(buffer, sizeof(buffer), &index, " dec=%d", value);
///   if(hex) ok &= microstrain_strfmt(buffer, sizeof(buffer), &index, " hex=%x", value);
///   return ok ? index : SIZE_MAX;
/// }
///@endcode
///
///@param buffer
///       Pointer to character buffer where string data will be stored.
///       If this is NULL, this function will only compute the required buffer
///       size (set buffer_size = 0 in this case).
///@param buffer_size
///       Size of the buffer. Up to buffer_size-1 chars will be written, plus
///       a NULL terminator. Must be 0 if buffer is NULL.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///
///@returns True if successful
///@returns False if an encoding error occurs (see snprintf). The index is
///         unchanged in this case.
///@returns False if insufficient space is available, unless buffer is NULL.
///
bool microstrain_strfmt(char* buffer, size_t buffer_size, size_t* index, const char* fmt, ...)
{
    assert(buffer != NULL || buffer_size == 0);
    assert(index != NULL);

    size_t fmt_size = buffer_size;

    // Allow a NULL buffer to be used to compute the size.
    if(buffer != NULL)
    {
        // If the buffer wasn't NULL but the size has already been exceeded,
        // pretend like it was NULL so that index can still be updated correctly.
        if(*index >= buffer_size)
        {
            buffer   = NULL;
            fmt_size = 0;
        }
        else
        {
            buffer   += *index;
            fmt_size -= *index;
        }
    }

    va_list args;
    va_start(args, fmt);
    int result = vsnprintf(buffer, fmt_size, fmt, args);
    va_end(args);

    // snprintf can fail with encoding errors, which result in a negative return code.
    if(result < 0)
        return false;

    // Update the index position (this may exceed the buffer size).
    *index += (unsigned int)result;

    // Return whether the buffer was large enough.
    return (buffer == NULL) || (*index < buffer_size);
}

static const char NIBBLE_HEX_TABLE[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};
static char nibble_to_hex_char(uint8_t value)
{
    return NIBBLE_HEX_TABLE[value & 0xF];
}

bool microstrain_strfmt_bytes(char* buffer, size_t buffer_size, size_t* index, const uint8_t* data, size_t data_size, unsigned int byte_grouping)
{
    assert(index != NULL);
    assert(buffer != NULL || buffer_size == 0);
    assert(data != NULL || data_size == 0);

    // Compute required buffer space.
    size_t num_chars = 2 * data_size;

    // Account for spaces between groups.
    // A perfectly divisible number of bytes will have one less
    // space because no leading/trailing spaces are added.
    if(byte_grouping > 0 && data_size > 0)
        num_chars += ((data_size-1) / byte_grouping);

    // Keep track of current position by pointer and update the index.
    char* ptr = &buffer[*index];
    *index += num_chars;

    // Don't write anything if the buffer is too small.
    if(*index >= buffer_size)
        return false;

    for(size_t i=0; i<data_size; i++)
    {
        if((i != 0) && (byte_grouping > 0) && (i % byte_grouping == 0))
            *(ptr++) = ' ';

        const uint8_t byte = data[i];
        const uint8_t upper_nibble = (byte & 0xF0) >> 4;
        const uint8_t lower_nibble = (byte & 0x0F) >> 0;
        *(ptr++) = nibble_to_hex_char(upper_nibble);
        *(ptr++) = nibble_to_hex_char(lower_nibble);
    }

    // Sanity check that num_chars math was correct.
    assert(ptr == &buffer[*index]);

    *ptr++ = '\0';

    return true;
}

