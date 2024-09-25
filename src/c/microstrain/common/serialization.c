
#include "serialization.h"


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a serialization struct for insertion into a buffer.
///
///@param serializer
///@param buffer
///       Buffer into which data will be written. Can be NULL if buffer_size==0.
///@param buffer_size
///       Size of the buffer. Data will not be written beyond this size.
///
void microstrain_serializer_init_insertion(microstrain_serializer* serializer, uint8_t* buffer, size_t buffer_size)
{
    serializer->_buffer      = buffer;
    serializer->_buffer_size = buffer_size;
    serializer->_offset      = 0;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a serialization struct for extraction from a buffer.
///
///@param serializer
///@param buffer
///       A pointer from which data will be read.
///@param buffer_size
///       Maximum number of bytes to be read from the buffer.
///
void microstrain_serializer_init_extraction(microstrain_serializer* serializer, const uint8_t* buffer, size_t buffer_size)
{
    serializer->_buffer      = (uint8_t*)buffer;
    serializer->_buffer_size = buffer_size;
    serializer->_offset      = 0;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines the total length the buffer.
///
///@param serializer
///
///@returns The buffer size.
///
size_t microstrain_serializer_capacity(const microstrain_serializer* serializer)
{
    return serializer->_buffer_size;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines the length of the data in the buffer.
///
///@param serializer
///
/// For insertion, returns how many bytes have been written.
/// For extraction, returns how many bytes have been read.
///
///@note This may exceed the buffer size. Check microstrain_serializer_is_ok() before using
///      the data.
///
size_t microstrain_serializer_length(const microstrain_serializer* serializer)
{
    return serializer->_offset;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines the difference between the length and buffer size.
///
///@param serializer
///
/// For insertion, returns how many unwritten bytes remain in the buffer.
/// For extraction, returns how many bytes have not been read.
///
///@note This can be a negative number if the application attempted to write
///      or read more data than contained in the buffer. This is not a bug and
///      it can be detected with the microstrain_serializer_is_ok() function.
///
int microstrain_serializer_remaining(const microstrain_serializer* serializer)
{
    return (int)(microstrain_serializer_capacity(serializer) - microstrain_serializer_length(serializer));
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the data read/written is less than the buffer size.
///
/// If the application attempts to read or write beyond the end of the buffer
/// (as defined by the buffer_size passed to the init function), the read or
/// write will be a no-op but the offset will still be advanced. This allows
/// the condition to be detected.
///
///@param serializer
///
///@returns true if microstrain_serializer_remaining() >= 0.
///
bool microstrain_serializer_is_ok(const microstrain_serializer* serializer)
{
    return microstrain_serializer_length(serializer) <= microstrain_serializer_capacity(serializer);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the number of remaining bytes is 0.
///
/// Use this to determine if the entire buffer has been extracted. It is not
/// particularly useful for insertion.
///
///@param serializer
///
///@returns true if microstrain_serializer_remaining() == 0.
///
bool microstrain_serializer_is_complete(const microstrain_serializer* serializer)
{
    return serializer->_offset == serializer->_buffer_size;
}


static void pack(uint8_t* buffer, const void* value, size_t size)
{
    for(size_t i=0; i<size; i++)
        buffer[ size-1 - i ] = ((uint8_t*)value)[i];
}

#define INSERT_MACRO(name, type) \
void microstrain_insert_##name(microstrain_serializer* serializer, type value) \
{ \
    const size_t offset = serializer->_offset + sizeof(type); \
    if( offset <= serializer->_buffer_size ) \
        pack(&serializer->_buffer[serializer->_offset], &value, sizeof(type)); \
    serializer->_offset = offset; \
}

INSERT_MACRO(bool,   bool    )
INSERT_MACRO(char,   char    )
INSERT_MACRO(u8,     uint8_t )
INSERT_MACRO(u16,    uint16_t)
INSERT_MACRO(u32,    uint32_t)
INSERT_MACRO(u64,    uint64_t)
INSERT_MACRO(s8,     int8_t  )
INSERT_MACRO(s16,    int16_t )
INSERT_MACRO(s32,    int32_t )
INSERT_MACRO(s64,    int64_t )
INSERT_MACRO(float,  float   )
INSERT_MACRO(double, double  )



static void unpack(const uint8_t* buffer, void* value, size_t size)
{
    for(size_t i=0; i<size; i++)
        ((uint8_t*)value)[i] = buffer[ size-1 - i ];
}


#define EXTRACT_MACRO(name, type) \
void microstrain_extract_##name(microstrain_serializer* serializer, type* value) \
{ \
    const size_t offset = serializer->_offset + sizeof(type); \
    if( offset <= serializer->_buffer_size ) \
        unpack(&serializer->_buffer[serializer->_offset], value, sizeof(type)); \
    serializer->_offset = offset; \
}

EXTRACT_MACRO(bool,   bool    )
EXTRACT_MACRO(char,   char    )
EXTRACT_MACRO(u8,     uint8_t )
EXTRACT_MACRO(u16,    uint16_t)
EXTRACT_MACRO(u32,    uint32_t)
EXTRACT_MACRO(u64,    uint64_t)
EXTRACT_MACRO(s8,     int8_t  )
EXTRACT_MACRO(s16,    int16_t )
EXTRACT_MACRO(s32,    int32_t )
EXTRACT_MACRO(s64,    int64_t )
EXTRACT_MACRO(float,  float   )
EXTRACT_MACRO(double, double  )


////////////////////////////////////////////////////////////////////////////////
///@brief Similar to microstrain_extract_u8 but allows a maximum value to be specified.
///
/// If the maximum count would be exceeded, an error is generated which causes
/// further extraction to fail.
///
///@param serializer
///@param count_out
///       The counter value read from the buffer.
///@param max_count
///       The maximum value of the counter. If the count exceeds this, it is
///       set to 0 and the serializer is insert into an error state.
///
void microstrain_extract_count(microstrain_serializer* serializer, uint8_t* count_out, uint8_t max_count)
{
    *count_out = 0;  // Default to zero if extraction fails.
    microstrain_extract_u8(serializer, count_out);
    if( *count_out > max_count )
    {
        // This is an error condition which can occur if the device sends
        // more array entries than the receiving structure expected.
        // This does not imply any sort of protocol violation, only that
        // the receiving array was not large enough.
        // Either way, deserialization cannot continue because the following
        // array extraction would leave some elements in the input buffer.
        *count_out = 0;
        serializer->_offset = SIZE_MAX;
    }
}
