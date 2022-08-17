
#include "serialization.h"

#include <string.h>

#ifdef __cplusplus
namespace mip {
#endif


void mip_serializer_init_insertion(struct mip_serializer* serializer, uint8_t* buffer, size_t buffer_size)
{
    serializer->buffer      = buffer;
    serializer->buffer_size = buffer_size;
    serializer->offset      = 0;
}
void mip_serializer_init_extraction(struct mip_serializer* serializer, const uint8_t* buffer, size_t buffer_size)
{
    serializer->buffer      = (uint8_t*)buffer;
    serializer->buffer_size = buffer_size;
    serializer->offset      = 0;
}

bool mip_serializer_ok(const struct mip_serializer* serializer)
{
    return serializer->offset <= serializer->buffer_size;
}

bool mip_serializer_finished(const struct mip_serializer* serializer, size_t expected_length)
{
    return serializer->offset == expected_length;
}


static void pack(uint8_t* buffer, const void* value, size_t size)
{
    for(size_t i=0; i<size; i++)
        buffer[ size-1 - i ] = ((uint8_t*)value)[i];
}

#define INSERT_MACRO(name, type) \
void insert_##name(struct mip_serializer* serializer, type value) \
{ \
    const size_t offset = serializer->offset + sizeof(type); \
    if( offset <= serializer->buffer_size ) \
        pack(&serializer->buffer[serializer->offset], &value, sizeof(type)); \
    serializer->offset = offset; \
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
void extract_##name(struct mip_serializer* serializer, type* value) \
{ \
    const size_t offset = serializer->offset + sizeof(type); \
    if( offset <= serializer->buffer_size ) \
        unpack(&serializer->buffer[serializer->offset], value, sizeof(type)); \
    serializer->offset = offset; \
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


void extract_count(struct mip_serializer* serializer, uint8_t* count_out, uint8_t max_count)
{
    *count_out = 0;  // Default to zero if extraction fails.
    extract_u8(serializer, count_out);
    if( *count_out > max_count )
    {
        // This is an error condition which can occur if the device sends
        // more array entries than the receiving structure expected.
        // This does not imply any sort of protocol violation, only that
        // the receiving array was not large enough.
        // Either way, deserialization cannot continue because the following
        // array extraction would leave some elements in the input buffer.
        *count_out = 0;
        serializer->offset = SIZE_MAX;
    }
}

#ifdef __cplusplus
} // namespace mip
#endif
