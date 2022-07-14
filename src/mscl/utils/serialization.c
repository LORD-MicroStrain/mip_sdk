
#include "serialization.h"

#include <string.h>


#define SWAP(x,y) do { uint8_t tmp=(x); (x)=(y); (y)=tmp; } while(false)

static void pack(uint8_t* buffer, const void* value, size_t size)
{
    for(size_t i=0; i<size; i++)
        buffer[ size-1 - i ] = ((uint8_t*)value)[i];
}

#define INSERT_MACRO(name, type) \
size_t insert_##name(uint8_t* buffer, size_t space, size_t offset, type value) \
{ \
    const size_t postOffset = offset + sizeof(type); \
    if( space >= postOffset ) \
        pack(&buffer[offset], &value, sizeof(type)); \
    return postOffset; \
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
size_t extract_##name(const uint8_t* buffer, size_t bufferSize, size_t offset, type* value) \
{ \
    size_t postOffset = offset + sizeof(type); \
    if( postOffset <= bufferSize ) \
        unpack(&buffer[offset], value, sizeof(type)); \
    return postOffset; \
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
