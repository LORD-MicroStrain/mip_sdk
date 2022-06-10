
#include "serialization.h"

#include <string.h>


#define SWAP(x,y) do { uint8_t tmp=(x); (x)=(y); (y)=tmp; } while(false)

static void pack_void(uint8_t* buffer, const uint8_t* value, size_t size)
{
    for(size_t i=0; i<size; i++)
        buffer[ size-1 - i ] = value[i];
}


void pack_u8(uint8_t* buffer, uint8_t value)
{
    *buffer = value;
}

void pack_u16(uint8_t* buffer, uint16_t value)
{
    buffer[1] = value >> 8;
    buffer[0] = value >> 0;
}

void pack_u32(uint8_t* buffer, uint32_t value)
{
    buffer[0] = value >> 24;
    buffer[1] = value >> 16;
    buffer[2] = value >>  8;
    buffer[3] = value >>  0;
}

void pack_u64(uint8_t* buffer, uint64_t value)
{
    buffer[0] = value >> 56;
    buffer[1] = value >> 48;
    buffer[2] = value >> 40;
    buffer[3] = value >> 32;
    buffer[4] = value >> 24;
    buffer[5] = value >> 16;
    buffer[6] = value >>  8;
    buffer[7] = value >>  0;
}

void pack_float(uint8_t* buffer, float value)
{
    pack_void(buffer, (const uint8_t*)&value, sizeof(value));
}

void pack_double(uint8_t* buffer, double value)
{
    pack_void(buffer, (const uint8_t*)&value, sizeof(value));
}

void pack_bool(uint8_t* buffer, bool value) { return pack_u8(buffer, value); }
void pack_char(uint8_t* buffer, char value) { return pack_u8(buffer, value); }

void pack_s8 (uint8_t* buffer, int8_t  value) { return pack_u8 (buffer, value); }
void pack_s16(uint8_t* buffer, int16_t value) { return pack_u16(buffer, value); }
void pack_s32(uint8_t* buffer, int32_t value) { return pack_u32(buffer, value); }
void pack_s64(uint8_t* buffer, int64_t value) { return pack_u64(buffer, value); }


#define APPEND_MACRO(name, type) \
void append_##name(uint8_t** buffer, RemainingCount* space, type value) { \
    *space -= sizeof(value); \
    if( *space >= 0 ) \
    { \
        pack_##name(*buffer, value); \
        *buffer += sizeof(value); \
    } \
}

#define INSERT_MACRO(name, type) \
size_t insert_##name(uint8_t* buffer, size_t space, size_t offset, type value) \
{ \
    const size_t postOffset = offset + sizeof(type); \
    if( space >= postOffset ) \
        pack_##name(&buffer[offset], value); \
    return postOffset; \
}

APPEND_MACRO(bool,   bool    )
APPEND_MACRO(char,   char    )
APPEND_MACRO(u8,     uint8_t )
APPEND_MACRO(u16,    uint16_t)
APPEND_MACRO(u32,    uint32_t)
APPEND_MACRO(u64,    uint64_t)
APPEND_MACRO(s8,     int8_t  )
APPEND_MACRO(s16,    int16_t )
APPEND_MACRO(s32,    int32_t )
APPEND_MACRO(s64,    int64_t )
APPEND_MACRO(float,  float   )
APPEND_MACRO(double, double  )

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



bool unpack_bool(const uint8_t* buffer) { return unpack_u8(buffer) > 0; }
char unpack_char(const uint8_t* buffer) { return unpack_u8(buffer); }

uint8_t unpack_u8(const uint8_t* buffer)
{
    return *buffer;
}

uint16_t unpack_u16(const uint8_t* buffer)
{
    uint16_t tmp = 0;
    tmp = (tmp << 8) | buffer[0];
    tmp = (tmp << 8) | buffer[1];
    return tmp;
}

uint32_t unpack_u32(const uint8_t* buffer)
{
    uint32_t tmp = 0;
    tmp = (tmp << 8) | buffer[0];
    tmp = (tmp << 8) | buffer[1];
    tmp = (tmp << 8) | buffer[2];
    tmp = (tmp << 8) | buffer[3];
    return tmp;
}

uint64_t unpack_u64(const uint8_t* buffer)
{
    uint64_t tmp = 0;
    tmp = (tmp << 8) | buffer[0];
    tmp = (tmp << 8) | buffer[1];
    tmp = (tmp << 8) | buffer[2];
    tmp = (tmp << 8) | buffer[3];
    tmp = (tmp << 8) | buffer[4];
    tmp = (tmp << 8) | buffer[5];
    tmp = (tmp << 8) | buffer[6];
    tmp = (tmp << 8) | buffer[7];
    return tmp;
}

float unpack_float(const uint8_t* buffer)
{
    float value;

    for(size_t i=0; i<sizeof(value); i++)
        ((uint8_t*)&value)[i] = buffer[ sizeof(value)-1 - i ];

    return value;
}

double unpack_double(const uint8_t* buffer)
{
    double value;

    for(size_t i=0; i<sizeof(value); i++)
        ((uint8_t*)&value)[i] = buffer[ sizeof(value)-1 - i ];

    return value;
}

int8_t  unpack_s8 (const uint8_t* buffer) { return unpack_u8 (buffer); }
int16_t unpack_s16(const uint8_t* buffer) { return unpack_u16(buffer); }
int32_t unpack_s32(const uint8_t* buffer) { return unpack_u32(buffer); }
int64_t unpack_s64(const uint8_t* buffer) { return unpack_u64(buffer); }


#define EXTRACT_MACRO(name, type) \
size_t extract_##name(const uint8_t* buffer, size_t bufferSize, size_t offset, type* value) \
{ \
    size_t postOffset = offset + sizeof(type); \
    if( postOffset <= bufferSize ) \
        *value = unpack_##name(buffer); \
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
