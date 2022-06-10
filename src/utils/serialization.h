#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "../types.h"

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@defgroup Serialization Functions for reading and writing to byte buffers.
///
///@{

void pack_bool(uint8_t* buffer, bool value);
void pack_char(uint8_t* buffer, char value);

void pack_u8 (uint8_t* buffer, uint8_t  value);
void pack_u16(uint8_t* buffer, uint16_t value);
void pack_u32(uint8_t* buffer, uint32_t value);
void pack_u64(uint8_t* buffer, uint64_t value);

void pack_s8 (uint8_t* buffer, int8_t   value);
void pack_s16(uint8_t* buffer, int16_t  value);
void pack_s32(uint8_t* buffer, int32_t  value);
void pack_s64(uint8_t* buffer, int64_t  value);

void pack_float (uint8_t* buffer, float  value);
void pack_double(uint8_t* buffer, double value);


////////////////////////////////////////////////////////////////////////////////
///@defgroup Append  Functions for serializing multiple values in a row.
///
/// append_xxx appends the specified type value to the given buffer.
///
///@param[in,out] buffer
///       A pointer to a pointer to the buffer where the value will be written.
///       This pointer is updated to point to the end of the value.
///@param[in,out] space
///       Available buffer space. Updated to reflect the remaining space after
///       the append operation is complete.
///@param[in] value
///       The value to be packed into the end of the buffer.
///
/// If insufficient space is available (i.e. space < sizeof(value)) then
/// the contents of the buffer are undefined.
///
///@{
void append_bool(uint8_t** buffer, RemainingCount* space, bool value);
void append_char(uint8_t** buffer, RemainingCount* space, char value);

void append_u8 (uint8_t** buffer, RemainingCount* space, uint8_t  value);
void append_u16(uint8_t** buffer, RemainingCount* space, uint16_t value);
void append_u32(uint8_t** buffer, RemainingCount* space, uint32_t value);
void append_u64(uint8_t** buffer, RemainingCount* space, uint64_t value);

void append_s8 (uint8_t** buffer, RemainingCount* space, int8_t   value);
void append_s16(uint8_t** buffer, RemainingCount* space, int16_t  value);
void append_s32(uint8_t** buffer, RemainingCount* space, int32_t  value);
void append_s64(uint8_t** buffer, RemainingCount* space, int64_t  value);

void append_float (uint8_t** buffer, RemainingCount* space, float  value);
void append_double(uint8_t** buffer, RemainingCount* space, double value);

///@}
////////////////////////////////////////////////////////////////////////////////


size_t insert_bool(uint8_t* buffer, size_t bufferSize, size_t offset, bool value);
size_t insert_char(uint8_t* buffer, size_t bufferSize, size_t offset, char value);

size_t insert_u8 (uint8_t* buffer, size_t bufferSize, size_t offset, uint8_t  value);
size_t insert_u16(uint8_t* buffer, size_t bufferSize, size_t offset, uint16_t value);
size_t insert_u32(uint8_t* buffer, size_t bufferSize, size_t offset, uint32_t value);
size_t insert_u64(uint8_t* buffer, size_t bufferSize, size_t offset, uint64_t value);

size_t insert_s8 (uint8_t* buffer, size_t bufferSize, size_t offset, int8_t  value);
size_t insert_s16(uint8_t* buffer, size_t bufferSize, size_t offset, int16_t value);
size_t insert_s32(uint8_t* buffer, size_t bufferSize, size_t offset, int32_t value);
size_t insert_s64(uint8_t* buffer, size_t bufferSize, size_t offset, int64_t value);

size_t insert_float (uint8_t* buffer, size_t bufferSize, size_t offset, float  value);
size_t insert_double(uint8_t* buffer, size_t bufferSize, size_t offset, double value);



bool unpack_bool(const uint8_t* buffer);
char unpack_char(const uint8_t* buffer);

uint8_t  unpack_u8 (const uint8_t* buffer);
uint16_t unpack_u16(const uint8_t* buffer);
uint32_t unpack_u32(const uint8_t* buffer);
uint64_t unpack_u64(const uint8_t* buffer);

int8_t   unpack_s8 (const uint8_t* buffer);
int16_t  unpack_s16(const uint8_t* buffer);
int32_t  unpack_s32(const uint8_t* buffer);
int64_t  unpack_s64(const uint8_t* buffer);

float  unpack_float (const uint8_t* buffer);
double unpack_double(const uint8_t* buffer);


size_t extract_bool(const uint8_t* buffer, size_t bufferSize, size_t offset, bool* value);
size_t extract_char(const uint8_t* buffer, size_t bufferSize, size_t offset, char* value);

size_t extract_u8 (const uint8_t* buffer, size_t bufferSize, size_t offset, uint8_t* value);
size_t extract_u16(const uint8_t* buffer, size_t bufferSize, size_t offset, uint16_t* value);
size_t extract_u32(const uint8_t* buffer, size_t bufferSize, size_t offset, uint32_t* value);
size_t extract_u64(const uint8_t* buffer, size_t bufferSize, size_t offset, uint64_t* value);

size_t extract_s8 (const uint8_t* buffer, size_t bufferSize, size_t offset, int8_t* value);
size_t extract_s16(const uint8_t* buffer, size_t bufferSize, size_t offset, int16_t* value);
size_t extract_s32(const uint8_t* buffer, size_t bufferSize, size_t offset, int32_t* value);
size_t extract_s64(const uint8_t* buffer, size_t bufferSize, size_t offset, int64_t* value);

size_t extract_float (const uint8_t* buffer, size_t bufferSize, size_t offset, float* value);
size_t extract_double(const uint8_t* buffer, size_t bufferSize, size_t offset, double* value);


///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mscl
} // extern "C"
#endif // __cplusplus
