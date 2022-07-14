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
