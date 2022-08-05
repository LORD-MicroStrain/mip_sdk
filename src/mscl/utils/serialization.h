#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "../types.h"

#ifdef __cplusplus
#include <type_traits>

namespace mscl {
namespace C {
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
} // extern "C"
} // namespace C

size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, bool value) { return C::insert_bool(buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, char value) { return C::insert_char(buffer, bufferSize, offset, value); }

size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, uint8_t  value) { return C::insert_u8 (buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, uint16_t value) { return C::insert_u16(buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, uint32_t value) { return C::insert_u32(buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, uint64_t value) { return C::insert_u64(buffer, bufferSize, offset, value); }

size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, int8_t  value) { return C::insert_s8 (buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, int16_t value) { return C::insert_s16(buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, int32_t value) { return C::insert_s32(buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, int64_t value) { return C::insert_s64(buffer, bufferSize, offset, value); }

size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, float  value) { return C::insert_float (buffer, bufferSize, offset, value); }
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, double value) { return C::insert_double(buffer, bufferSize, offset, value); }

template<typename Enum>
size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, std::enable_if< std::is_enum<Enum>::value, Enum>::type value) { return insert(buffer, bufferSize, offset, static_cast< std::underlying_type<Enum>::type >(value) ); }


size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, bool& value) { return C::extract_bool(buffer, bufferSize, offset, &value); }
size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, char& value) { return C::extract_char(buffer, bufferSize, offset, &value); }

size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, uint8_t&  value) { return C::extract_u8 (buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, uint16_t& value) { return C::extract_u16(buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, uint32_t& value) { return C::extract_u32(buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, uint64_t& value) { return C::extract_u64(buffer, bufferSize, offset, &value); }

size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, int8_t&  value) { return C::extract_s8 (buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, int16_t& value) { return C::extract_s16(buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, int32_t& value) { return C::extract_s32(buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, int64_t& value) { return C::extract_s64(buffer, bufferSize, offset, &value); }

size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, float&  value) { return C::extract_float (buffer, bufferSize, offset, &value); }
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, double& value) { return C::extract_double(buffer, bufferSize, offset, &value); }

template<typename Enum>
size_t extract(uint8_t* buffer, size_t bufferSize, size_t offset, std::enable_if< std::is_enum<Enum>::value, Enum>::type value) { return insert(buffer, bufferSize, offset, static_cast< std::underlying_type<Enum>::type >(value) ); }

} // namespace mscl
#endif // __cplusplus
