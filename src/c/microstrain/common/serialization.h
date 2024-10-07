#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
///@defgroup microstrain_serialization  MicroStrain Serialization
///
///@brief Serialization functions for reading and writing to byte buffers.
///@{
///@defgroup microstrain_serialization_c   MicroStrain Serialization [C]
///@defgroup microstrain_serialization_cpp MicroStrain Serialization [CPP]
///@}

#ifdef __cplusplus
#include <type_traits>

namespace microstrain {
namespace C {
extern "C" {
#endif // __cplusplus



////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_serialization_c
///
///@brief (De)serialization in C.
///
///@{


////////////////////////////////////////////////////////////////////////////////
///@brief Structure used for serialization.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
///
typedef struct microstrain_serializer
{
    uint8_t* _buffer;        ///<@private Pointer to data for serialization.
    size_t   _buffer_size;   ///<@private Size of the buffer.
    size_t   _offset;        ///<@private Current offset into the buffer (can exceed buffer_size!).
} microstrain_serializer;

void microstrain_serializer_init_insertion(microstrain_serializer* serializer, uint8_t* buffer, size_t buffer_size);
void microstrain_serializer_init_extraction(microstrain_serializer* serializer, const uint8_t* buffer, size_t buffer_size);

size_t microstrain_serializer_capacity(const microstrain_serializer* serializer);
size_t microstrain_serializer_length(const microstrain_serializer* serializer);
int    microstrain_serializer_remaining(const microstrain_serializer* serializer);

bool microstrain_serializer_is_ok(const microstrain_serializer* serializer);
bool microstrain_serializer_is_complete(const microstrain_serializer* serializer);


void microstrain_insert_bool(microstrain_serializer* serializer, bool value);
void microstrain_insert_char(microstrain_serializer* serializer, char value);

void microstrain_insert_u8 (microstrain_serializer* serializer, uint8_t  value);
void microstrain_insert_u16(microstrain_serializer* serializer, uint16_t value);
void microstrain_insert_u32(microstrain_serializer* serializer, uint32_t value);
void microstrain_insert_u64(microstrain_serializer* serializer, uint64_t value);

void microstrain_insert_s8 (microstrain_serializer* serializer, int8_t  value);
void microstrain_insert_s16(microstrain_serializer* serializer, int16_t value);
void microstrain_insert_s32(microstrain_serializer* serializer, int32_t value);
void microstrain_insert_s64(microstrain_serializer* serializer, int64_t value);

void microstrain_insert_float(microstrain_serializer* serializer, float  value);
void microstrain_insert_double(microstrain_serializer* serializer, double value);


void microstrain_extract_bool(microstrain_serializer* serializer, bool* value);
void microstrain_extract_char(microstrain_serializer* serializer, char* value);

void microstrain_extract_u8 (microstrain_serializer* serializer, uint8_t* value);
void microstrain_extract_u16(microstrain_serializer* serializer, uint16_t* value);
void microstrain_extract_u32(microstrain_serializer* serializer, uint32_t* value);
void microstrain_extract_u64(microstrain_serializer* serializer, uint64_t* value);

void microstrain_extract_s8 (microstrain_serializer* serializer, int8_t* value);
void microstrain_extract_s16(microstrain_serializer* serializer, int16_t* value);
void microstrain_extract_s32(microstrain_serializer* serializer, int32_t* value);
void microstrain_extract_s64(microstrain_serializer* serializer, int64_t* value);

void microstrain_extract_float (microstrain_serializer* serializer, float* value);
void microstrain_extract_double(microstrain_serializer* serializer, double* value);

void microstrain_extract_count(microstrain_serializer* serializer, uint8_t* count_out, uint8_t max_count);

///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace microstrain
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
