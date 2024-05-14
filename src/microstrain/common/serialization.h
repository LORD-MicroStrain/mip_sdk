#pragma once

#include "mip/mip_field.h"
#include "mip/mip_packet.h"
#include "mip/mip_types.h"

#include <stdint.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_serialization  MIP Serialization
///
///@brief Serialization Functions for reading and writing to byte buffers.
///@{
///@defgroup mip_serialization_c   MIP Serialization [C]
///@defgroup mip_serialization_cpp MIP Serialization [CPP]
///@}

#ifdef __cplusplus
#include <type_traits>

namespace microstrain {
namespace C {
extern "C" {
#endif // __cplusplus



////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_serialization_c
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

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_serialization_cpp
///
///@brief (De)serialization in C++.
///
/// There are two overloaded functions defined in the mip namespace, insert and
/// extract. The signature for each is as follows:
///@code{.cpp}
/// void mip::insert(Serializer& serializer, Type value);
/// voie mip::extract(Serializer& serializer, Type value);
///@endcode
/// Where `Type` is a struct or numeric type.
///
/// There are overloads for all of the MIP definition types:
///@li Command, response, and data fields
///@li Enums, bitfields, and nested structs for commands
///
/// Additionally, there are overloads with a different signature which allow
/// one to avoid creating a Serializer object every time. These overloads
/// create a serializer internally and pass it on to the regular version.
///@code{.cpp}
/// template<typename T> bool mip::insert(const T& value, uint8_t* buffer, size_t bufferSize);
/// template<typename T> bool mip::Field::extract(T& value);
///@endcode
/// This makes it easy to extract data from a field:
///@code{.cpp}
/// data_sensor::ScaledAccel accel;
/// MipField field(...);
/// if( field.extract(accel) )
/// {
///   // Do something with accel data
///   printf("Accel X=%f\n", accel.scaled_accel[0]);
/// }
///@endcode
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Serialization class.
///
class Serializer : public C::microstrain_serializer
{
public:
    // Serializer(C::mip_packet& packet, uint8_t newFieldDescriptor) { C::microstrain_serializer_init_new_field(this, &packet, newFieldDescriptor); }
    Serializer(uint8_t* buffer, size_t size, size_t offset=0) { C::microstrain_serializer_init_insertion(this, buffer, size); this->_offset = offset; }
    Serializer(const uint8_t* buffer, size_t size, size_t offset=0) { C::microstrain_serializer_init_extraction(this, const_cast<uint8_t*>(buffer), size); this->_offset = offset; }

    size_t capacity() const { return C::microstrain_serializer_capacity(this); }
    size_t length() const { return C::microstrain_serializer_length(this); }
    int remaining() const { return C::microstrain_serializer_remaining(this); }

    bool isOk() const { return C::microstrain_serializer_is_ok(this); }
    bool isComplete() const { return C::microstrain_serializer_is_complete(this); }

    operator const void*() const { return isOk() ? this : nullptr; }
    bool operator!() const { return !isOk(); }
};



inline void insert(Serializer& serializer, bool value)     { return C::microstrain_insert_bool(&serializer, value); }
inline void insert(Serializer& serializer, char value)     { return C::microstrain_insert_char(&serializer, value); }
inline void insert(Serializer& serializer, uint8_t  value) { return C::microstrain_insert_u8(&serializer, value); }
inline void insert(Serializer& serializer, uint16_t value) { return C::microstrain_insert_u16(&serializer, value); }
inline void insert(Serializer& serializer, uint32_t value) { return C::microstrain_insert_u32(&serializer, value); }
inline void insert(Serializer& serializer, uint64_t value) { return C::microstrain_insert_u64(&serializer, value); }
inline void insert(Serializer& serializer, int8_t  value)  { return C::microstrain_insert_s8(&serializer, value); }
inline void insert(Serializer& serializer, int16_t value)  { return C::microstrain_insert_s16(&serializer, value); }
inline void insert(Serializer& serializer, int32_t value)  { return C::microstrain_insert_s32(&serializer, value); }
inline void insert(Serializer& serializer, int64_t value)  { return C::microstrain_insert_s64(&serializer, value); }
inline void insert(Serializer& serializer, float  value)   { return C::microstrain_insert_float(&serializer, value); }
inline void insert(Serializer& serializer, double value)   { return C::microstrain_insert_double(&serializer, value); }

////////////////////////////////////////////////////////////////////////////////
///@brief Inserts an enum into the buffer.
///
///@tparam Enum The type of the enum to serialize. Must be a c++ typed enum.
///
///@param serializer The serialization instance.
///@param value      The enum to insert.
///
template<typename Enum>
typename std::enable_if< std::is_enum<Enum>::value, void>::type
/*void*/ insert(Serializer& serializer, Enum value) { return insert(serializer, static_cast< typename std::underlying_type<Enum>::type >(value) ); }

////////////////////////////////////////////////////////////////////////////////
///@brief Insert the given value into the buffer.
///
/// If the buffer has insufficient space, this function returns false and the
/// contents of buffer may be partially modified.
///
///@param value      Value to insert.
///@param buffer     Buffer to udpate with the value.
///@param bufferSize Size of the buffer.
///@param offset     Starting offset into the buffer.
///
///@returns true if sufficient space was available, false otherwise.
///
template<typename T>
bool insert(const T& value, uint8_t* buffer, size_t bufferSize, size_t offset=0)
{
    Serializer serializer(buffer, bufferSize, offset);
    insert(serializer, value);
    return !!serializer;
}

inline void extract(Serializer& serializer, bool& value)     { return C::microstrain_extract_bool(&serializer, &value); }
inline void extract(Serializer& serializer, char& value)     { return C::microstrain_extract_char(&serializer, &value); }
inline void extract(Serializer& serializer, uint8_t&  value) { return C::microstrain_extract_u8(&serializer, &value); }
inline void extract(Serializer& serializer, uint16_t& value) { return C::microstrain_extract_u16(&serializer, &value); }
inline void extract(Serializer& serializer, uint32_t& value) { return C::microstrain_extract_u32(&serializer, &value); }
inline void extract(Serializer& serializer, uint64_t& value) { return C::microstrain_extract_u64(&serializer, &value); }
inline void extract(Serializer& serializer, int8_t&  value)  { return C::microstrain_extract_s8(&serializer, &value); }
inline void extract(Serializer& serializer, int16_t& value)  { return C::microstrain_extract_s16(&serializer, &value); }
inline void extract(Serializer& serializer, int32_t& value)  { return C::microstrain_extract_s32(&serializer, &value); }
inline void extract(Serializer& serializer, int64_t& value)  { return C::microstrain_extract_s64(&serializer, &value); }
inline void extract(Serializer& serializer, float&  value)   { return C::microstrain_extract_float(&serializer, &value); }
inline void extract(Serializer& serializer, double& value)   { return C::microstrain_extract_double(&serializer, &value); }

////////////////////////////////////////////////////////////////////////////////
///@brief Extract an enum from the buffer.
///
///@tparam Enum The type of the enum to deserialize. Must be a c++ typed enum.
///
///@param serializer The serialization instance.
///@param[out] value The enum to populate.
///
template<typename Enum>
typename std::enable_if< std::is_enum<Enum>::value, void>::type
/*void*/ extract(Serializer& serializer, Enum& value) {
    typename std::underlying_type<Enum>::type tmp;
    extract(serializer, tmp);
    value = static_cast<Enum>(tmp);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Extract the value given a buffer, size, and starting offset.
///
///@param[out] value_out
///       This parameter will be filled with the extracted value.
///@param buffer
///       A pointer to the raw data.
///@param bufferSize
///       Length of the buffer, or the relevant data within the buffer.
///@param offset
///       Start reading from this offset in the buffer. Default 0.
///@param exact_size
///       If true, exactly bufferSize bytes must be used in order for the return
///       value to be true.
///
///@returns True if the extraction was successful, false otherwise. "Success"
///         means the supplied data was sufficient. If exact_size is true, then
///         this function only returns true if exactly bufferSize bytes were
///         consumed.
///
template<typename T>
bool extract(T& value_out, const uint8_t* buffer, size_t bufferSize, size_t offset=0, bool exact_size=false)
{
    Serializer serializer(buffer, bufferSize, offset);
    extract(serializer, value_out);
    return exact_size ? serializer.isComplete() : serializer.isOk();
}

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace microstrain
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////