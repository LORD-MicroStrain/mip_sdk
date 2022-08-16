#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

#include "../utils/serialization.h"
#include "../utils/enum_wrapper.hpp"

#include <tuple>
#include <type_traits>

namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus

enum {
    MIP_INVALID_DESCRIPTOR_SET        = 0x00,
    MIP_DATA_DESCRIPTOR_SET_START     = 0x80,
    MIP_RESERVED_DESCRIPTOR_SET_START = 0xF0,

    MIP_INVALID_FIELD_DESCRIPTOR  = 0x00,
    MIP_REPLY_DESCRIPTOR          = 0xF1,
    MIP_RESERVED_DESCRIPTOR_START = 0xF0,
    MIP_RESPONSE_DESCRIPTOR_START = 0x80,
};

bool is_valid_descriptor_set(uint8_t descriptor_set);
bool is_data_descriptor_set(uint8_t descriptor_set);
bool is_cmd_descriptor_set(uint8_t descriptor_set);
bool is_reserved_descriptor_set(uint8_t descriptor_set);

bool is_valid_descriptor(uint8_t field_descriptor);
bool is_command_descriptor(uint8_t field_descriptor);
bool is_reply_descriptor(uint8_t field_descriptor);
bool is_response_descriptor(uint8_t field_descriptor);
bool is_reserved_descriptor(uint8_t field_descriptor);


struct mip_serializer;

enum mip_function_selector
{
    MIP_FUNCTION_WRITE = 0x01,
    MIP_FUNCTION_READ  = 0x02,
    MIP_FUNCTION_SAVE  = 0x03,
    MIP_FUNCTION_LOAD  = 0x04,
    MIP_FUNCTION_RESET = 0x05,
};
void insert_mip_function_selector(struct mip_serializer* serializer, enum mip_function_selector self);
void extract_mip_function_selector(struct mip_serializer* serializer, enum mip_function_selector* self);

struct mip_descriptor_rate
{
    uint8_t  descriptor;
    uint16_t decimation;
};
void insert_mip_descriptor_rate(struct mip_serializer* serializer, const struct mip_descriptor_rate* self);
void extract_mip_descriptor_rate(struct mip_serializer* serializer, struct mip_descriptor_rate* self);

#ifdef __cplusplus

} // extern "C"
} // namespace "C"


////////////////////////////////////////////////////////////////////////////////
///@brief Convenience struct holding both descriptor set and field descriptor.
///
struct CompositeDescriptor
{
    uint8_t descriptorSet;    ///< MIP descriptor set.
    uint8_t fieldDescriptor;  ///< MIP field descriptor.

    CompositeDescriptor(uint8_t descSet, uint8_t fieldDesc) : descriptorSet(descSet), fieldDescriptor(fieldDesc) {}
    CompositeDescriptor(uint16_t combo) : descriptorSet(combo >> 8), fieldDescriptor(combo & 0xFF) {}

    CompositeDescriptor& operator=(uint16_t combo) { return *this = CompositeDescriptor(combo); }

    uint16_t as_u16() const { return (uint16_t(descriptorSet) << 8) | fieldDescriptor; }

//    operator uint16_t() const { return as_u16(); }

    bool operator==(const CompositeDescriptor& other) const { return other.descriptorSet == descriptorSet && other.fieldDescriptor == fieldDescriptor; }
    bool operator<(const CompositeDescriptor& other) const { return descriptorSet < other.descriptorSet || (!(descriptorSet > other.descriptorSet) && (fieldDescriptor < other.fieldDescriptor)); }

};


///@brief A dummy struct which is used to mark bitfield objects.
///
template<typename DerivedT> struct Bitfield {};

template<class Derived> void insert (Serializer& serializer, Bitfield<Derived> bitfield) { insert(serializer, static_cast<const Derived&>(bitfield).value); }
template<class Derived> void extract(Serializer& serializer, Bitfield<Derived>& bitfield) { insert(serializer, static_cast<Derived&>(bitfield).value); }


enum class FunctionSelector : uint8_t
{
    WRITE = C::MIP_FUNCTION_WRITE,
    READ  = C::MIP_FUNCTION_READ,
    SAVE  = C::MIP_FUNCTION_SAVE,
    LOAD  = C::MIP_FUNCTION_LOAD,
    RESET = C::MIP_FUNCTION_RESET,
};

using DescriptorRate = C::mip_descriptor_rate;

inline bool isDataDescriptorSet   (uint8_t descriptorSet)  { return C::is_data_descriptor_set(descriptorSet); }
inline bool isCommandDescriptorSet(uint8_t descriptorSet)  { return C::is_cmd_descriptor_set(descriptorSet); }
inline bool isReservedDescriptorSet(uint8_t descriptorSet) { return C::is_reserved_descriptor_set(descriptorSet); }

inline bool isCommandDescriptor (uint8_t fieldDescriptor) { return C::is_command_descriptor(fieldDescriptor); }
inline bool isReplyDescriptor   (uint8_t fieldDescriptor) { return C::is_reply_descriptor(fieldDescriptor); }
inline bool isResponseDescriptor(uint8_t fieldDescriptor) { return C::is_response_descriptor(fieldDescriptor); }
inline bool isReservedDescriptor(uint8_t fieldDescriptor) { return C::is_reserved_descriptor(fieldDescriptor); }


inline void insert(Serializer& serializer, const DescriptorRate& self) { return C::insert_mip_descriptor_rate(&serializer, &self); }
inline void extract(Serializer& serializer, DescriptorRate& self) { return C::extract_mip_descriptor_rate(&serializer, &self); }

} // namespace mip

#endif // __cplusplus
