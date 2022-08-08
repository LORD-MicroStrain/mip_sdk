#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

#include "../../utils/serialization.h"
#include "../../utils/enum_wrapper.hpp"

#include <tuple>
#include <type_traits>

namespace mscl {
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


////////////////////////////////////////////////////////////////////////////////
///@brief Convenience struct holding both descriptor set and field descriptor.
///
struct MipCompositeDescriptor
{
#ifdef __cplusplus
    bool operator==(const MipCompositeDescriptor& other) const { return other.descriptorSet == descriptorSet && other.fieldDescriptor == fieldDescriptor; }
    bool operator<(const MipCompositeDescriptor& other) const { return descriptorSet < other.descriptorSet || (!(descriptorSet > other.descriptorSet) && (fieldDescriptor < other.fieldDescriptor)); }
#endif // __cplusplus

    uint8_t descriptorSet;    ///< MIP descriptor set.
    uint8_t fieldDescriptor;  ///< MIP field descriptor.
};

#ifdef __cplusplus

} // extern "C"
} // namespace "C"


struct MipFunctionSelector : detail::EnumWrapper<C::mip_function_selector>
{
    static const C::mip_function_selector WRITE = C::MIP_FUNCTION_WRITE;
    static const C::mip_function_selector READ  = C::MIP_FUNCTION_READ;
    static const C::mip_function_selector SAVE  = C::MIP_FUNCTION_SAVE;
    static const C::mip_function_selector LOAD  = C::MIP_FUNCTION_LOAD;
    static const C::mip_function_selector RESET = C::MIP_FUNCTION_RESET;

    // size_t insert(struct mip_serializer* serializer) const { return C::insert_mip_function_selector(buffer, bufferSize, offset, *this); }
    // size_t extract(const struct mip_serializer* serializer) { return C::extract_mip_function_selector(buffer, bufferSize, offset, &_value); }
};

using MipDescriptorRate = C::mip_descriptor_rate;

inline bool isDataDescriptorSet   (uint8_t descriptorSet)  { return C::is_data_descriptor_set(descriptorSet); }
inline bool isCommandDescriptorSet(uint8_t descriptorSet)  { return C::is_cmd_descriptor_set(descriptorSet); }
inline bool isReservedDescriptorSet(uint8_t descriptorSet) { return C::is_reserved_descriptor_set(descriptorSet); }

inline bool isCommandDescriptor (uint8_t fieldDescriptor) { return C::is_command_descriptor(fieldDescriptor); }
inline bool isReplyDescriptor   (uint8_t fieldDescriptor) { return C::is_reply_descriptor(fieldDescriptor); }
inline bool isResponseDescriptor(uint8_t fieldDescriptor) { return C::is_response_descriptor(fieldDescriptor); }
inline bool isReservedDescriptor(uint8_t fieldDescriptor) { return C::is_reserved_descriptor(fieldDescriptor); }


inline void insert(MipSerializer& serializer, MipFunctionSelector self) { return C::insert_mip_function_selector(&serializer, self); }
inline void extract(MipSerializer& serializer, MipFunctionSelector& self) { return C::extract_mip_function_selector(&serializer, &self._value); }

inline void insert(MipSerializer& serializer, const MipDescriptorRate& self) { return C::insert_mip_descriptor_rate(&serializer, &self); }
inline void extract(MipSerializer& serializer, MipDescriptorRate& self) { return C::extract_mip_descriptor_rate(&serializer, &self); }

// ////////////////////////////////////////////////////////////////////////////////
// ///@brief Type traits struct for obtaining descriptors, etc. from field structs.
// ///
// /// This struct is specialized for each defined MIP field.
// ///
// template<class Field>
// struct MipFieldInfo
// {
//     static const uint8_t descriptorSet   = MIP_INVALID_DESCRIPTOR_SET;
//     static const uint8_t fieldDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
//
//     static_assert(!std::is_same<Field,Field>::value, "Missing specialization - did you forget to include the definition header?");
//
//     using Tuple = std::tuple<>;
//
//     static const bool responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;  // No response by default
//     using Response = void;
// };

} // namespace mscl

#endif // __cplusplus
