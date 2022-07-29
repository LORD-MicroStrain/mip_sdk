#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

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


enum mip_function_selector
{
    MIP_FUNCTION_WRITE = 0x01,
    MIP_FUNCTION_READ  = 0x02,
    MIP_FUNCTION_SAVE  = 0x03,
    MIP_FUNCTION_LOAD  = 0x04,
    MIP_FUNCTION_RESET = 0x05,
};
size_t insert_mip_function_selector(uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_function_selector self);
size_t extract_mip_function_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_function_selector* self);

struct mip_descriptor_rate
{
    uint8_t  descriptor;
    uint16_t decimation;
};
size_t insert_mip_descriptor_rate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_descriptor_rate* self);
size_t extract_mip_descriptor_rate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_descriptor_rate* self);


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

using MipFunctionSelector = C::mip_function_selector;
using MipDescriptorRate   = C::mip_descriptor_rate;

inline bool isDataDescriptorSet(uint8_t descriptorSet)     { return C::is_data_descriptor_set(descriptorSet); }
inline bool isCommandDescriptorSet(uint8_t descriptorSet)  { return C::is_cmd_descriptor_set(descriptorSet); }
inline bool isReservedDescriptorSet(uint8_t descriptorSet) { return C::is_reserved_descriptor_set(descriptorSet); }

inline bool isCommandDescriptor(uint8_t fieldDescriptor)  { return C::is_command_descriptor(fieldDescriptor); }
inline bool isReplyDescriptor(uint8_t fieldDescriptor)    { return C::is_reply_descriptor(fieldDescriptor); }
inline bool isResponseDescriptor(uint8_t fieldDescriptor) { return C::is_response_descriptor(fieldDescriptor); }
inline bool isReservedDescriptor(uint8_t fieldDescriptor) { return C::is_reserved_descriptor(fieldDescriptor); }

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
