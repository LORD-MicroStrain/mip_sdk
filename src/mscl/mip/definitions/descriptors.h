#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

#include <tuple>
#include <type_traits>

namespace mscl {

namespace C { struct MipInterfaceState; }
using C::MipInterfaceState;

extern "C" {

#endif // __cplusplus

enum {
    MIP_INVALID_DESCRIPTOR_SET   = 0x00,
    MIP_INVALID_FIELD_DESCRIPTOR = 0x00,
};

bool isDataDescriptorSet(uint8_t descriptorSet);

enum MipFunctionSelector
{
    MIP_FUNCTION_WRITE = 0x01,
    MIP_FUNCTION_READ  = 0x02,
    MIP_FUNCTION_SAVE  = 0x03,
    MIP_FUNCTION_LOAD  = 0x04,
    MIP_FUNCTION_RESET = 0x05,
};
size_t insert_MipFunctionSelector(uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFunctionSelector self);
size_t extract_MipFunctionSelector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFunctionSelector* self);

struct MipDescriptorRate
{
    uint8_t  descriptor;
    uint16_t decimation;
};
size_t insert_MipDescriptorRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipDescriptorRate* self);
size_t extract_MipDescriptorRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipDescriptorRate* self);


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

////////////////////////////////////////////////////////////////////////////////
///@brief Type traits struct for obtaining descriptors, etc. from field structs.
///
/// This struct is specialized for each defined MIP field.
///
template<class Field>
struct MipFieldInfo
{
    static const uint8_t descriptorSet   = MIP_INVALID_DESCRIPTOR_SET;
    static const uint8_t fieldDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;

    static_assert(!std::is_same<Field,Field>::value, "Missing specialization - did you forget to include the definition header?");

    using Tuple = std::tuple<>;

    static const bool responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;  // No response by default
    using Response = void;
};

} // namespace mscl

#endif // __cplusplus