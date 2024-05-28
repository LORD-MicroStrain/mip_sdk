#pragma once

#include "mip_descriptors.h"

#include "mip_result.hpp"

#include "microstrain/common/buffer.hpp"

#include <tuple>
#include <type_traits>
#include <functional>


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Convenience struct holding both descriptor set and field descriptor.
///
struct CompositeDescriptor
{
    uint8_t descriptorSet;    ///< MIP descriptor set.
    uint8_t fieldDescriptor;  ///< MIP field descriptor.

    constexpr CompositeDescriptor(uint8_t descSet, uint8_t fieldDesc) : descriptorSet(descSet), fieldDescriptor(fieldDesc) {}
    constexpr CompositeDescriptor(uint16_t combo) : descriptorSet(combo >> 8), fieldDescriptor(combo & 0xFF) {}

    CompositeDescriptor& operator=(uint16_t combo) { return *this = CompositeDescriptor(combo); }

    constexpr uint16_t as_u16() const { return (uint16_t(descriptorSet) << 8) | fieldDescriptor; }

//    operator uint16_t() const { return as_u16(); }

    constexpr bool operator==(const CompositeDescriptor& other) const { return other.descriptorSet == descriptorSet && other.fieldDescriptor == fieldDescriptor; }
    constexpr bool operator<(const CompositeDescriptor& other) const { return as_u16() < other.as_u16(); }

};


///@brief A dummy struct which is used to mark bitfield objects.
///
template<typename DerivedT> struct Bitfield {};

template<class Derived> void insert (::microstrain::Buffer& serializer, const Bitfield<Derived>& bitfield) { insert(serializer, static_cast<const Derived&>(bitfield).value); }
template<class Derived> void extract(::microstrain::Buffer& serializer, Bitfield<Derived>& bitfield) { extract(serializer, static_cast<Derived&>(bitfield).value); }


enum class FunctionSelector : uint8_t
{
    WRITE = C::MIP_FUNCTION_WRITE,
    READ  = C::MIP_FUNCTION_READ,
    SAVE  = C::MIP_FUNCTION_SAVE,
    LOAD  = C::MIP_FUNCTION_LOAD,
    RESET = C::MIP_FUNCTION_RESET,
};

static constexpr uint8_t INVALID_FIELD_DESCRIPTOR = C::MIP_INVALID_FIELD_DESCRIPTOR;
static constexpr uint8_t INVALID_DESCRIPTOR_SET   = C::MIP_INVALID_DESCRIPTOR_SET;

inline bool isValidDescriptorSet   (uint8_t descriptorSet) { return C::mip_is_valid_descriptor_set(descriptorSet); }
inline bool isDataDescriptorSet    (uint8_t descriptorSet) { return C::mip_is_data_descriptor_set(descriptorSet); }
inline bool isCommandDescriptorSet (uint8_t descriptorSet) { return C::mip_is_cmd_descriptor_set(descriptorSet); }
inline bool isReservedDescriptorSet(uint8_t descriptorSet) { return C::mip_is_reserved_descriptor_set(descriptorSet); }
inline bool isGnssDataDescriptorSet(uint8_t descriptorSet) { return C::mip_is_gnss_data_descriptor_set(descriptorSet); }

inline bool isValidFieldDescriptor   (uint8_t fieldDescriptor)   { return C::mip_is_valid_field_descriptor(fieldDescriptor); }
inline bool isCommandFieldDescriptor (uint8_t fieldDescriptor)   { return C::mip_is_cmd_field_descriptor(fieldDescriptor); }
inline bool isReplyFieldDescriptor   (uint8_t fieldDescriptor)   { return C::mip_is_reply_field_descriptor(fieldDescriptor); }
inline bool isResponseFieldDescriptor(uint8_t fieldDescriptor)   { return C::mip_is_response_field_descriptor(fieldDescriptor); }
inline bool isReservedFieldDescriptor(uint8_t fieldDescriptor)   { return C::mip_is_reserved_cmd_field_descriptor(fieldDescriptor); }
inline bool isSharedDataFieldDescriptor(uint8_t fieldDescriptor) { return C::mip_is_shared_data_field_descriptor(fieldDescriptor); }


////////////////////////////////////////////////////////////////////////////////
///@brief A CmdResult that knows the corresponding command type.
///
///@tparam MipCmd Type of the command struct.
///
template<class MipCmd>
struct TypedResult : public CmdResult
{
    using Cmd = MipCmd;

    // Same constructor as CmdResult.
    using CmdResult::CmdResult;

    ///@brief The command descriptor.
    ///
    static constexpr CompositeDescriptor DESCRIPTOR = MipCmd::DESCRIPTOR;

    ///@brief Returns the composite descriptor of the command.
    ///
    constexpr CompositeDescriptor descriptor() const { return DESCRIPTOR; }
};

///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip

