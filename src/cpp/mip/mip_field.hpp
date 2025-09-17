#pragma once

#include "mip_descriptors.hpp"
#include "mip_serialization.hpp"

#include <microstrain/array_view.hpp>

#include <mip/mip_field.h>
#include <mip/mip_offsets.h>

#include <cstring>


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a MIP field.
///
/// This is a thin wrapper around the C mip_field struct.
///
class FieldView : public C::mip_field_view
{
public:
    static constexpr size_t MAX_PAYLOAD_LENGTH = C::MIP_FIELD_PAYLOAD_LENGTH_MAX;

    /// Construct an empty MIP field.
    FieldView() { C::mip_field_view::_payload=nullptr; C::mip_field_view::_payload_length=0; C::mip_field_view::_field_descriptor=0x00; C::mip_field_view::_descriptor_set=0x00; C::mip_field_view::_remaining_length=0; }
    ///@copydoc mip_field_init()
    FieldView(uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length) { C::mip_field_init(this, descriptor_set, field_descriptor, payload, payload_length); }
    ///@copydoc mip_field_from_header_ptr()
    FieldView(const uint8_t* header, uint8_t total_length, uint8_t descriptor_set) { *this = C::mip_field_from_header_ptr(header, total_length, descriptor_set); }
    /// Creates a %Field class from the mip_field C struct.
    FieldView(const C::mip_field_view& other) { std::memcpy(static_cast<C::mip_field_view*>(this), &other, sizeof(C::mip_field_view)); }

    /// Construct from descriptor and payload bytes
    FieldView(CompositeDescriptor descriptor, microstrain::ConstU8ArrayView payload) : FieldView(descriptor.descriptorSet, descriptor.fieldDescriptor, payload.data(), uint8_t(payload.size())) {}

    //
    // C function wrappers
    //

    ///@copydoc mip::C::mip_field_descriptor_set
    uint8_t descriptorSet() const { return C::mip_field_descriptor_set(this); }
    ///@copydoc mip::C::mip_field_field_descriptor
    uint8_t fieldDescriptor() const { return C::mip_field_field_descriptor(this); }
    ///@brief Returns the descriptor set and field descriptor.
    CompositeDescriptor descriptor() const { return {descriptorSet(), fieldDescriptor()}; }
    ///@copydoc mip::C::mip_field_payload_length
    uint8_t payloadLength() const { return C::mip_field_payload_length(this); }
    ///@copydoc mip::C::mip_field_total_length
    uint8_t totalLength() const { return C::mip_field_total_length(this); }
    ///@copydoc mip::C::mip_field_payload
    const uint8_t* payloadPointer() const { return C::mip_field_payload(this); }

    ///@brief Index the payload at the given location.
    ///@param index Byte index into payload. 0 <= index < payloadLength().
    ///@returns payload byte
    uint8_t payload(size_t index) const { assert(index < payloadLength()); return payloadPointer()[index]; }

    uint8_t operator[](size_t index) const { return payload(index); }

    ///@brief Get a const view of the payload data.
    microstrain::ConstU8ArrayView payload() const { return {payloadPointer(), payloadLength()}; }

    ///@brief Gets a view of the entire field, including the header.
    ///
    ///@warning FieldView contains a descriptor and payload pointer. It does not
    ///         guarantee the descriptors are also stored in the same buffer as
    ///         the payload.
    ///         This function operates by using a negative index from the
    ///         payload pointer, so it is up to the user to ensure such an index
    ///         is safe. Generally, if the field was obtained from an existing
    ///         packet then the header bytes will be available. E.g. the field
    ///         was obtained via `PacketView::firstField()`,
    ///         `for(FieldView field : packet)`, etc. then the whole packet,
    ///         including all fields and their headers, should exist in the
    ///         buffer. Standalone FieldViews created from a descriptor and
    ///         payload array may not have header bytes and this function isn't
    ///         safe in that case.
    ///
    microstrain::ConstU8ArrayView data() const { return {payloadPointer()-C::MIP_HEADER_LENGTH, size_t(payloadLength()+C::MIP_HEADER_LENGTH)}; }

    ///@copydoc mip::C::mip_field_is_valid
    bool isValid() const { return C::mip_field_is_valid(this); }

    ///@copybrief mip::C::mip_field_next_after
    FieldView nextAfter() const { return C::mip_field_next_after(this); }
    ///@copybrief mip::C::mip_field_next
    bool next() { return C::mip_field_next(this); }

    //
    // Additional functions which have no C equivalent
    //

    ///@brief Deserializes the field data to specific field struct.
    ///
    ///@tparam FieldType Any field class from a file in the mip/definitions directory.
    ///
    ///@param[out] field A reference to the field struct to be filled out. Valid
    ///                  only if the function returns true.
    ///@param exact_size If true, the function fails if any bytes remain after deserialization.
    ///
    ///@returns True if the field was successfully deserialized, or false if the field contains
    ///         too few bytes (or to many if exact_size is specified). The field data is not
    ///         valid unless this function returns true.
    template<class FieldType>
    bool extract(FieldType& field, bool exact_size=true) const { return microstrain::extract<microstrain::serialization::Endian::big>(field, payload(), 0, exact_size); }


    ///@brief Determines if the field holds data (and not a command, reply, or response).
    bool isData() const { return isDataDescriptorSet(descriptorSet()); }

    ///@brief Determines if the field is from a command descriptor set (a command, reply, or response field).
    bool isCommandSet() const { return isCommandDescriptorSet(descriptorSet()); }

    ///@brief Determines if the field holds a command.
    bool isCommand() const { return isCommandSet() && isCommandFieldDescriptor(fieldDescriptor()); }

    ///@brief Determines if the field holds an ack/nack reply code.
    bool isReply() const { return isCommandSet() && isReplyFieldDescriptor(fieldDescriptor()) && payloadLength()==2; }

    ///@brief Determines if the field holds command response data (not an ack/nack reply).
    bool isResponse() const { return isCommandSet() && isResponseFieldDescriptor(fieldDescriptor()); }
};


///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
