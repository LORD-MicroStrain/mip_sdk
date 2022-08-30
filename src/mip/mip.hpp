#pragma once

#include <cstring>

#include <stdint.h>
#include <stddef.h>

#include "mip_packet.h"
#include "mip_field.h"
#include "mip_parser.h"
#include "mip_offsets.h"
#include "definitions/descriptors.h"
#include "mip_types.h"

#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cpp  MIP C++ API
///
///@brief This module contains functions and classes for communicating with a
///       MIP device in C++.
///
///@see mip namespace
///

////////////////////////////////////////////////////////////////////////////////
///@brief A collection of C++ classes and functions covering the full
/// mip api.
///
///@see mip_cpp
///
namespace mip
{

using PacketLength = C::packet_length;

template<class Field> struct MipFieldInfo;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a MIP field.
///
class Field : public C::mip_field
{
public:
    /// Construct an empty MIP field.
    Field() { C::mip_field::_payload=nullptr; C::mip_field::_payload_length=0; C::mip_field::_field_descriptor=0x00; C::mip_field::_descriptor_set=0x00; C::mip_field::_remaining_length=0; }
    ///@copydoc mip_field_init()
    Field(uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length) { C::mip_field_init(this, descriptor_set, field_descriptor, payload, payload_length); }
    ///@copydoc mip_field_from_header_ptr()
    Field(const uint8_t* header, uint8_t total_length, uint8_t descriptor_set) { *this = C::mip_field_from_header_ptr(header, total_length, descriptor_set); }
    /// Creates a %Field class from the mip_field C struct.
    Field(const C::mip_field& other) { std::memcpy(static_cast<C::mip_field*>(this), &other, sizeof(C::mip_field)); }

    ///@copydoc mip_field_descriptor_set
    uint8_t descriptorSet() const { return C::mip_field_descriptor_set(this); }
    ///@copydoc mip_field_field_descriptor
    uint8_t fieldDescriptor() const { return C::mip_field_field_descriptor(this); }
    ///@copydoc mip_field_payload_length
    uint8_t payloadLength() const { return C::mip_field_payload_length(this); }
    ///@copydoc mip_field_payload
    const uint8_t* payload() const { return C::mip_field_payload(this); }

    template<class Field>
    bool extract(Field& field) const { return mip::extract(field, payload(), payloadLength(), 0, true); }

    ///@brief Index the payload at the given location.
    ///@param index
    ///@returns payload byte
    uint8_t payload(unsigned int index) const { return payload()[index]; }

    ///@copydoc mip_field_is_valid
    bool isValid() const { return C::mip_field_is_valid(this); }

    ///@copydoc mip_field_next_after
    Field nextAfter() const { return C::mip_field_next_after(this); }
    ///@copydoc mip_field_next
    bool next() { return C::mip_field_next(this); }


    ///@brief Determines if the field is from a command descriptor set (a command, reply, or response field).
    bool isCommandSet() const { return isCommandDescriptorSet(descriptorSet()); }

    ///@brief Determines if the field contains a data field.
    bool isData() const { return isDataDescriptorSet(descriptorSet()); }

    ///@brief Determines if the field holds a command.
    bool isCommand() const { return isCommandSet() && isCommandFieldDescriptor(fieldDescriptor()); }

    ///@brief Determines if the field holds command response data.
    bool isResponse() const { return isCommandSet() && isResponseFieldDescriptor(fieldDescriptor()); }

    ///@brief Determines if the field holds an ack/nack reply code.
    bool isReply() const { return isCommandSet() && isReplyFieldDescriptor(fieldDescriptor()) && payloadLength()==2; }
};


////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a MIP Packet.
///
/// Fields may be iterated over using the C-style method or with a range-based
/// for loop:
///@code{.cpp}
/// for(Field field : packet) { ... }
///@endcode
///
class Packet : public C::mip_packet
{
    class FieldIterator;

public:
    ///@copydoc mip::C::mip_packet_create
    Packet(uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet) { C::mip_packet_create(this, buffer, bufferSize, descriptorSet); }
    ///@copydoc mip_packet_from_buffer
    Packet(uint8_t* buffer, size_t length) { C::mip_packet_from_buffer(this, buffer, length); }
    /// Constructs a C++ %Packet class from the base C object.
    Packet(const C::mip_packet* other) { std::memcpy(static_cast<C::mip_packet*>(this), other, sizeof(*this)); }
    /// Constructs a C++ %Packet class from the base C object.
    Packet(const C::mip_packet& other) { std::memcpy(static_cast<C::mip_packet*>(this), &other, sizeof(*this)); }

    uint8_t      descriptorSet() const { return C::mip_packet_descriptor_set(this); }  ///<@copydoc mip::C::mip_packet_descriptor_set
    PacketLength totalLength()   const { return C::mip_packet_total_length(this);   }  ///<@copydoc mip::C::mip_packet_total_length
    uint8_t      payloadLength() const { return C::mip_packet_payload_length(this); }  ///<@copydoc mip::C::mip_packet_payload_length

    bool isData() const { return C::mip_packet_is_data(this); }
    bool isCommand() const { return !C::mip_packet_is_data(this); }

    const uint8_t* pointer() const { return C::mip_packet_pointer(this); }  ///<@copydoc mip::C::mip_packet_pointer
    const uint8_t* payload() const { return C::mip_packet_payload(this); }  ///<@copydoc mip::C::mip_packet_payload

    uint16_t checksumValue() const { return C::mip_packet_checksum_value(this); }     ///<@copydoc mip::C::mip_packet_checksum_value
    uint16_t computeChecksum() const { return C::mip_packet_compute_checksum(this); } ///<@copydoc mip::C::mip_packet_compute_checksum

    bool isSane() const { return C::mip_packet_is_sane(this); }    ///<@copydoc mip::C::mip_packet_is_sane
    bool isValid() const { return C::mip_packet_is_valid(this); }  ///<@copydoc mip::C::mip_packet_is_valid
    bool isEmpty() const { return C::mip_packet_is_empty(this); }  ///<@copydoc mip::C::mip_packet_is_empty

    PacketLength bufferSize() const { return C::mip_packet_buffer_size(this); }            ///<@copydoc mip::C::mip_packet_buffer_size
    RemainingCount remainingSpace() const { return C::mip_packet_remaining_space(this); }  ///<@copydoc mip::C::mip_packet_remaining_space

    bool addField(uint8_t fieldDescriptor, const uint8_t* payload, size_t payloadLength) { return C::mip_packet_add_field(this, fieldDescriptor, payload, payloadLength); }  ///<@copydoc mip::C::mip_packet_add_field
    RemainingCount allocField(uint8_t fieldDescriptor, uint8_t payloadLength, uint8_t** payloadPtr_out) { return C::mip_packet_alloc_field(this, fieldDescriptor, payloadLength, payloadPtr_out); }  ///<@copydoc mip::C::mip_packet_alloc_field
    RemainingCount reallocLastField(uint8_t* payloadPtr, uint8_t newPayloadLength) { return C::mip_packet_realloc_last_field(this, payloadPtr, newPayloadLength); }  ///<@copydoc mip::C::mip_packet_realloc_last_field
    RemainingCount cancelLastField(uint8_t* payloadPtr) { return C::mip_packet_cancel_last_field(this, payloadPtr); }  ///<@copydoc mip::C::mip_packet_cancel_last_field

    void finalize() { C::mip_packet_finalize(this); }  ///<@copydoc mip::C::mip_packet_finalize

    void reset(uint8_t descSet) { C::mip_packet_reset(this, descSet); }  ///<@copydoc mip::C::mip_packet_reset
    void reset() { reset(descriptorSet()); }  ///<@brief Resets the packet using the same descriptor set.

    /// Returns the first field in the packet.
    Field firstField() const { return Field(C::mip_field_first_from_packet(this)); }

    /// Returns a forward iterator to the first field in the packet.
    ///@internal
    FieldIterator begin() const { return firstField(); }

    /// Returns a sentry object representing the end of fields in the packet.
    ///@internal
#if __cpp_range_based_for >= 201603
    std::nullptr_t     end() const { return nullptr; }
#else
    FieldIterator end() const { return Field(); }
#endif

    template<class Field>
    bool addField(const Field& field, uint8_t fieldDescriptor = Field::FIELD_DESCRIPTOR)
    {
        uint8_t* payload;
        size_t available = allocField(fieldDescriptor, 0, &payload);
        Serializer serializer(payload, available);
        insert(serializer, field);
        return reallocLastField(payload, serializer.length()) >= 0;
    }

    template<class Field>
    static Packet createFromField(uint8_t* buffer, size_t bufferSize, const Field& field, uint8_t fieldDescriptor=Field::FIELD_DESCRIPTOR)
    {
        Packet packet(buffer, bufferSize, Field::DESCRIPTOR_SET);
        packet.addField<Field>(field, fieldDescriptor);
        packet.finalize();
        return packet;
    }

private:
    /// Iterator class for use with the range-based for loop.
    ///@internal
    class FieldIterator
    {
    public:
        FieldIterator(const Field& first) : mField(first) {}
        FieldIterator() {}

        bool operator==(const FieldIterator& other) const {
            // Required to make invalid fields equivalent for range-based for loop
            if( !mField.isValid() && !other.mField.isValid() )
                return true;
            return mField.descriptorSet() == other.mField.descriptorSet() && mField.fieldDescriptor() == other.mField.fieldDescriptor() && mField.payload() == other.mField.payload();
        }
        bool operator!=(const FieldIterator& other) const { return !(*this == other); }

        bool operator==(std::nullptr_t) const { return !mField.isValid(); }
        bool operator!=(std::nullptr_t) const { return mField.isValid(); }

        const Field& operator*() const { return mField; }

        FieldIterator& operator++() { mField.next(); return *this; }
    private:
        Field mField;
    };

};


////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a MIP parser.
///
/// See @ref parsing_packets
///
class Parser : public C::mip_parser
{
public:
    ///@copydoc mip::C::mip_parser_init
    Parser(uint8_t* buffer, size_t bufferSize, C::mip_packet_callback callback, void* callbackObject, Timeout timeout) { C::mip_parser_init(this, buffer, bufferSize, callback, callbackObject, timeout); }
    ///@copydoc mip::C::mip_parser_init
    Parser(uint8_t* buffer, size_t bufferSize, bool (*callback)(void*,const Packet*,Timestamp), void* callbackObject, Timeout timeout) { C::mip_parser_init(this, buffer, bufferSize, (C::mip_packet_callback)callback, callbackObject, timeout); }

    Parser(uint8_t* buffer, size_t bufferSize, Timeout timeout) { C::mip_parser_init(this, buffer, bufferSize, nullptr, nullptr, timeout); }

    template<class T, bool (T::*Callback)(const Packet&, Timestamp)>
    void setCallback(T& object);

    ///@copydoc mip::C::mip_parser_reset
    void reset() { C::mip_parser_reset(this); }

    ///@copydoc mip::C::mip_parser_parse
    RemainingCount parse(const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp, unsigned int maxPackets) { return C::mip_parser_parse(this, inputBuffer, inputCount, timestamp, maxPackets); }

    ///@copydoc mip::C::mip_parser_timeout
    Timeout timeout() const { return C::mip_parser_timeout(this); }
    ///@copydoc mip::C::mip_parser_set_timeout
    void setTimeout(Timeout timeout) { return C::mip_parser_set_timeout(this, timeout); }
};


///@brief Initializes the MIP Parser
///
/// This version allows binding a member function instead of a C-style callback.
/// Example:
///@code{.cpp}
/// struct MyClass
/// {
///     void handlePacket(const Packet& packet, Timeout timeout);
/// };
/// MyClass myInstance;
/// Parser parser<MyClass, &MyClass::handlePacket>(myInstance);
///@endcode
///
///@tparam T Class type containing the member function to be called.
///@tparam Callback A pointer to a member function on a T to be called when a
///        packet is parsed.
///
///@param object
///       Instance of T to call the callback.
///
template<class T, bool (T::*Callback)(const Packet&, Timestamp)>
void Parser::setCallback(T& object)
{
    C::mip_packet_callback callback = [](void* obj, const C::mip_packet* pkt, Timestamp timestamp)->bool
    {
        return (static_cast<T*>(obj)->*Callback)(Packet(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, &object);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Read data from a source into the internal parsing buffer.
///
///@tparam Function
/// A function-like object with the following signature:
/// `bool read(size_t maxCount, size_t* count_out, Timestamp* timestamp_out);`
/// The parameters are as follows:
/// @li buffer - Buffer into which to write data.
/// @li maxCount - The maximum number of bytes to read.
/// @li count_out - Updated with the number of bytes actually read.
/// @li timestamp_out - Updated with the timestamp of the data.
///
///@param parser
///
///@param reader
///       A callback function, lambda, or similar which will read data into the
///       buffer and capture the timestamp. It should return true if successful
///       or false otherwise. If it returns false, parsing is skipped. The read
///       function may also throw an exception instead of returning false.
///
///@param maxPackets
///       Maximum number of packets to parse, just like for Parser::parse().
///
///@return Same as the return value of reader.
///
template<class Function>
bool parseMipDataFromSource(C::mip_parser& parser, Function reader, size_t maxPackets)
{
    uint8_t* ptr;
    size_t maxCount = C::mip_parser_get_write_ptr(&parser, &ptr);

    size_t count;
    Timestamp timestamp;
    if( !reader(ptr, maxCount, &count, &timestamp) )
        return false;

    assert(count <= maxCount);

    C::mip_parser_process_written(&parser, count, timestamp, maxPackets);

    return true;
}

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace mip
