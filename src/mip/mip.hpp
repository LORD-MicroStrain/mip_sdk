#pragma once

#include <cstring>

#include <stdint.h>
#include <stddef.h>

namespace mscl
{
namespace C
{
extern "C" {
    #include <mip/mip_packet.h>
    #include <mip/mip_field.h>
    #include <mip/mip_parser.h>
    #include <mip/mip_offsets.h>
}
}


using C::PacketLength;
using C::RemainingCount;
using C::Timestamp;


class MipField : public C::MipField
{
public:
    MipField() { C::MipField::payload=nullptr; C::MipField::payloadLength=0; C::MipField::fieldDescriptor=0x00; C::MipField::descriptorSet=0x00; C::MipField::remainingLength=0; }
    MipField(uint8_t descriptorSet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength) { C::MipField_init(this, descriptorSet, fieldDescriptor, payload, payloadLength); }
    MipField(const uint8_t* header, uint8_t totalLength, uint8_t descriptorSet) { *this = C::MipField_fromHeaderPtr(header, totalLength, descriptorSet); }
    MipField(const C::MipField& other) { std::memcpy(static_cast<C::MipField*>(this), &other, sizeof(C::MipField)); }

    uint8_t descriptorSet() const { return C::MipField_descriptorSet(this); }
    uint8_t fieldDescriptor() const { return C::MipField_fieldDescriptor(this); }
    uint8_t payloadLength() const { return C::MipField_payloadLength(this); }
    const uint8_t* payload() const { return C::MipField_payload(this); }

    bool isValid() const { return C::MipField_isValid(this); }

    MipField nextAfter() const { return C::MipField_nextAfter(this); }
    bool next() { return MipField_next(this); }
};


class MipPacket : public C::MipPacket
{
public:
    MipPacket(uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet) { C::MipPacket_create(this, buffer, bufferSize, descriptorSet); }
    MipPacket(uint8_t* buffer, size_t length) { C::MipPacket_fromBuffer(this, buffer, length); }
    MipPacket(const C::MipPacket& other) { std::memcpy(static_cast<C::MipPacket*>(this), &other, sizeof(*this)); }

    uint8_t      descriptorSet() const { return C::MipPacket_descriptorSet(this); }
    PacketLength totalLength()   const { return C::MipPacket_totalLength(this);   }
    uint8_t      payloadLength() const { return C::MipPacket_payloadLength(this); }

    const uint8_t* pointer() const { return C::MipPacket_pointer(this); }
    const uint8_t* payload() const { return C::MipPacket_payload(this); }

    uint16_t checksumValue() const { return C::MipPacket_checksumValue(this); }
    uint16_t MipPacket_computeChecksum() const { return C::MipPacket_computeChecksum(this); }

    bool isSane() const { return C::MipPacket_isSane(this); }
    bool isValid() const { return C::MipPacket_isValid(this); }

    PacketLength bufferSize() const { return C::MipPacket_bufferSize(this); }
    RemainingCount remainingSpace() const { return C::MipPacket_remainingSpace(this); }

    bool addField(uint8_t fieldDescriptor, const uint8_t* payload, size_t payloadLength) { return C::MipPacket_addField(this, fieldDescriptor, payload, payloadLength); }
    RemainingCount allocField(uint8_t fieldDescriptor, uint8_t payloadLength, uint8_t** payloadPtr_out) { return C::MipPacket_allocField(this, fieldDescriptor, payloadLength, payloadPtr_out); }
    // RemainingCount reallocLastField(uint8_t* payloadPtr, uint8_t newPayloadLength) { return C::MipPacket_reallocField(this, payload, newPayloadLength); }

    void finalize() { C::MipPacket_finalize(this); }

    MipField firstField() const { return MipField(C::MipField_fromPacket(this)); }

    class FieldIterator
    {
    public:
        FieldIterator(const MipField& first) : mField(first) {}
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

        const MipField& operator*() const { return mField; }

        FieldIterator& operator++() { mField.next(); return *this; }
    private:
        MipField mField;
    };

    FieldIterator begin() const { return firstField(); }
#if __cpp_range_based_for >= 201603
    nullptr_t     end() const { return nullptr; }
#else
    FieldIterator end() const { return MipField(); }
#endif
};


class MipParser : public C::MipParsingState
{
public:
    MipParser(uint8_t* buffer, size_t bufferSize, C::PacketCallback, void* callbackObject, Timestamp timeout) { C::MipParser_init(this, buffer, bufferSize, callback, callbackObject, timeout); }
    MipParser(uint8_t* buffer, size_t bufferSize, bool (*callback)(void*,const MipPacket*,Timestamp), void* callbackObject, Timestamp timeout) { C::MipParser_init(this, buffer, bufferSize, (C::PacketCallback)callback, callbackObject, timeout); }

    void reset() { C::MipParser_reset(this); }

    RemainingCount parse(const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp, unsigned int maxPackets) { return C::MipParser_parse(this, inputBuffer, inputCount, timestamp, maxPackets); }

    Timestamp timeout() const { return C::MipParser_timeout(this); }
    void setTimeout(Timestamp timeout) { return C::MipParser_setTimeout(this, timeout); }
};

} // namespace mscl
