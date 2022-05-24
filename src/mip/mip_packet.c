
#include "mip_packet.h"
#include "mip_offsets.h"

#include <string.h>
#include <assert.h>


//
// Initialization
//

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes a MIP packet from an existing buffer.
///
/// Use this when receiving or parsing MIP packets.
///
/// The data in the buffer should be a valid or suspected MIP packet.
///
///@param packet
///@param buffer
///       The data buffer containing the bytes for a MIP packet. Must be at
///       least MIP_PACKET_LENGTH_MIN bytes in size.
///@param length
///       The length of the data pointed to by buffer.
///
///@note The data does not need to be a valid MIP packet, for instance to use
///      the MipPacket_isSane() or MipPacket_isValid() functions. However, if
///      it is NOT a valid MIP packet, the result of calling any accessor
///      function is unpredictable. In particular, if length is less than
///      MIP_PACKET_LENGTH_MIN bytes, calling the accessor functions is undefined
///      behavior.
///
void MipPacket_fromBuffer(struct MipPacket* packet, uint8_t* buffer, size_t length)
{
    assert(buffer != NULL);

    // Limit the length in case it's longer than a mip packer (or worse, longer than the buffer size field can hold)
    if( length > MIP_PACKET_LENGTH_MAX )
        length = MIP_PACKET_LENGTH_MAX;

    packet->buffer = buffer;
    packet->length = length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Create a brand-new MIP packet in the given buffer.
///
/// Use this along with the packet building functions to create MIP packets.
///
///@param packet
///@param buffer
///       This is where the packet bytes will be stored. Must be at least
///       MIP_PACKET_LENGTH_MIN bytes in size.
///@param bufferSize
///       The size of buffer, in bytes.
///@param descriptorSet
///       The MIP descriptor set for the packet.
///
void MipPacket_create(struct MipPacket* packet, uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet)
{
    MipPacket_fromBuffer(packet, buffer, bufferSize);

    if( bufferSize < MIP_PACKET_LENGTH_MIN )
    {
        assert(false); // Buffer too small!
        return;
    }

    packet->buffer[MIP_INDEX_SYNC1] = MIP_SYNC1;
    packet->buffer[MIP_INDEX_SYNC2] = MIP_SYNC2;
    packet->buffer[MIP_INDEX_DESCSET] = descriptorSet;
    packet->buffer[MIP_INDEX_LENGTH] = 0;
}



//
// Accessors
//

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the MIP descriptor set for this packet.
///
uint8_t MipPacket_descriptorSet(const struct MipPacket* packet)
{
    return packet->buffer[MIP_INDEX_DESCSET];
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the payload (MIP fields).
///
uint8_t MipPacket_payloadLength(const struct MipPacket* packet)
{
    return packet->buffer[MIP_INDEX_LENGTH];
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the total length of the packet, in bytes.
///
PacketLength MipPacket_totalLength(const struct MipPacket* packet)
{
    return MipPacket_payloadLength(packet) + MIP_PACKET_LENGTH_MIN;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a pointer to the data buffer containing the packet.
///
const uint8_t* MipPacket_pointer(const struct MipPacket* packet)
{
    return packet->buffer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a pointer to the packet's payload (the first field).
///
const uint8_t* MipPacket_payload(const struct MipPacket* packet)
{
    return packet->buffer + MIP_INDEX_PAYLOAD;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the value of the checksum as written in the packet.
///
/// This function does not compute the checksum. To do so, use
/// MipPacket_computeChecksum().
///
uint16_t MipPacket_checksumValue(const struct MipPacket* packet)
{
    const PacketLength index = MipPacket_totalLength(packet) - MIP_CHECKSUM_LENGTH;

    return ((uint16_t)(packet->buffer[index+0]) << 8) | (uint16_t)(packet->buffer[index+1]);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the packet buffer is large enough, not NULL, and has a valid
///         descriptor set.
///
bool MipPacket_isSane(const struct MipPacket* packet)
{
    return packet->buffer && (MipPacket_bufferSize(packet) > MIP_PACKET_LENGTH_MIN) && (MipPacket_descriptorSet(packet) != 0x00);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if MipPacket_isSane() returns true and the checksum is valid.
///
bool MipPacket_isValid(const struct MipPacket* packet)
{
    if( !MipPacket_isSane(packet) )
        return false;

    const uint16_t listedChecksum = MipPacket_checksumValue(packet);
    const uint16_t computedChecksum = MipPacket_computeChecksum(packet);

    return listedChecksum == computedChecksum;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the size of the buffer backing the MIP packet.
///
///@note This is the BUFFER SIZE and not the packet length.
///
PacketLength MipPacket_bufferSize(const struct MipPacket* packet)
{
    return packet->length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the remaining space available for more payload data.
///
/// This is equal to the buffer size less the total packet length.
///
RemainingCount MipPacket_remainingSpace(const struct MipPacket* packet)
{
    return MipPacket_bufferSize(packet) - MipPacket_totalLength(packet);
}


//
// Packet Building
//

////////////////////////////////////////////////////////////////////////////////
///@brief Computes the checksum of the MIP packet.
///
///@returns The computed checksum value.
///
uint16_t MipPacket_computeChecksum(const struct MipPacket* packet)
{
    uint8_t a = 0;
    uint8_t b = 0;

    const PacketLength length = MipPacket_totalLength(packet) - MIP_CHECKSUM_LENGTH;

    for(PacketLength i=0; i<length; i++)
    {
        a += packet->buffer[i];
        b += a;
    }

    return ((uint16_t)(a) << 8) | (uint16_t)(b);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Adds a pre-constructed MIP field to the packet.
///
///@param packet
///@param fieldDescriptor
///       The MIP field descriptor (e.g. command or data descriptor).
///@param payload
///       A pointer to the field payload data (without the header).
///@param payloadLength
///       The length of the payload data. Must be less than or equal to
///       MIP_FIELD_PAYLOAD_LENGTH_MAX. Does not include the header.
///
///@returns true if the field was added, or false if there was not enough space.
///
bool MipPacket_addField(struct MipPacket* packet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength)
{
    uint8_t* payloadBuffer;
    RemainingCount remaining = MipPacket_allocField(packet, fieldDescriptor, payloadLength, &payloadBuffer);
    if( remaining < 0 )
        return false;

    memcpy(payloadBuffer, payload, payloadLength);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Allocate a MIP field within the packet and return the payload pointer.
///
///@param packet
///@param fieldDescriptor
///       The MIP field descriptor (e.g. command or data descriptor).
///@param payloadLength
///       The requested length of the field payload (not including the header).
///       If the size is not known ahead of time, pass 0 and inspect the return
///       value to see how much payload data can be written. Then call
///       MipPacket_reallocField() with the used size and same payload pointer.
///@param payloadPtr_out
///       A pointer to a pointer to the field payload. This will receive the
///       payload pointer into which data should be written.
///
///@returns The amount of space remaining after allocating this field. If this
///         is negative, the field could not be allocated and the payload must
///         not be written.
///
RemainingCount MipPacket_allocField(struct MipPacket* packet, uint8_t fieldDescriptor, uint8_t payloadLength, uint8_t** payloadPtr_out)
{
    // assert( payloadLength <= MIP_FIELD_PAYLOAD_LENGTH_MAX );

    const RemainingCount remaining = MipPacket_remainingSpace(packet);

    const PacketLength fieldLength = MIP_FIELD_HEADER_LENGTH + (PacketLength)payloadLength;

    *payloadPtr_out = NULL;

    if( fieldLength <= remaining )
    {
        PacketLength fieldIndex = MIP_HEADER_LENGTH + MipPacket_payloadLength(packet);

        packet->buffer[MIP_INDEX_LENGTH] += fieldLength;

        packet->buffer[fieldIndex+MIP_INDEX_FIELD_LEN]  = fieldLength;
        packet->buffer[fieldIndex+MIP_INDEX_FIELD_DESC] = fieldDescriptor;

        *payloadPtr_out = &packet->buffer[fieldIndex + MIP_INDEX_FIELD_PAYLOAD];
    }

    return remaining - fieldLength;
}

// RemainingCount MipPacket_reallocLastField(struct MipPacket* packet, uint8_t* payloadPtr, uint8_t newPayloadLength)
// {
//     assert( payloadLength <= MIP_FIELD_PAYLOAD_LENGTH_MAX );
//
//     const RemainingCount remaining = MipPacket_remainingSpace(packet);
//
//     if( payloadLength < remaining )
//     {
//     }
// }

void MipPacket_finalize(struct MipPacket* packet)
{
    uint16_t checksum = MipPacket_computeChecksum(packet);
    PacketLength length = MipPacket_totalLength(packet) - MIP_CHECKSUM_LENGTH;

    packet->buffer[length+0] = checksum >> 8;
    packet->buffer[length+1] = checksum & 0xFF;
}
