
#include "mip_packet.h"
#include "mip_offsets.h"

#include <string.h>
#include <assert.h>


//
// Initialization
//

void MipPacket_init(struct MipPacket* packet, uint8_t* buffer, size_t length)
{
    assert(buffer != NULL);

    // Can't use more than the max length, otherwise the payload length byte could overflow.
    // Limiting this here lets us avoid checking every time the remaining count is used.
    if( length > MIP_PACKET_LENGTH_MAX )
        length = MIP_PACKET_LENGTH_MAX;

    packet->buffer = buffer;
    packet->length = length;
}

void MipPacket_create(struct MipPacket* packet, uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet)
{
    MipPacket_init(packet, buffer, bufferSize);

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

uint8_t MipPacket_descriptorSet(const struct MipPacket* packet)
{
    return packet->buffer[MIP_INDEX_DESCSET];
}

uint8_t MipPacket_payloadLength(const struct MipPacket* packet)
{
    return packet->buffer[MIP_INDEX_LENGTH];
}

uint_least16_t MipPacket_totalLength(const struct MipPacket* packet)
{
    return MipPacket_payloadLength(packet) + MIP_PACKET_LENGTH_MIN;
}

const uint8_t* MipPacket_buffer(const struct MipPacket* packet)
{
    return packet->buffer;
}

const uint8_t* MipPacket_payload(const struct MipPacket* packet)
{
    return packet->buffer + MIP_INDEX_PAYLOAD;
}

uint16_t MipPacket_checksumValue(const struct MipPacket* packet)
{
    const uint_least16_t index = MipPacket_totalLength(packet) - MIP_CHECKSUM_LENGTH;

    return ((uint16_t)(packet->buffer[index+0]) << 8) | (uint16_t)(packet->buffer[index+1]);
}


bool MipPacket_isSane(const struct MipPacket* packet)
{
    return packet->buffer && (MipPacket_bufferSize(packet) > MIP_PACKET_LENGTH_MIN) && (MipPacket_descriptorSet(packet) != 0x00);
}



uint_least16_t MipPacket_bufferSize(const struct MipPacket* packet)
{
    return packet->length;
}

RemainingCount MipPacket_remainingSpace(const struct MipPacket* packet)
{
    return MipPacket_bufferSize(packet) - MipPacket_totalLength(packet);
}


//
// Packet Building
//

uint16_t MipPacket_computeChecksum(const struct MipPacket* packet)
{
    uint8_t a = 0;
    uint8_t b = 0;

    const uint_least16_t length = MipPacket_totalLength(packet) - MIP_CHECKSUM_LENGTH;

    for(uint_least16_t i=0; i<length; i++)
    {
        a += packet->buffer[i];
        b += a;
    }

    return ((uint16_t)(a) << 8) | (uint16_t)(b);
}

bool MipPacket_addField(struct MipPacket* packet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength)
{
    uint8_t* payloadBuffer;
    RemainingCount remaining = MipPacket_allocField(packet, fieldDescriptor, payloadLength, &payloadBuffer);
    if( remaining < 0 )
        return false;

    memcpy(payloadBuffer, payload, payloadLength);

    return true;
}

RemainingCount MipPacket_allocField(struct MipPacket* packet, uint8_t fieldDescriptor, uint8_t payloadLength, uint8_t** payloadPtr_out)
{
    // assert( payloadLength <= MIP_FIELD_PAYLOAD_LENGTH_MAX );

    const RemainingCount remaining = MipPacket_remainingSpace(packet);

    const uint_least16_t fieldLength = MIP_FIELD_HEADER_LENGTH + (uint_least16_t)payloadLength;

    if( fieldLength < remaining )
    {
        uint_least16_t fieldIndex = MIP_HEADER_LENGTH + MipPacket_payloadLength(packet);

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
    uint_least16_t length = MipPacket_totalLength(packet) - MIP_CHECKSUM_LENGTH;

    packet->buffer[length+0] = checksum >> 8;
    packet->buffer[length+1] = checksum & 0xFF;
}
