#pragma once

#include "types.h"


struct MipPacket
{
    uint8_t*       buffer;
    uint_least16_t length;
};


//
// Initialization / creation functions (use only one)
//

void MipPacket_init(struct MipPacket* packet, uint8_t* buffer, size_t length);
void MipPacket_create(struct MipPacket* packet, uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet);


//
// Accessors
//

uint8_t        MipPacket_descriptorSet(const struct MipPacket* packet);
uint_least16_t MipPacket_totalLength(const struct MipPacket* packet);
uint8_t        MipPacket_payloadLength(const struct MipPacket* packet);
const uint8_t* MipPacket_buffer(const struct MipPacket* packet);
const uint8_t* MipPacket_payload(const struct MipPacket* packet);
uint16_t       MipPacket_checksumValue(const struct MipPacket* packet);

bool           MipPacket_isSane(const struct MipPacket* packet);

uint_least16_t MipPacket_bufferSize(const struct MipPacket* packet);
RemainingCount MipPacket_remainingSpace(const struct MipPacket* packet);


//
// Packet Building
//

uint16_t MipPacket_computeChecksum(const struct MipPacket* packet);

bool           MipPacket_addField(struct MipPacket* packet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength);
RemainingCount MipPacket_allocField(struct MipPacket* packet, uint8_t fieldDescriptor, uint8_t payloadLength, uint8_t** payloadPtr_out);
RemainingCount MipPacket_reallocLastField(struct MipPacket* packet, uint8_t* payloadPtr, uint8_t newPayloadLength);

void MipPacket_finalize(struct MipPacket* packet);
