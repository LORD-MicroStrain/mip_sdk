#pragma once

#include "../types.h"

#ifdef __cplusplus
namespace mscl{
namespace C {
extern "C" {
#endif



////////////////////////////////////////////////////////////////////////////////
///@defgroup MipPacket MipPacket - Functions for handling MIP packets.
///
/// A MIP Packet is represented by the MipPacket struct.
///
///~~~
/// +-------+-------+------+------+------------+-----/ /----+------------+----
/// | SYNC1 | SYNC2 | DESC | PLEN |   Field    |     ...    |  Checksum  |  remaining buffer space...
/// +-------+-------+------+------+------------+-----/ /----+------------+----
///~~~
///
///@{


typedef uint_least16_t PacketLength;  ///< Type used for the length of a MIP packet.

////////////////////////////////////////////////////////////////////////////////
///@brief Structure representing a MIP Packet.
///
/// Use to inspect received packets or construct new ones.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
struct MipPacket
{
    uint8_t*       buffer;        ///<@private Pointer to the packet data.
    uint_least16_t bufferLength;  ///<@private Length of the buffer (NOT the packet length!).
};


////////////////////////////////////////////////////////////////////////////////
///@defgroup PacketBuilding  Packet Building - Functions for building new MIP packets.
///
/// Use these functions to create a new packet, add fields, and write the
/// checksum.
///
///@{

void MipPacket_create(struct MipPacket* packet, uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet);

bool           MipPacket_addField(struct MipPacket* packet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength);
RemainingCount MipPacket_allocField(struct MipPacket* packet, uint8_t fieldDescriptor, uint8_t payloadLength, uint8_t** payloadPtr_out);
RemainingCount MipPacket_reallocLastField(struct MipPacket* packet, uint8_t* payloadPtr, uint8_t newPayloadLength);

void MipPacket_finalize(struct MipPacket* packet);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup Accessors  Accessors - Functions for accessing information about an existing MIP packet.
///
/// Use these functions to get information about a MIP packet after it has been
/// parsed. Generally, first the descriptor set would be inspected followed by
/// iterating the fields using the MipFieldIteration functions.
///
/// With the exception of MipPacket_checksumValue() (and any function which
/// calls it, e.g. MipPacket_isValid()), these functions may also be used on
/// packets which are under construction via the PacketBuilding functions.
///
///@{

void MipPacket_fromBuffer(struct MipPacket* packet, uint8_t* buffer, size_t length);

uint8_t        MipPacket_descriptorSet(const struct MipPacket* packet);
PacketLength   MipPacket_totalLength(const struct MipPacket* packet);
uint8_t        MipPacket_payloadLength(const struct MipPacket* packet);
const uint8_t* MipPacket_pointer(const struct MipPacket* packet);
const uint8_t* MipPacket_payload(const struct MipPacket* packet);
uint16_t       MipPacket_checksumValue(const struct MipPacket* packet);
uint16_t       MipPacket_computeChecksum(const struct MipPacket* packet);


bool           MipPacket_isSane(const struct MipPacket* packet);
bool           MipPacket_isValid(const struct MipPacket* packet);

PacketLength   MipPacket_bufferSize(const struct MipPacket* packet);
RemainingCount MipPacket_remainingSpace(const struct MipPacket* packet);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mscl
} // namespace C
} // extern "C"
#endif
