#pragma once

#include "mip_packet.h"
#include "mip_offsets.h"

#include "utils/byte_ring.h"
#include "../types.h"

#ifdef __cplusplus
namespace mscl{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipParser MipParser - Functions for parsing MIP packets
///@{
///


///@brief Callback function which receives parsed MIP packets.
///@param user A user-specified pointer which will be given the callbackObject parameter which was previously passed to MipParser_init.
///@param Packet A pointer to the MIP packet. Do not store this pointer as it will be invalidated after the callback returns.
///@param timestamp The approximate time the packet was parsed.
typedef bool (*PacketCallback)(void* user, const struct MipPacket* packet, Timestamp timestamp);


////////////////////////////////////////////////////////////////////////////////
///@brief MIP Parser state.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
struct MipParsingState
{
    Timestamp startTime;                         ///<@private The timestamp when the first byte was observed by the parser.
    Timestamp timeout;                           ///<@private Duration to wait for the rest of the data in a packet.
    uint8_t resultBuffer[MIP_PACKET_LENGTH_MAX]; ///<@private Buffer used to output MIP packets to the callback.
    PacketLength expectedLength;                 ///<@private Expected length of the packet currently being parsed. Keeps track of parser state. Always 1, MIP_HEADER_LENGTH, or at least MIP_PACKET_LENGTH_MAX.
    struct ByteRingState ring;                   ///<@private Ring buffer which holds data being parsed. User-specified backing buffer and size.
    PacketCallback callback;                     ///<@private Callback called when a valid packet is parsed.
    void* callbackObject;                        ///<@private User-specified pointer passed to the callback function.
};



#define MIPPARSER_UNLIMITED_PACKETS   0   ///< Specifies no limit when used as the maxPackets argument to MipParser_parse.
#define MIPPARSER_DEFAULT_TIMEOUT_MS 100  ///< Specifies the default timeout for a MIP parser, assuming timestamps are in milliseconds.


void MipParser_init(struct MipParsingState* parser, uint8_t* buffer, size_t bufferSize, PacketCallback callback, void* callbackObject, Timestamp timeout);
bool MipParser_parseOnePacketFromRing(struct MipParsingState* parser, struct MipPacket* packet_out, Timestamp timestamp);
RemainingCount MipParser_parse(struct MipParsingState* parser, const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp, unsigned int maxPackets);

void MipParser_reset(struct MipParsingState* parser);

//
// Accessors
//

Timeout MipParser_timeout(const struct MipParsingState* parser);
void MipParser_setTimeout(struct MipParsingState* parser, Timeout timeout);

Timestamp MipParser_lastPacketTimestamp(const struct MipParsingState* parser);

///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mscl
} // namespace C
} // extern "C"
#endif
