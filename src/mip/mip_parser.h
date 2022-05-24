#pragma once

#include "mip_packet.h"
#include "mip_offsets.h"

#include "utils/byte_ring.h"


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipParser MipParser - Functions for parsing MIP packets
///@{
///

///@brief Type used for packet timestamps and timeouts.
///
/// Timestamps must be monotonic except for overflow at the maximum value back to 0.
/// The units can be anything, but typically are milliseconds. Choose a long enough
/// unit so that consecutive calls to the parser will not exceed half of the maximum
/// value for this type. For milliseconds, the time to overflow is approximately 50
/// days, so the parser should be invoked at least every 25 days. Failure to observe
/// this requirement may result in false timeouts or delays in getting parsed packets.
///
typedef uint32_t Timestamp;


///@brief Callback function which receives parsed MIP packets.
///@param user A user-specified pointer which will be given the callbackObject parameter which was previously passed to MipParser_init.
///@param Packet A pointer to the MIP packet. Do not store this pointer as it will be invalidated after the callback returns.
///@param timestamp The approximate time the packet was parsed.
typedef bool (*PacketCallback)(void* user, const struct MipPacket* packet, Timestamp timestamp);



///@brief MIP Parser state.
struct MipParsingState
{
    Timestamp startTime;                         ///<@internal The timestamp when the first byte was observed by the parser.
    Timestamp timeout;                           ///<@internal Duration to wait for the rest of the data in a packet.
    uint8_t resultBuffer[MIP_PACKET_LENGTH_MAX]; ///<@internal Buffer used to output MIP packets to the callback.
    PacketLength expectedLength;                 ///<@internal Expected length of the packet currently being parsed. Keeps track of parser state.
    struct ByteRingState ring;                   ///<@internal Ring buffer which holds data being parsed. User-specified backing buffer and size.
    PacketCallback callback;                     ///<@internal Callback called when a valid packet is parsed.
    void* callbackObject;                        ///<@internal User-specified pointer passed to the callback function.
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

Timestamp MipParser_timeout(const struct MipParsingState* parser);
void MipParser_setTimeout(struct MipParsingState* parser, Timestamp timeout);

///@}
////////////////////////////////////////////////////////////////////////////////
