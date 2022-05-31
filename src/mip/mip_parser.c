
#include "mip_parser.h"

#include "mip_offsets.h"


#define MIPPARSER_RESET_LENGTH 1

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MIP parser.
///
///
///@param parser
///@param buffer
///       Scratch space for the parser to use internally; input data is consumed
///       and fed to this buffer.
///@param bufferSize
///       Size of buffer, in bytes.
///@param callback
///       A function to be called when a valid packet is identified. It will be
///       passed an optional user-supplied parameter, a pointer to the packet,
///       and the time the first byte was parsed.
///@param callbackObject
///       An optional user-specified pointer which is directly passed to
///       the callback as the first parameter.
///@param timeout
///       The timeout for receiving one packet. Depends on the serial baud rate
///       and is typically 100 milliseconds.
///
void MipParser_init(struct MipParsingState* parser, uint8_t* buffer, size_t bufferSize, PacketCallback callback, void* callbackObject, Timestamp timeout)
{
    parser->startTime = 0;
    parser->timeout = timeout;

    parser->resultBuffer[0] = 0;

    ByteRing_init(&parser->ring, buffer, bufferSize);

    parser->expectedLength = MIPPARSER_RESET_LENGTH;

    parser->callback = callback;
    parser->callbackObject = callbackObject;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Resets the MIP parser.
///
/// Clears the current packet and internal buffer. The parser will be restored
/// as if MipParser_init had just been called.
///
///
///@param parser
///
void MipParser_reset(struct MipParsingState* parser)
{
    parser->expectedLength = MIPPARSER_RESET_LENGTH;
    parser->resultBuffer[0] = 0;
    parser->startTime = 0;
    ByteRing_clear(&parser->ring);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Parses packets from the input data buffer.
///
/// For every valid MIP packet, the callback function will be called with the
/// packet and timestamp.
///
///
///@param parser
///@param inputBuffer
///       Pointer to bytes received from the device or file. This buffer may
///       contain non-mip data (e.g. NMEA 0183), which will be ignored.
///       This buffer may be NULL if inputCount is 0.
///@param inputCount
///       The number of bytes in the input buffer.
///@param timestamp
///       The local time the data was received. This is used to check for
///       timeouts and is passed to the callback as the packet timestamp.
///@param maxPackets
///       The maximum number of packets to process. Unprocessed data is left in
///       the internal buffer. If 0, processing runs until no complete packets
///       remain in the buffer.
///
///@returns The number of bytes left unprocessed from the input buffer.
///         If maxPackets is 0, this will also be zero as all of the data will
///         be consumed. Data may still remain in the internal buffer.
///
///@note If maxPackets is 0, then this function is guaranteed to consume all
///      of the input data and the buffer can be reused or discarded afterward.
///      However, if maxPackets is nonzero (meaning the number of packets parsed
///      will be limited), then this is no longer guaranteed as the excess data
///      may fill up the internal bufffer. In this case, you must process packets
///      faster than they arrive on average. For bursty data (e.g. GNSS data),
///      use a large internal buffer (see MipParser_init) to help average out
///      the packet processing load.
///
///@note The timestamp of parsed packets is based on the time the packet was
///      parsed. When maxPackets==0, this is the same as the input timestamp.
///      When maxPackets!=0, packets received during an earlier parse call
///      may be timestamped with the time from a later parse call. Therefore,
///      if packet timestamping is critical to your application, avoid using
///      maxPackets > 0.
///
///@note The parser will do its best to ignore non-MIP data. However, it is
///      possible for some binary data to appear to be a MIP packet if it
///      conntains 0x75,0x65, has at least 6 bytes, and has a valid checksum. A
///      16-bit checksum has a 1 in 65,536 chance of appearing to be valid.
///
RemainingCount MipParser_parse(struct MipParsingState* parser, const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp, unsigned int maxPackets)
{
    // Reset the state if the timeout time has elapsed.
    if( parser->expectedLength != MIPPARSER_RESET_LENGTH && (timestamp - parser->startTime) > parser->timeout )
    {
        if( ByteRing_count(&parser->ring) > 0 )
            ByteRing_pop(&parser->ring, 1);
        parser->expectedLength = MIPPARSER_RESET_LENGTH;
    }

    unsigned int numPackets = 0;
    do
    {
        // Copy as much data as will fit in the ring buffer.
        ByteRing_copyFromAndUpdate(&parser->ring, &inputBuffer, &inputCount);

        struct MipPacket packet;
        while( MipParser_parseOnePacketFromRing(parser, &packet, timestamp) )
        {
            numPackets++;
            bool stop = (maxPackets > 0) && (numPackets >= maxPackets);

            if( parser->callback )
                stop |= !parser->callback(parser->callbackObject, &packet, parser->startTime);

            if( stop )
            {
                // Pull more data from the input buffer if possible.
                ByteRing_copyFromAndUpdate(&parser->ring, &inputBuffer, &inputCount);

                return -inputCount;
            }
        }

        // Need more data to continue parsing.
        // This code assumes the ring buffer is large enough for any single
        // received mip packet, otherwise it will get stuck in an infinite loop.

    } while( inputCount );

    return -inputCount;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Parses a single packet from the internal buffer.
///
///@internal
///
///@param parser
///@param packet_out
///       The mip packet to initialize with a valid packet, if found.
///@param timestamp
///       Time of the most recently received data.
///
///@returns true if a packet was found, false if more data is required. If false,
///         the packet is not initialized.
///
bool MipParser_parseOnePacketFromRing(struct MipParsingState* parser, struct MipPacket* packet_out, Timestamp timestamp)
{
    // Parse packets while there is sufficient data in the ring buffer.
    while( ByteRing_count(&parser->ring) >= parser->expectedLength )
    {
        if( parser->expectedLength == MIPPARSER_RESET_LENGTH )
        {
            if( ByteRing_at(&parser->ring, MIP_INDEX_SYNC1) != MIP_SYNC1 )
                ByteRing_pop(&parser->ring, 1);
            else
            {
                // Synchronized - set the start time and expect more data.
                parser->startTime = timestamp;
                parser->expectedLength = MIP_HEADER_LENGTH;
            }
        }
        else if( parser->expectedLength == MIP_HEADER_LENGTH )
        {
            // Check the sync bytes and drop a single byte if not sync'd.
            if( ByteRing_at(&parser->ring, MIP_INDEX_SYNC2) != MIP_SYNC2 )
            {
                ByteRing_pop(&parser->ring, 1);
                parser->expectedLength = MIPPARSER_RESET_LENGTH;
            }
            else
            {
                // Read the payload length and add it and the checksum size to the complete packet size.
                parser->expectedLength += ByteRing_at(&parser->ring, MIP_INDEX_LENGTH) + MIP_CHECKSUM_LENGTH;
            }
        }
        else // Just waiting on enough data
        {
            uint_least16_t packetLength = parser->expectedLength;
            parser->expectedLength = MIPPARSER_RESET_LENGTH;  // Reset parsing state

            ByteRing_copyTo(&parser->ring, parser->resultBuffer, packetLength);

            MipPacket_fromBuffer(packet_out, parser->resultBuffer, packetLength);

            if( !MipPacket_isValid(packet_out) )
            {
                // Invalid packet, drop just the first sync byte and restart.
                ByteRing_pop(&parser->ring, 1);
            }
            else // Checksum is valid
            {
                // Discard the packet bytes from the ring buffer since a copy was made.
                ByteRing_pop(&parser->ring, packetLength);

                // Successfully parsed a packet.
                return true;
            }
        }
    }

    // Need more data to continue.

    return false;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the packet timeout of the parser.
///
///
Timestamp MipParser_timeout(const struct MipParsingState* parser)
{
    return parser->timeout;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Changes the timeout of the MIP parser.
///
///
void MipParser_setTimeout(struct MipParsingState* parser, Timestamp timeout)
{
    parser->timeout = timeout;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Gets the timestamp of the last parsed packet.
///
/// This is only valid after a valid packet has been parsed.
///
/// This function is provided to allow additional calls to MipParser_parse()
/// with no input data (buffer=NULL and length=0) when maxPackets > 0. The
/// additional calls can use the same timestamp because no new data will be
/// processed.
///
/// There are two possible situations after the last call to parse:
/// 1. Either maxPackets was reached, meaning at least one packet was parsed, and
///    thus the timestamp is valid, or
/// 2. More data is required, in which case this time may not be valid, but it
///    won't matter because an additional call to parse won't produce a new
///    packet to be timestamped.
///
Timestamp MipParser_lastPacketTimestamp(const struct MipParsingState* parser)
{
    return parser->startTime;
}
