
#include "mip_parser.h"

#include "mip_offsets.h"

#include <assert.h>
#include <string.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MIP parser.
///
///
///@param parser
///@param callback
///       A function to be called when a valid packet is identified. It will be
///       passed an optional user-supplied parameter, a pointer to the packet,
///       and the time the first byte was parsed.
///@param callback_object
///       An optional user-specified pointer which is directly passed to
///       the callback as the first parameter.
///@param timeout
///       The timeout for receiving one packet. Depends on the serial baud rate
///       and is typically 100 milliseconds.
///
void mip_parser_init(mip_parser* parser, mip_packet_callback callback, void* callback_object, timestamp_type timeout)
{
    parser->_timeout = timeout;

    mip_parser_reset(parser);

    parser->_callback = callback;
    parser->_callback_object = callback_object;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Resets the MIP parser.
///
/// Clears the current packet and internal buffer. The parser will be restored
/// as if mip_parser_init had just been called.
///
///
///@param parser
///
void mip_parser_reset(mip_parser* parser)
{
    parser->_start_time      = 0;
    parser->_buffered_length = 0;

    MIP_DIAG_ZERO(parser->_diag_bytes_read);
    MIP_DIAG_ZERO(parser->_diag_bytes_skipped);
    MIP_DIAG_ZERO(parser->_diag_packet_bytes);
    MIP_DIAG_ZERO(parser->_diag_valid_packets);
    MIP_DIAG_ZERO(parser->_diag_invalid_packets);
    MIP_DIAG_ZERO(parser->_diag_timeouts);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Parses at most one packet from the input buffer.
///
///@note This is an advanced function which may require additional code to be
///      useful. In particular it does not provide timeouts nor diagnostic
///      information. However, it forms the core mip parsing logic and could
///      be useful (e.g. when parsing binary mip files). This is the only
///      method that can show you packets which fail the checksum.
///
///@param packet_buffer
///       An optional buffer to store partial and complete mip packets.
///       If this is specified, packet bytes will be copied to this buffer.
///       It can be used to hold partial packets between calls so that the
///       input buffer may be fully processed on each call (see note).
///       If NULL, packet data will be left in the input_buffer and the user
///       must ensure the input buffer remains valid as long as the packet
///       data is referenced.
///       If *packet_length_ptr > 0 and packet_buffer[0] != MIP_SYNC1, then
///       this signals that the packet timed out or failed the checksum.
///       In this case the packet_buffer is reparsed in order to check for
///       valid packets contained within the buffer but masked by the invalid
///       packet.
///@param[in,out] packet_length_out
///       As an input, this must point to the number of "leftover" bytes in
///       packet_buffer from a previous call to this function. If packet_buffer
///       is NULL (or if there's no leftover data), the initial value must be 0.
///       As an output, if a packet is parsed the pointed-to value will be set
///       to the number of bytes in the packet. Additionally, it will be updated
///       with the number of "leftover" bytes that form an incomplete packet.
///       (If packet_buffer is not NULL, these bytes are copied into it).
///       This pointer cannot be NULL.
///@param input_buffer
///       Buffer containing data to be parsed.
///@param input_length
///       Number of bytes to parse from input_buffer.
///@param[in,out] consumed_input_length_out
///       If not NULL, the pointed-to value will be set to the number of bytes
///       in input_buffer that were consumed. The user should discard this much
///       data from the input buffer. This will include all bytes up to the
///       first byte of a (potential) packet. If packet_buffer is not NULL,
///       it includes any complete or partial packet as well.
///       Note that if packet_buffer is supplied (not NULL) and no packet is
///       found, then all of the input data will be processed and this value
///       will match input_length.
///
///@returns Positive if a valid packet was parsed.
///@returns Negative if an invalid packet was detected (bad checksum)
///@returns 0 if more data is required (incomplete or no packet detected).
///
int mip_parse_one_packet(uint8_t* packet_buffer, packet_length* leftover_length_ptr, const uint8_t* input_buffer, size_t input_length, size_t* consumed_input_length_out, packet_length* packet_length_out)
{
    assert(!leftover_length_ptr || *leftover_length_ptr == 0 || packet_buffer);  // Buffer can't be NULL if leftover data is specified.
    assert(packet_length_out);  // Must specify packet length pointer.
    assert(input_buffer || input_length > 0);  // Input buffer can be NULL IFF input_length == 0.

    if(consumed_input_length_out)
        *consumed_input_length_out = 0;

    *packet_length_out = 0;

    // Offset into the input buffer (nonzero if bytes are dropped/skipped due to garbage/non-mip data)
    size_t unparsed_input_offset = 0;

    // Bytes already in the parser buffer from previous call.
    packet_length leftover_length = leftover_length_ptr ? *leftover_length_ptr : 0;

    assert(leftover_length < MIP_PACKET_LENGTH_MAX);

    // Expected length of the packet or current header byte being parsed. 1, 2, 4 or >= 6.
    size_t expected_packet_length = (leftover_length < MIP_HEADER_LENGTH) ? (leftover_length + 1) : (MIP_HEADER_LENGTH + packet_buffer[MIP_INDEX_LENGTH] + MIP_CHECKSUM_LENGTH);

    // Number of bytes not yet parsed from the input buffer.
    size_t remaining_input_length;  // Always set in for loop

    for(;;)
    {
        // Input offset must not exceed input length.
        assert(unparsed_input_offset <= input_length);
        // Remaining processed bytes from the input buffer
        remaining_input_length = input_length - unparsed_input_offset;

        // Only input_buffer is processed in this function, so complete packet(s)
        // sitting in packet_buffer won't be noticed.
        assert(expected_packet_length > leftover_length);
        // Number of bytes remaining before the packet will be complete.
        size_t remaining_packet_length = expected_packet_length - leftover_length;

        // Check if there is enough input data to finish parsing a packet.
        if(remaining_input_length < remaining_packet_length)
        {
            // Not enough data for a/the packet.

            // If a separate packet buffer is used, copy remaining data to it
            // so that the entire input buffer is consumed. Note that this
            // cannot overflow since by definition it's not enough for a max size packet.
            if(packet_buffer != NULL)
            {
                // The total number of leftover bytes should be less than the packet size.
                assert(remaining_input_length + leftover_length < expected_packet_length);

                memcpy(&packet_buffer[leftover_length], &input_buffer[unparsed_input_offset], remaining_input_length);

                leftover_length += remaining_input_length;
                remaining_packet_length -= remaining_input_length;
                assert(remaining_packet_length > 0);

                unparsed_input_offset += remaining_input_length;
                remaining_input_length = 0;

                assert(leftover_length <= MIP_PACKET_LENGTH_MAX);
                assert(leftover_length_ptr != NULL);
                *leftover_length_ptr = leftover_length;
            }
            else
            {
                assert(!leftover_length_ptr || *leftover_length_ptr == 0);
            }

            if (consumed_input_length_out != NULL)
                *consumed_input_length_out = unparsed_input_offset;

            *packet_length_out = 0;

            return 0;  // Not enough data
        }

        //
        // The actual parsing logic
        //
        switch(expected_packet_length)
        {
        // Nothing parsed yet (expecting 1 sync byte)
        case MIP_INDEX_SYNC1+1:
            assert(leftover_length == 0);
            if(input_buffer[unparsed_input_offset + MIP_INDEX_SYNC1] == MIP_SYNC1)
                expected_packet_length++;
            else  // Not SYNC1
                unparsed_input_offset++;
            break;

        // Got single byte of 0x75 either in the input buffer or in the parser buffer (expecting 2 bytes so far).
        case MIP_INDEX_SYNC2+1:
            assert(leftover_length <= MIP_INDEX_SYNC2);  // SYNC1 may or may not be in the parser buffer.
            if(input_buffer[unparsed_input_offset + (MIP_INDEX_SYNC2 - leftover_length)] == MIP_SYNC2)
                expected_packet_length = MIP_HEADER_LENGTH;
            else  // Next byte is not 0x65 --> not a mip packet, reset parser
            {
                // Don't advance the input offset if the SYNC1 was in the packet buffer.
                if(leftover_length > 0)
                    leftover_length = 0;
                else
                    unparsed_input_offset++;

                expected_packet_length = 1;
            }
            break;

        // Nothing special to do for the descriptor set.
        // This case can only happen when leftover_length == 2.
        case MIP_INDEX_DESCSET+1:
            expected_packet_length = 4;
            break;

        // Got the expected 4 bytes for the header - read packet's length field.
        case MIP_HEADER_LENGTH:
            assert(leftover_length < MIP_HEADER_LENGTH);  // Packet length shouldn't be in the parser buffer yet.
            expected_packet_length = MIP_HEADER_LENGTH + input_buffer[unparsed_input_offset + MIP_INDEX_LENGTH - leftover_length] + MIP_CHECKSUM_LENGTH;
            break;

        default:  // All packet data is available, check checksum
        {
            assert(expected_packet_length >= leftover_length);  // Overflow check
            const size_t packet_length_from_input = expected_packet_length - leftover_length;
            assert(packet_length_from_input <= input_length-unparsed_input_offset);  // Check math

            if (packet_buffer)
            {
                memcpy(&packet_buffer[leftover_length], &input_buffer[unparsed_input_offset], packet_length_from_input);
                unparsed_input_offset += packet_length_from_input;
            }

            assert(expected_packet_length <= MIP_PACKET_LENGTH_MAX && expected_packet_length >= MIP_PACKET_LENGTH_MIN);
            *packet_length_out = (packet_length)expected_packet_length;

            mip_packet packet = {
                ._buffer        = packet_buffer ? packet_buffer : (uint8_t*)&input_buffer[unparsed_input_offset],
                ._buffer_length = *packet_length_out
            };

            const bool checksum_valid = mip_packet_compute_checksum(&packet) == mip_packet_checksum_value(&packet);

            if(checksum_valid)
                leftover_length = 0;
            else
                leftover_length = expected_packet_length;

            // Note: if the checksum wasn't valid, need to reparse the parse_buffer since
            // there may be valid packets nested within the bad "packet".

            if (consumed_input_length_out != NULL)
                *consumed_input_length_out = unparsed_input_offset;

            if(leftover_length_ptr != NULL)
                *leftover_length_ptr = leftover_length;

            return checksum_valid ? +1 : -1;
        }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Parses packets from the input data buffer.
///
/// For every valid MIP packet, the callback function will be called with the
/// packet and timestamp.
///
///
///@param parser
///@param input_buffer
///       Pointer to bytes received from the device or file. This buffer may
///       contain non-mip data (e.g. NMEA 0183), which will be ignored.
///       This buffer may be NULL if input_count is 0.
///@param input_length
///       The number of bytes in the input buffer.
///@param timestamp
///       The local time the data was received. This is used to check for
///       timeouts and is passed to the callback as the packet time of arrival.
///@param max_packets
///       The maximum number of packets to process. Unprocessed data is left in
///       the input buffer. If MIP_PARSER_UNLIMITED_PACKETS (0), the entire
///       input buffer will be processed.
///
///@returns The number of bytes processed/consumed from the input buffer.
///         If max_packets is MIP_PARSER_UNLIMITED_PACKETS (0), this will also
///         be zero as all of the data will be consumed. Incomplete packets are
///         be held in an internal buffer.
///
///@note If max_packets is 0, then this function is guaranteed to consume all
///      of the input data and the buffer can be reused or discarded afterward.
///      However, if max_packets is nonzero (meaning the number of packets parsed
///      will be limited), then this is no longer true and the calling code is
///      responsible for buffering the excess data.
///
///@note The timestamp of parsed packets is based on the time the packet was
///      parsed. When max_packets==0, this is the same as the input timestamp.
///      When max_packets!=0, packets received during an earlier parse call
///      may be timestamped with the time from a later parse call. Therefore,
///      if packet timestamping is critical to your application, avoid using
///      max_packets > 0.
///
///@note The parser will do its best to ignore non-MIP data. However, it is
///      possible for some binary data to appear to be a MIP packet if it
///      contains 0x75,0x65, has at least 6 bytes, and has a valid checksum. A
///      16-bit checksum has a 1 in 65,536 chance of appearing to be valid.
///
size_t mip_parser_parse(mip_parser* parser, const uint8_t* input_buffer, size_t input_length, timestamp_type timestamp, unsigned int max_packets)
{
    bool reparse = false;

    // Check for timeout
    if((parser->_buffered_length > 0) && (timestamp >= parser->_start_time + parser->_timeout))
    {
        MIP_DIAG_INC(parser->_diag_timeouts, 1);

        // If a packet times out, it could be due to random garbage creating a fake mip header with
        // a large packet size. There may be real packets within the fake packet and these
        // should be parsed normally, since they technically may not have timed out.
        // If any partial packet remains after reparsing, discard it.
        //
        // E.g. if leftover data in parser buffer looks like:
        //   75 65 xx FF | xx ... 75 65 01 | 02 02 01 E0 | C6 zz zz | zz
        //   where '|' separates data being processed in chunks, then:
        // * The outer "packet" (maybe just garbage that looks like a packet) has timed out.
        // * The inner ping command has NOT timed out because it was probably received
        //   in a narrow span of time. Either way, it's been properly received so accept it.
        // * The data at the end ('zz') shouldn't time out either, as it could be relatively
        //   recent.
        //
        // The timeout is a safeguard against really long bogus packets clogging the parser
        // indefinitely until more data arrives. It's ok to have false negative timeouts.

        reparse = true;
    }
    // Set parser start time if not continuing a previous packet.
    else if(parser->_buffered_length == 0)
        parser->_start_time = timestamp;

    size_t total_consumed = 0;

    mip_packet packet = {
        ._buffer = parser->_buffer,
        ._buffer_length = 0,
    };

    for(unsigned int packet_num = 0; (max_packets==MIP_PARSER_UNLIMITED_PACKETS) || (packet_num < max_packets) || reparse; packet_num++)
    {
        size_t consumed_length;
        int    result;
        bool   reparsed = reparse;

        if(reparse)
        {
            if(parser->_buffered_length == 0)
            {
                reparse = false;
                continue;
            }

            parser->_start_time = timestamp;

            // Reparse the packet buffer, skipping the first byte.
            //
            // |   |start             |buffered
            // |75 XX XX XX XX XX ... |

            result = mip_parse_one_packet(NULL, NULL, parser->_buffer+1, parser->_buffered_length-1, &consumed_length, &packet._buffer_length);
            ++consumed_length;  // Include first skipped byte

            if(result > 0)  // valid packet
            {
                // When using NULL parse buffer to mip_parse_one_packet, the packet buffer is in the "input" buffer.
                packet._buffer = &parser->_buffer[consumed_length];

                // Mark packet as consumed now that it's been processed.
                //
                // |         |consumed                |buffered
                // |75 XX XX 75 65 YY ... XX XX XX XX |
                // |         |packet  ... |           |buffered
                consumed_length += packet._buffer_length;
                // |                      |consumed   |buffered
                // |75 XX XX 75 65 YY ... XX XX XX XX |
            }

            assert(parser->_buffered_length >= consumed_length);
        }
        else
        {
            if(input_length == 0)
                break;

            result = mip_parse_one_packet(parser->_buffer, &parser->_buffered_length, input_buffer, input_length, &consumed_length, &packet._buffer_length);

            input_buffer   += consumed_length;
            input_length   -= consumed_length;
            total_consumed += consumed_length;

            MIP_DIAG_INC(parser->_diag_bytes_read, consumed_length);
        }

        if (result > 0)  // Valid packet
        {
            MIP_DIAG_INC(parser->_diag_valid_packets, 1);
            MIP_DIAG_INC(parser->_diag_packet_bytes, packet._buffer_length);
            if(consumed_length > packet._buffer_length)
            {
                // Some bytes were skipped if more than the packet length was consumed.
                MIP_DIAG_INC(parser->_diag_bytes_skipped, (consumed_length - packet._buffer_length));
            }

            if(parser->_callback)
                parser->_callback(parser->_callback_object, &packet, parser->_start_time);

            parser->_start_time = timestamp;

        }
        else if(result < 0)  // Invalid packet
        {
            MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
            MIP_DIAG_INC(parser->_diag_bytes_skipped, consumed_length);

            // Go back and check for "nested" packets.
            reparse = true;
        }
        else if(reparse)  // Need more data (done reparsing packet_buffer)
        {
            reparse = false;
        }
        else  // No more data in input_buffer.
        {
            assert(input_length == 0);
            break;
        }

        // Consume data in parser buffer if this was a reparse cycle.
        if(reparsed)
        {
            // Get rid of any junk and make room for a full packet.
            parser->_buffered_length -= consumed_length;
            memmove(parser->_buffer, &parser->_buffer[consumed_length], parser->_buffered_length);
        }
    }

    return total_consumed;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the packet timeout of the parser.
///
///
timestamp_type mip_parser_timeout(const mip_parser* parser)
{
    return parser->_timeout;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Changes the timeout of the MIP parser.
///
///@param parser
///@param timeout
///
void mip_parser_set_timeout(mip_parser* parser, timestamp_type timeout)
{
    parser->_timeout = timeout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief mip_parser_set_callback
///
///@param parser
///@param callback
///@param callback_object
///
void mip_parser_set_callback(mip_parser* parser, mip_packet_callback callback, void* callback_object)
{
    parser->_callback_object = callback_object;
    parser->_callback        = callback;
}

////////////////////////////////////////////////////////////////////////////////
///@brief mip_parser_callback
///
///@param parser
///
///@returns the packet callback function.
///
mip_packet_callback mip_parser_callback(const mip_parser* parser)
{
    return parser->_callback;
}


////////////////////////////////////////////////////////////////////////////////
///@brief mip_parser_callback
///
///@param parser
///
///@returns the packet callback user data pointer.
///
void* mip_parser_callback_object(const mip_parser* parser)
{
    return parser->_callback_object;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Gets the timestamp of the last parsed packet.
///
/// This is only valid after a valid packet has been parsed.
///
/// This function is provided to allow additional calls to mip_parser_parse()
/// with no input data (buffer=NULL and length=0) when max_packets > 0. The
/// additional calls can use the same timestamp because no new data will be
/// processed.
///
/// There are two possible situations after the last call to parse:
/// 1. Either max_packets was reached, meaning at least one packet was parsed, and
///    thus the timestamp is valid, or
/// 2. More data is required, in which case this time may not be valid, but it
///    won't matter because an additional call to parse won't produce a new
///    packet to be timestamped.
///
timestamp_type mip_parser_last_packet_timestamp(const mip_parser* parser)
{
    return parser->_start_time;
}


#ifdef MIP_ENABLE_DIAGNOSTICS

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of bytes read from the user input buffer.
///
/// This includes data read into the internal ring buffer but not yet seen by
/// the parser. Ensure all packets have been processed by the parser before
/// comparing against the packet_bytes counter.
///
uint32_t mip_parser_diagnostic_bytes_read(const mip_parser* parser)
{
    return parser->_diag_bytes_read;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets total the number of bytes that have been parsed into valid
///       packets.
///
/// This is a summation of the total length of every valid mip packet emitted by
/// the parser.
///
uint32_t mip_parser_diagnostic_packet_bytes(const mip_parser* parser)
{
    return parser->_diag_packet_bytes;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of bytes which weren't part of a valid packet.
///
/// This is the difference between the "packet bytes" and "bytes read" counters.
///
uint32_t mip_parser_diagnostic_bytes_skipped(const mip_parser* parser)
{
    return parser->_diag_bytes_skipped;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of valid packets emitted by the parser.
///
uint32_t mip_parser_diagnostic_valid_packets(const mip_parser* parser)
{
    return parser->_diag_valid_packets;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of packets that failed the checksum check.
///
/// These invalid packets are not emitted by the parser and are not included in
/// the "valid packets" or "packet bytes" counters.
///
uint32_t mip_parser_diagnostic_invalid_packets(const mip_parser* parser)
{
    return parser->_diag_invalid_packets;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of times a packet timed out waiting for more
///       data.
///
/// Packets may time out under the following conditions:
///@li The connection is interrupted
///@li The length byte is corrupted to make the packet look longer
///@li The connection bandwidth and/or latency is too low
///
uint32_t mip_parser_diagnostic_timeouts(const mip_parser* parser)
{
    return parser->_diag_timeouts;
}

#endif // MIP_ENABLE_DIAGNOSTICS


////////////////////////////////////////////////////////////////////////////////
///@brief Computes an appropriate packet timeout for a given serial baud rate.
///
///@note This function assumes a standard serial port with 10 symbols per byte:
/// 1 start bit, 8 data bits, and 1 stop bit.
///
///@param baudrate Serial baud rate in bits per second
///
///@return A timeout value in ms representing the time it would take to transmit
///        a single mip packet of maximum size at the given baud rate, plus some
///        tolerance.
///
timeout_type mip_timeout_from_baudrate(uint32_t baudrate)
{
    // num_symbols [b] = (packet_length [B]) * (10 [b/B])
    unsigned int num_symbols = MIP_PACKET_LENGTH_MAX * 10;

    // packet_time [s] = (num_symbols [b]) / (baudrate [b/s])

    // timeout [ms] = (packet_time [s]) * (1000 [ms/s]) * tolerance
    return num_symbols * 1500 / baudrate;
}
