
#include "mip_parser.h"

#include "mip_offsets.h"

#include <assert.h>
#include <string.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif


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
void mip_parser_init(mip_parser* parser, mip_packet_callback callback, void* callback_object, mip_timeout timeout)
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
///@brief Internal function which scans for the next possible packet.
///
/// Looks for SYNC1 followed by SYNC2 and returns what it finds.
///
/// This function uses memchr to search which should be maximally efficient on
/// most platforms.
///
///@internal
///
///@param buffer
///       Buffer to search.
///@param buffer_len
///       Length of buffer and maximum length to search.
///@param[in,out] offset_ptr
///       As an input, specifies that the search should start at this index.
///       As an output, returns the position of the next potential packet.
///       If no packet is found, this will be equal to buffer_len.
///
///@returns The offset of the next byte to be parsed in the packet. This will be
///         1 if no packet is found, 2 if a lone SYNC1 (0x75) is found at the
///         end of the buffer, or 4 if consecutive SYNC1,SYNC2 (0x75,0x65) bytes
///         are found.
///
static size_t mip_find_sop(const uint8_t* buffer, size_t buffer_len, size_t* offset_ptr)
{
    assert(buffer != NULL);
    assert(offset_ptr != NULL);
    assert(*offset_ptr <= buffer_len);

    const uint8_t* ptr = buffer + *offset_ptr;
    size_t offset = *offset_ptr;
    for(;;)
    {
        ptr = memchr(ptr, MIP_SYNC1, buffer_len - offset);

        if(!ptr)
        {
            *offset_ptr = buffer_len;
            return 1;
        }

        offset = ptr - buffer;
        *offset_ptr = offset;

        // 0x75 right at end of buffer?
        if(offset+1 == buffer_len)
            return 2;

        if((offset+MIP_INDEX_SYNC2 < buffer_len) && buffer[offset+MIP_INDEX_SYNC2] == MIP_SYNC2)
            return MIP_HEADER_LENGTH;

        ++ptr;
        ++offset;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Discards data from the internal buffer.
///
/// This is an internal function called during parsing when an invalid packet
/// is detected. At least 'offset' bytes will be discarded, after which the
/// remaining data will be scanned for the next possible MIP packet. Data is
/// discarded up to this point.
///
///@internal
///
///@param parser
///@param offset Discard at least this many bytes.
///
///@returns The expected number of bytes to be parsed next. This can be 1, 2, or
///         4 bytes depending on a few factors. See mip_find_sop.
///
static size_t mip_parser_discard(mip_parser* parser, size_t offset)
{
    assert(offset <= parser->_buffered_length);

    // Search for start of a new packet.
    size_t expected_packet_length = mip_find_sop(parser->_buffer, parser->_buffered_length, &offset);

    // Shift buffered packet and any trailing data down to offset 0.
    // This makes parsing logic simpler and allows room for a full length packet.
    memmove(&parser->_buffer[0], &parser->_buffer[offset], parser->_buffered_length-offset);
    parser->_buffered_length -= (uint16_t)offset;

    return expected_packet_length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Parse packets from a buffer.
///
/// The buffer may contain non-mip data (e.g. NMEA 0183) which will be ignored.
///
///@param parser
///
///@param input_buffer
///       Buffer from which to parse packets. If NULL, parses from the internal
///       buffer instead (see mip_parser_get_write_ptr).
///@param input_length
///       Length of data in the buffer.
///@param timestamp
///       Time of arrival of the data to be parsed. This is used to set packets'
///       timestamp and to time out incomplete packets.
///
///@note The timestamp of a packet is based on the time the packet was parsed.
///      Packets received during an earlier parse call may be timestamped with
///      the time from a later parse call, but will never be timestamped before
///      they were actually received.
///
///@note The parser will do its best to ignore non-MIP data. However, it is
///      possible for some binary data to appear to be a MIP packet if it
///      contains the two-byte sequence 0x75, 0x65. This may cause temporary
///      stalls in parsed data if the following bytes suggest that more data is
///      needed to complete the "packet". Once the fake packet times out or
///      enough data is received, real MIP packets received in the meantime will
///      be properly parsed. Note that the 16-bit checksum has a 1 in 65,536
///      chance of appearing to be valid at random.
///
void mip_parser_parse(mip_parser* parser, const uint8_t* input_buffer, size_t input_length, mip_timestamp timestamp)
{
    // Allow the user to specify bytes written into the parser buffer itself
    // via mip_parser_get_write_ptr().
    if(input_buffer == NULL)
    {
        parser->_buffered_length += (uint16_t)input_length;
        input_length = 0;

        // Check that the buffer has not been overrun (and likely corrupting the mip interface).
        assert(parser->_buffered_length <= sizeof(parser->_buffer));
    }

    // Offset into the input buffer (nonzero if bytes are dropped/skipped due to garbage/non-mip data)
    size_t unparsed_input_offset = 0;

    // Bytes already in the parser buffer from previous call.
    //packet_length buffered_length = parser->_buffered_length;
    assert(parser->_buffered_length <= MIP_PACKET_LENGTH_MAX);

    // Expected length of the packet or current header byte being parsed. 1, 2, 4 or >= 6.
    size_t expected_packet_length =
               (parser->_buffered_length < MIP_HEADER_LENGTH) ?
               (parser->_buffered_length + 1) :
               (MIP_HEADER_LENGTH + parser->_buffer[MIP_INDEX_LENGTH] + MIP_CHECKSUM_LENGTH)
    ;

    // size_t total_packet_bytes = 0;

    for(;;)
    {
        // Input offset must not exceed input length.
        assert(unparsed_input_offset <= input_length);
        // Remaining processed bytes from the input buffer
        const size_t remaining_input_length = input_length - unparsed_input_offset;

        const bool reparsing = parser->_buffered_length >= expected_packet_length;

        // Check if there is enough input data to finish parsing a packet.
        if(!reparsing && remaining_input_length < (expected_packet_length - parser->_buffered_length))
        {
            // Not enough data for a/the packet.

            // Check for timeout
            if(timestamp >= (parser->_start_time + parser->_timeout))
            {
                // Discard first packet in buffer and reparse remaining buffered data.
                if(parser->_buffered_length > 0)
                    expected_packet_length = mip_parser_discard(parser, 1);

                parser->_start_time = timestamp;
                continue;
            }

            memcpy(&parser->_buffer[parser->_buffered_length], &input_buffer[unparsed_input_offset], remaining_input_length);

            parser->_buffered_length += (uint16_t)remaining_input_length;
            unparsed_input_offset    += remaining_input_length;
            //remaining_packet_length  -= remaining_input_length;
            //remaining_input_length   = 0;

            assert(unparsed_input_offset == input_length);
            assert(parser->_buffered_length <= MIP_PACKET_LENGTH_MAX);
            //assert(total_packet_bytes <= input_length);

            MIP_DIAG_INC(parser->_diag_bytes_read, input_length);
            //MIP_DIAG_INC(parser->_diag_bytes_skipped, (input_length - total_packet_bytes));

            return;
        }

        //
        // The actual parsing logic
        //
        switch(expected_packet_length)
        {
            // Nothing parsed yet (expecting 1 sync byte)
        case MIP_INDEX_SYNC1+1:
            assert(!reparsing);
            assert(parser->_buffered_length == 0);
            // Optimization: use memchr to find the SOP, it's way faster than iterating this loop one byte at a time.
            // Note: this also tries to find SYNC2 if SYNC1 is not at the end of the buffer.
            expected_packet_length = mip_find_sop(input_buffer, input_length, &unparsed_input_offset);
            //remaining_input_length = input_length - unparsed_input_offset;

            // Reset start time (OK even if no packet was found).
            parser->_start_time = timestamp;

            break;

            // Got single byte of 0x75 in the parse buffer.
        case MIP_INDEX_SYNC2+1:
            // mip_find_sop() always tries to find both 0x75 and 0x65, so this
            // case only happens when 0x75 is left at the end of the parse buffer.
            assert(!reparsing);

            if(input_buffer[unparsed_input_offset + (MIP_INDEX_SYNC2 - parser->_buffered_length)] == MIP_SYNC2)
                expected_packet_length = MIP_HEADER_LENGTH;
            else  // Next byte is not 0x65 --> not a mip packet, reset parser
            {
                // Don't advance the input offset if the SYNC1 was in the packet buffer.
                if(parser->_buffered_length != 0)
                    parser->_buffered_length = 0;
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
            if(reparsing)
            {
                assert(parser->_buffered_length >= MIP_HEADER_LENGTH);
                expected_packet_length = parser->_buffer[MIP_INDEX_LENGTH];
            }
            else // parser->_buffered_length >= MIP_HEADER_LENGTH
            {
                assert(parser->_buffered_length < MIP_HEADER_LENGTH);
                expected_packet_length = input_buffer[unparsed_input_offset + MIP_INDEX_LENGTH - parser->_buffered_length];
            }

            expected_packet_length += MIP_HEADER_LENGTH + MIP_CHECKSUM_LENGTH;
            break;

        default:  // All packet data is available, check checksum
        {
            if(!reparsing)
            {
                const size_t packet_length_from_input = expected_packet_length - parser->_buffered_length;
                assert(packet_length_from_input <= input_length-unparsed_input_offset);  // Check math

                memcpy(&parser->_buffer[parser->_buffered_length], &input_buffer[unparsed_input_offset], packet_length_from_input);
                unparsed_input_offset    += packet_length_from_input;
                parser->_buffered_length += (uint16_t)packet_length_from_input;
            }

            assert(expected_packet_length <= MIP_PACKET_LENGTH_MAX && expected_packet_length >= MIP_PACKET_LENGTH_MIN);

            mip_packet_view packet = {
                ._buffer        = parser->_buffer,
                ._buffer_length = (uint_least16_t)expected_packet_length,
            };

            const bool checksum_valid = mip_packet_compute_checksum(&packet) == mip_packet_checksum_value(&packet);

            // Note: if the checksum wasn't valid, need to reparse the parse_buffer since
            // there may be valid packets nested within the bad "packet".

            if(checksum_valid)
            {
                // total_packet_bytes += expected_packet_length;
                MIP_DIAG_INC(parser->_diag_valid_packets, 1);
                MIP_DIAG_INC(parser->_diag_packet_bytes, expected_packet_length);

                if(parser->_callback)
                    parser->_callback(parser->_callback_object, &packet, parser->_start_time);
            }
            else
            {
                MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
                expected_packet_length = 1;  // discard at least one byte in mip_parser_discard (below).
            }

            // Shift any leftover data down to 0 (multiple packets within a buffered false packet).
            expected_packet_length = mip_parser_discard(parser, expected_packet_length);
            parser->_start_time = timestamp;
        }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Processes all previously buffered data.
///
/// Call this at the end of reading a binary file to ensure that any trailing
/// packets are fully processed.
///
///@param parser
///
void mip_parser_flush(mip_parser* parser)
{
    while(parser->_buffered_length > 0)
    {
        mip_parser_parse(parser, NULL, 0, parser->_start_time);

        if(mip_parser_discard(parser, 1) == 1)
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets a pointer into which a small amount of data may be written for
///       parsing.
///
/// Generally you should call mip_parser_parse with your input buffer directly.
/// This method may be more efficient for data which arrives in small chunks
/// by avoiding an intermediate buffer. However, for large chunks of data (e.g.
/// reading from a file) it's more efficient to declare a bigger buffer (at
/// least 1024 bytes) and parse that instead.
///
///@caution If you use this function, you must call mip_parser_parse with
///         input_buffer=NULL and input_length equal to the number of bytes
///         written into the parser. Otherwise the data will be lost.
///
///@param parser
///
///@param[out] ptr_out
///       Pointer to a pointer which will be set to an internal buffer location.
///
///@returns The maximum number of bytes which may be written to the parser.
///         This will never be more than the maximum MIP packet size.
///
uint_least16_t mip_parser_get_write_ptr(mip_parser* parser, uint8_t** ptr_out)
{
    assert(ptr_out);  // Can't be NULL.

    *ptr_out = &parser->_buffer[parser->_buffered_length];
    return sizeof(parser->_buffer) - parser->_buffered_length;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the packet timeout of the parser.
///
///
mip_timeout mip_parser_timeout(const mip_parser* parser)
{
    return parser->_timeout;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Changes the timeout of the MIP parser.
///
///@param parser
///@param timeout
///
void mip_parser_set_timeout(mip_parser* parser, mip_timeout timeout)
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
mip_timestamp mip_parser_current_timestamp(const mip_parser* parser)
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
mip_timeout mip_timeout_from_baudrate(uint32_t baudrate)
{
    // num_symbols [b] = (packet_length [B]) * (10 [b/B])
    unsigned int num_symbols = MIP_PACKET_LENGTH_MAX * 10;

    // packet_time [s] = (num_symbols [b]) / (baudrate [b/s])

    // timeout [ms] = (packet_time [s]) * (1000 [ms/s]) * tolerance
    return num_symbols * 1500 / baudrate;
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif
