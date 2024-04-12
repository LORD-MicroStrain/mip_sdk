
#include "mip_parser.h"

#include "mip_offsets.h"

#include <assert.h>
#include <string.h>


#define MIPPARSER_RESET_LENGTH 1

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
///@param[in,out] packet
///       A pointer to a MIP packet. If the buffer is NULL (and size is 0), the
///       buffer and length will be updated to point into the input buffer when
///       a mip packet is parsed out. Otherwise, it is assumed that
///       the packet points to a buffer where the packet data should be copied.
///@param input_buffer
///       Buffer containing data to be parsed.
///@param input_length
///       Number of bytes to parse from input_buffer.
///@param[in,out] leftover_length_ptr
///       If not NULL, this value identifies if there is leftover data in the
///       packet buffer that should be included when parsing the data. Normally
///       you'd set this to 0. This function will set it to the number of bytes
///       that remain to be parsed from the input.
///
///@returns Positive if a valid packet was parsed.
///@returns Negative if an invalid packet was detected (bad checksum)
///@returns 0 if more data is required (no or incomplete packets detected).
///
int mip_parse_one_packet(uint8_t* packet_buffer, packet_length* packet_length_ptr, const uint8_t* input_buffer, size_t input_length, size_t* consumed_input_length_ptr)
{
    assert(packet_length_ptr != NULL);  // Must supply a size pointer.
    assert(*packet_length_ptr == 0 || packet_buffer != NULL);  // Buffer can't be NULL if leftover data is specified.

    // Should the packet data be copied to an existing buffer or should the packet buffer point at the input data?
    const bool existing_buffer    = packet_buffer != NULL;
    // Bytes already in the parser buffer from previous call.
    size_t leftover_length        = *packet_length_ptr;
    // Expected length of the packet or current header byte being parsed. 1, 2, 4 or >= 6.
    size_t expected_packet_length = (leftover_length < MIP_HEADER_LENGTH) ? (leftover_length + 1) : (MIP_HEADER_LENGTH + packet_buffer[MIP_INDEX_LENGTH] + MIP_CHECKSUM_LENGTH);
    // Offset into the input buffer (nonzero if bytes are dropped/skipped due to garbage/non-mip data)
    size_t unparsed_input_offset = 0;

    assert(leftover_length < MIP_PACKET_LENGTH_MAX);
    //assert(expected_packet_length > leftover_length);

    // Leftover data, but not a valid packet?
    // This happens when a bad checksum or timeout occurs.
    if(leftover_length > 0 && packet_buffer[0] != MIP_SYNC1)
    {
        // Search for a nested SYNC1 and shift the data over.
        // This simplifies the parsing code by ensuring the packet buffer offset is always 0
        // and that there is enough space for a max size packet. Theoretically this could
        // be done only if the buffer runs out of space but that would be more complex.
        size_t i;
        for(i=1; i<leftover_length; i++)
        {
            if(packet_buffer[i] == MIP_SYNC1)
            {
                memmove(packet_buffer, &packet_buffer[i], leftover_length-i);
                break;
            }
        }
        leftover_length -= i;
    }

    while((input_length - unparsed_input_offset) >= (expected_packet_length - leftover_length))
    {
        switch(expected_packet_length)
        {
            // Nothing parsed yet (expecting 1 sync byte)
            case MIP_INDEX_SYNC1+1:
                assert(leftover_length == 0);
                if(input_buffer[unparsed_input_offset + MIP_INDEX_SYNC1] == MIP_SYNC1)
                    expected_packet_length++;
                else
                    unparsed_input_offset++;
                break;

            // Got single byte of 0x75 either in the input buffer or in the parser buffer (expecting 2 bytes so far).
            case MIP_INDEX_SYNC2+1:
                assert(leftover_length <= MIP_INDEX_SYNC2);  // SYNC1 may or may not be in the parser buffer.
                if(input_buffer[unparsed_input_offset + (MIP_INDEX_SYNC2 - leftover_length)] == MIP_SYNC2)
                    expected_packet_length = MIP_HEADER_LENGTH;
                else  // Next byte is not 0x65 --> not a mip packet, reset parser
                {
                    unparsed_input_offset++;
                    expected_packet_length = 1;
                    leftover_length        = 0;
                }
                break;

            // Got the expected 4 bytes for the header - read packet's length field.
            case MIP_HEADER_LENGTH:
                assert(leftover_length < MIP_HEADER_LENGTH);  // Packet length shouldn't be in the parser buffer yet.
                expected_packet_length = MIP_HEADER_LENGTH + input_buffer[unparsed_input_offset + MIP_INDEX_LENGTH - leftover_length] + MIP_CHECKSUM_LENGTH;
                break;

            default:  // All packet data is available, check checksum
            {
                assert(expected_packet_length > leftover_length);  // At least one byte should have been parsed from input.
                const size_t packet_length_from_input = expected_packet_length - leftover_length;

                if (existing_buffer)
                    memcpy(&packet_buffer[leftover_length], &input_buffer[unparsed_input_offset], packet_length_from_input);
                //else
                //    packet_buffer = (uint8_t *) &input_buffer[unparsed_input_offset];

                assert(expected_packet_length <= MIP_PACKET_LENGTH_MAX && expected_packet_length >= MIP_PACKET_LENGTH_MIN);
                *packet_length_ptr = expected_packet_length;

                mip_packet packet = {
                    ._buffer        = packet_buffer,
                    ._buffer_length = *packet_length_ptr
                };

                const bool checksum_valid = mip_packet_compute_checksum(&packet) == mip_packet_checksum_value(&packet);

                // Whole valid packet was copied, mark entire input packet as consumed.
                if(existing_buffer && checksum_valid)
                    unparsed_input_offset += packet_length_from_input;

                // Note: if the checksum wasn't valid, need to reparse the parse_buffer since
                // there may be valid packets nested within the bad one.

                if (consumed_input_length_ptr != NULL)
                    *consumed_input_length_ptr = unparsed_input_offset;

                return checksum_valid ? +1 : -1;
            }
        }
    }

    if(existing_buffer)
    {
        // Copy the entire rest of the input buffer into the packet buffer as "leftovers" for the next parse call.
        const size_t remaining_unparsed_input_length = input_length - unparsed_input_offset;

        // The total number of leftover bytes should be less than the packet size, otherwise
        // it would have been parsed out above.
        assert(remaining_unparsed_input_length + leftover_length < expected_packet_length);

        memcpy(&packet_buffer[leftover_length], &input_buffer[unparsed_input_offset], remaining_unparsed_input_length);
        *packet_length_ptr = leftover_length + remaining_unparsed_input_length;
    }

    if(consumed_input_length_ptr != NULL)
        *consumed_input_length_ptr = unparsed_input_offset;

    return 0;

}
//
//size_t mip_parser_parse_one_packet(mip_parser* parser, const uint8_t* input_buffer, size_t input_length, timestamp_type timestamp, mip_packet* packet_out)
//{
//    // Check for timeout
//    if(timestamp >= parser->_start_time + parser->_timeout)
//    {
//        MIP_DIAG_INC(parser->_diag_timeouts, 1);
//        parser->_buffer_length = 0;
//    }
//
//    // Bytes already in the parser buffer from previous call.
//    size_t leftover_length        = parser->_buffer_length;
//    // Expected length of the packet or current header byte being parsed. 1, 2, 4 or >= 6.
//    size_t expected_packet_length = (leftover_length < MIP_HEADER_LENGTH) ? (leftover_length + 1) : (MIP_HEADER_LENGTH + parser->_buffer[MIP_INDEX_LENGTH] + MIP_CHECKSUM_LENGTH);
//    // Offset into the input buffer (nonzero if bytes are dropped/skipped due to garbage/non-mip data)
//    size_t unparsed_input_offset = 0;
//
//    assert(leftover_length < MIP_PACKET_LENGTH_MAX);
//    assert(expected_packet_length > leftover_length);
//
//    while((input_length - unparsed_input_offset) >= (expected_packet_length - leftover_length))
//    {
//        switch(expected_packet_length)
//        {
//        // Nothing parsed yet (expecting 1 sync byte)
//        case MIP_INDEX_SYNC1+1:
//            assert(leftover_length == 0);
//            if(input_buffer[unparsed_input_offset + MIP_INDEX_SYNC1] == MIP_SYNC1)
//                expected_packet_length++;
//            else
//                unparsed_input_offset++;
//            break;
//
//        // Got single byte of 0x75 either in the input buffer or in the parser buffer (expecting 2 bytes so far).
//        case MIP_INDEX_SYNC2+1:
//            assert(leftover_length <= MIP_INDEX_SYNC2);  // SYNC1 may or may not be in the parser buffer.
//            if(input_buffer[unparsed_input_offset + (MIP_INDEX_SYNC2 - leftover_length)] == MIP_SYNC2)
//                expected_packet_length = MIP_HEADER_LENGTH;
//            else  // Next byte is not 0x65 --> not a mip packet, reset parser
//            {
//                unparsed_input_offset++;
//                expected_packet_length = 1;
//                leftover_length        = 0;
//            }
//            break;
//
//        // Got the expected 4 bytes for the header - read packet's length field.
//        case MIP_HEADER_LENGTH:
//            assert(leftover_length < MIP_HEADER_LENGTH);  // Packet length shouldn't be in the parser buffer yet.
//            expected_packet_length = MIP_HEADER_LENGTH + input_buffer[unparsed_input_offset + MIP_INDEX_LENGTH - leftover_length] + MIP_CHECKSUM_LENGTH;
//            break;
//
//        default:  // All packet data is available, check checksum
//            assert(expected_packet_length > leftover_length);  // At least one byte should have been parsed from input.
//            memcpy(&parser->_buffer[leftover_length], &input_buffer[unparsed_input_offset], expected_packet_length - leftover_length);
//            mip_packet_from_buffer(packet_out, parser->_buffer, expected_packet_length);
//            if(mip_packet_compute_checksum(packet_out) == mip_packet_checksum_value(packet_out))
//            {
//                MIP_DIAG_INC(parser->_diag_valid_packets, 1);
//                MIP_DIAG_INC(parser->_diag_packet_bytes, packet_out->_buffer_length);
//                MIP_DIAG_INC(parser->_diag_bytes_skipped, unparsed_input_offset);
//
//                unparsed_input_offset += expected_packet_length - leftover_length;
//                MIP_DIAG_INC(parser->_diag_bytes_read, unparsed_input_offset);
//
//                return unparsed_input_offset;
//            }
//
//            MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
//
//            unparsed_input_offset++;  // Next byte
//
//            leftover_length        = 0;
//            expected_packet_length = 1;
//
//            packet_out->_buffer = NULL;
//            packet_out->_buffer_length = 0;
//            break;
//        }
//    }
//
//done:;
//    // Preserve parsed data for the next parse call (will be in leftover_length next time).
//
//    // Fresh unfinished packet --> set timeout on the unparsed data
//    if(leftover_length == 0)
//        parser->_start_time = timestamp;
//
//    // Copy the entire rest of the input buffer into the parser buffer as "leftovers" for the next parse call.
//    const size_t remaining_unparsed_input_length = input_length - unparsed_input_offset;
//
//    // The total number of leftover bytes should be less than the packet size, otherwise
//    // it would have been parsed out above.
//    assert(remaining_unparsed_input_length + leftover_length < expected_packet_length);
//
//    memcpy(&parser->_buffer[leftover_length], &input_buffer[unparsed_input_offset], remaining_unparsed_input_length);
//    parser->_buffer_length = leftover_length + remaining_unparsed_input_length;
//
//    MIP_DIAG_INC(parser->_diag_bytes_skipped, unparsed_input_offset);
//    MIP_DIAG_INC(parser->_diag_bytes_read,    input_length);
//
//    return input_length;  // All data was parsed.
//
////
////    if(parser->_packet_length)
////    {
////        expected_packet_length = MIP_HEADER_LENGTH + parser->_buffer[MIP_INDEX_LENGTH] + MIP_CHECKSUM_LENGTH;
////        assert(parser->_buffer_length < expected_packet_length);
////    }
////
////    if(parser->_buffer_length == MIP_INDEX_SYNC1)
////    {
////        for(offset=0; (offset+MIP_HEADER_LENGTH)<buffer_size; offset++)
////        {
////            if(buffer[offset] == MIP_SYNC1)
////                break;
////        }
////
////    }
////
////
////    const size_t remaining_parser_space = sizeof(parser->_buffer)-parser->_buffer_length;
////    const size_t copy_count = (buffer_size <= remaining_parser_space) ? buffer_size : remaining_parser_space;
////
////    // Copy as much data as possible to the parser buffer.
////    memcpy(&parser->_buffer[parser->_buffer_index], buffer, copy_count);
////    parser->_buffer_length += copy_count;
////    assert(parser->_buffer_length <= sizeof(parser->_buffer));
////    MIP_DIAG_INC(parser->_diag_bytes_read, copy_count);
////
////    size_t consumed = mip_parse_one_potential_packet(parser->_buffer, parser->_buffer_length, packet_out);
////
////    // Need more data?
////    if(consumed > (SIZE_MAX + MIP_PACKET_LENGTH_MAX))
////    {
////        const size_t remaining = SIZE_MAX - consumed;
////        if(parser->_buffer_length + remaining > sizeof(parser->_buffer))
////    }
////    // Valid packet?
////    else if(mip_packet_compute_checksum(packet_out) == mip_packet_checksum_value(packet_out))
////    {
////        parser->_buffer_index += consumed;
////
////        MIP_DIAG_INC(parser->_diag_packet_bytes, packet_out->_buffer_length);
////        MIP_DIAG_INC(parser->_diag_bytes_skipped, (consumed - packet_out->_buffer_length));
////        MIP_DIAG_INC(parser->_diag_valid_packets, 1);
////    }
////    else
////    {
////        MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
////        MIP_DIAG_INC(parser->_diag_bytes_skipped, consumed);
////    }
//}

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
///@param input_count
///       The number of bytes in the input buffer.
///@param timestamp
///       The local time the data was received. This is used to check for
///       timeouts and is passed to the callback as the packet timestamp.
///@param max_packets
///       The maximum number of packets to process. Unprocessed data is left in
///       the internal buffer. If 0, processing runs until no complete packets
///       remain in the buffer.
///
///@returns The number of bytes left unprocessed from the input buffer.
///         If max_packets is 0, this will also be zero as all of the data will
///         be consumed. Data may still remain in the internal buffer.
///
///@note If max_packets is 0, then this function is guaranteed to consume all
///      of the input data and the buffer can be reused or discarded afterward.
///      However, if max_packets is nonzero (meaning the number of packets parsed
///      will be limited), then this is no longer guaranteed as the excess data
///      may fill up the internal bufffer. In this case, you must process packets
///      faster than they arrive on average. For bursty data (e.g. GNSS data),
///      use a large internal buffer (see mip_parser_init) to help average out
///      the packet processing load.
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
///      conntains 0x75,0x65, has at least 6 bytes, and has a valid checksum. A
///      16-bit checksum has a 1 in 65,536 chance of appearing to be valid.
///
size_t mip_parser_parse(mip_parser* parser, const uint8_t* input_buffer, size_t input_count, timestamp_type timestamp, unsigned int max_packets)
{
    // Check for timeout
    if(timestamp >= parser->_start_time + parser->_timeout)
    {
        MIP_DIAG_INC(parser->_diag_timeouts, 1);

        // If a packet times out, it could be due to random garbage creating a fake mip header with
        // a large packet size. There may be real packets within the fake packet and these
        // need to be parsed out still.
        // Find the next possible packet (SYNC1) in the parser buffer and shift the data over so
        // that it can be parsed starting at index 0 (and there will be enough room for a max size packet)

        parser->_buffer[0] = 0x00; // NULL-out the sync byte to signal parse_one_packet.

        size_t i=0;
        for(i=1; i<parser->_buffered_length; i++)
        {
            if(parser->_buffer[i] == MIP_SYNC1)
            {
                memmove(parser->_buffer, &parser->_buffer[i], parser->_buffered_length-i);
                break;
            }
        }
        parser->_buffered_length -= i;
    }

    bool continuation = parser->_buffered_length > 0;

    size_t total_consumed = 0;

    for(unsigned int packet_num = 0; (max_packets==0) || (packet_num < max_packets); packet_num++)
    {
        size_t consumed_input_length = 0;

        int result = mip_parse_one_packet(parser->_buffer, &parser->_buffered_length, input_buffer, input_count, &consumed_input_length);

        total_consumed += consumed_input_length;

        MIP_DIAG_INC(parser->_diag_bytes_read, consumed_input_length);

        if (result > 0)  // Valid packet
        {
            MIP_DIAG_INC(parser->_diag_valid_packets, 1);
            MIP_DIAG_INC(parser->_diag_packet_bytes, parser->_buffered_length);
            MIP_DIAG_INC(parser->_diag_bytes_skipped, (consumed_input_length - parser->_buffered_length));

            parser->_start_time = timestamp;
        }
        else if(result < 0)  // Invalid packet
        {
            MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
            MIP_DIAG_INC(parser->_diag_bytes_skipped, consumed_input_length);

            parser->_buffer[0] = 0x00; // NULL-out the sync byte to signal parse_one_packet.
        }
        else // Need more data
        {
            break;
        }

        input_buffer   += consumed_input_length;
        input_count    -= consumed_input_length;

        continuation = false;
    }

    // If a NEW incomplete packet is received, update the start time.
    if(!continuation)
        parser->_start_time = timestamp;

    return total_consumed;

//    // Reset the state if the timeout time has elapsed.
//    if( parser->_packet_length != MIPPARSER_RESET_LENGTH && (timestamp - parser->_start_time) > parser->_timeout )
//    {
//        if( parser->_packet_length > 0 )
//        {
//            MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
//        }
//
//        parser->_expected_length = MIPPARSER_RESET_LENGTH;
//
//        MIP_DIAG_INC(parser->_diag_timeouts, 1);
//    }
//
//    unsigned int num_packets = 0;
//    size_t remaining = 0;
//    size_t offset = 0;
//    do
//    {
//        while(remaining >= parser->_packet_length)
//        {
//            if(parser->_packet_length == 0)
//            {
//                if(input_buffer[offset+MIP_INDEX_SYNC1] == MIP_SYNC1)
//                    parser->_buffer[parser->_packet_length++] = MIP_SYNC1;
//                else
//                    MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
//
//                offset++;
//            }
//            if(parser->_packet_length == 1)
//            {
//                if(input_buffer[offset+MIP_INDEX_SYNC2] == MIP_SYNC2)
//                {
//                    parser->_buffer[parser->_packet_length++] = MIP_SYNC2;
//                    offset++;
//                }
//                else
//                {
//                    parser->_buffer_length = 0;
//                    MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
//                }
//            }
//            if(parser->_buffer_length )
//        }
//
//
//
//        // Copy as much data as will fit in the ring buffer.
//        size_t count = byte_ring_copy_from_and_update(&parser->_ring, &input_buffer, &input_count);
//
//        MIP_DIAG_INC(parser->_diag_bytes_read, count);
//
//        mip_packet packet;
//        while( mip_parser_parse_one_packet_from_ring(parser, &packet, timestamp) )
//        {
//            num_packets++;
//            bool stop = (max_packets > 0) && (num_packets >= max_packets);
//
//            if( parser->_callback )
//                stop |= !parser->_callback(parser->_callback_object, &packet, parser->_start_time);
//
//            if( stop )
//            {
//                // Pull more data from the input buffer if possible.
//                size_t count = byte_ring_copy_from_and_update(&parser->_ring, &input_buffer, &input_count);
//
//                MIP_DIAG_INC(parser->_diag_bytes_read, count);
//
//                return -(remaining_count)input_count;
//            }
//        }
//
//        // Need more data to continue parsing.
//        // This code assumes the ring buffer is large enough for any single
//        // received mip packet, otherwise it will get stuck in an infinite loop.
//
//    } while( input_count );
//
//    return -(remaining_count)input_count;
}


//////////////////////////////////////////////////////////////////////////////////
/////@brief Parses a single packet from the internal buffer.
/////
/////@internal
/////
/////@param packet_out
/////       The mip packet to initialize with a valid packet, if found.
/////       This will have zero length if no packet is found.
/////
/////@returns If more data is required to parse the packet, the return value is
/////         SIZE_MAX - N, where N is the number of bytes needed. Otherwise, the
/////         return value is the number of bytes consumed from the buffer.
/////
//size_t mip_parse_one_potential_packet(const uint8_t* buffer, size_t buffer_size, mip_packet* packet_out)
//{
//    assert(buffer != NULL || buffer_size == 0);               // Buffer can be NULL iff buffer_size == 0
//    assert(buffer_size < (SIZE_MAX - MIP_PACKET_LENGTH_MAX)); // Buffer size must be a little smaller than SIZE_MAX
//    assert(packet_out != NULL);                               // Packet out can't be NULL
//
//    // Clear out / invalidate the packet.
//    packet_out->_buffer_length = 0;
//    packet_out->_buffer = NULL;
//
//    size_t packet_size = MIP_HEADER_LENGTH;  // Size of packet and state (MIP_HEADER_LEN, or MIP_PACKET_LENGTH_MIN+payload_len)
//    size_t offset = 0;                       // Search position in buffer
//
//    // Keep searching for a packet as long as there is enough data.
//    // Parsing will stop if there aren't enough bytes to at least read the payload length.
//    while(buffer_size >= (offset+packet_size))
//    {
//        // Packet not yet valid?
//        if(packet_size == MIP_HEADER_LENGTH)
//        {
//            // Since packet_size is always at least MIP_HEADER_LENGTH, there is enough data
//            // to read the whole header if the sync bytes are valid.
//
//            // Check if we have correct SYNC1 and SYNC2.
//            if (buffer[offset + MIP_INDEX_SYNC1] != MIP_SYNC1 || buffer[offset + MIP_INDEX_SYNC2] != MIP_SYNC2)
//                offset++;  // Nope, skip a byte and try again.
//            else // Yes! Read the expected payload length.
//                packet_size = MIP_HEADER_LENGTH + buffer[MIP_INDEX_LENGTH] + MIP_CHECKSUM_LENGTH;
//        }
//        else // All data received
//        {
//            mip_packet_from_buffer(packet_out, (uint8_t*)(buffer+offset), packet_size);
//            if( mip_packet_compute_checksum(packet_out) == mip_packet_checksum_value(packet_out) )
//                break;
//
//            // Not a valid packet, keep searching.
//            packet_out->_buffer_length = 0;
//            packet_out->_buffer = NULL;
//            offset++;
//        }
//    }
//
//    assert(offset + packet_size);
//
//    return offset + packet_size;
//
//
//
//    // Parse packets while there is sufficient data in the ring buffer.
//    while( byte_ring_count(&parser->_ring) >= parser->_expected_length )
//    {
//        if( parser->_expected_length == MIPPARSER_RESET_LENGTH )
//        {
//            if( byte_ring_at(&parser->_ring, MIP_INDEX_SYNC1) != MIP_SYNC1 )
//            {
//                byte_ring_pop(&parser->_ring, 1);
//
//                MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
//            }
//            else
//            {
//                // Synchronized - set the start time and expect more data.
//                parser->_start_time = timestamp;
//                parser->_expected_length = MIP_HEADER_LENGTH;
//            }
//        }
//        else if( parser->_expected_length == MIP_HEADER_LENGTH )
//        {
//            // Check the sync bytes and drop a single byte if not sync'd.
//            if( byte_ring_at(&parser->_ring, MIP_INDEX_SYNC2) != MIP_SYNC2 )
//            {
//                byte_ring_pop(&parser->_ring, 1);
//                MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
//                parser->_expected_length = MIPPARSER_RESET_LENGTH;
//            }
//            else
//            {
//                // Read the payload length and add it and the checksum size to the complete packet size.
//                parser->_expected_length += byte_ring_at(&parser->_ring, MIP_INDEX_LENGTH) + MIP_CHECKSUM_LENGTH;
//            }
//        }
//        else // Just waiting on enough data
//        {
//            uint_least16_t packet_length = parser->_expected_length;
//            parser->_expected_length = MIPPARSER_RESET_LENGTH;  // Reset parsing state
//
//            byte_ring_copy_to(&parser->_ring, parser->_result_buffer, packet_length);
//
//            mip_packet_from_buffer(packet_out, parser->_result_buffer, packet_length);
//
//            if( !mip_packet_is_valid(packet_out) )
//            {
//                // Invalid packet, drop just the first sync byte and restart.
//                byte_ring_pop(&parser->_ring, 1);
//                MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
//                MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
//            }
//            else // Checksum is valid
//            {
//                // Discard the packet bytes from the ring buffer since a copy was made.
//                byte_ring_pop(&parser->_ring, packet_length);
//
//                MIP_DIAG_INC(parser->_diag_valid_packets, 1);
//                MIP_DIAG_INC(parser->_diag_packet_bytes, packet_length);
//
//                // Successfully parsed a packet.
//                return true;
//            }
//        }
//    }
//
//    // Need more data to continue.
//
//    return false;
//}


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


////////////////////////////////////////////////////////////////////////////////
///@brief Obtain a pointer into which data may be read for processing.
///
/// Use this function when the source data stream (e.g. a file or serial port)
/// requires that you pass in a buffer when reading data. This avoids the need
/// for an intermediate buffer.
///
/// Call mip_parser_process_written() after the data has been read to update the
/// buffer count and process any packets.
///
///@code{.cpp}
/// uint8_t ptr;
/// size_t space = mip_parser_get_write_ptr(&parser, &ptr);
/// size_t used = fread(ptr, 1, space, file);
/// mip_parser_process_written(&parser, used);
///@endcode
///
///@param parser
///@param ptr_out
///       A pointer to a pointer which will be set to the buffer where data
///       should be written. Cannot be NULL.
///
///@returns How many bytes can be written to the buffer. Due to the use of a
///         cicular buffer, this may be less than the total available buffer
///         space. Do not write more data than specified.
///
size_t mip_parser_get_write_ptr(mip_parser* parser, uint8_t** const ptr_out)
{
    assert(ptr_out != NULL);

    return byte_ring_get_write_ptr(&parser->_ring, ptr_out);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Notify the parser that data has been written to the pointer previously
///       obtained via mip_parser_get_write_ptr().
///
/// The write pointer changes after calling this with count > 0. To write more
/// data, call mip_parser_get_write_ptr again.
///
///@param parser
///@param count
///@param timestamp
///@param max_packets
///
void mip_parser_process_written(mip_parser* parser, size_t count, timestamp_type timestamp, unsigned int max_packets)
{
    MIP_DIAG_INC(parser->_diag_bytes_read, count);

    byte_ring_notify_written(&parser->_ring, count);
    mip_parser_parse(parser, NULL, 0, timestamp, max_packets);
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
