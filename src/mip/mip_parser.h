#pragma once

#include "mip_packet.h"
#include "mip_offsets.h"

#include "utils/byte_ring.h"
#include "mip_types.h"

#ifdef __cplusplus
namespace mip{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_parser mip_parser - Functions for parsing MIP packets
///@{
///


///@brief Callback function which receives parsed MIP packets.
///@param user A user-specified pointer which will be given the callback_object parameter which was previously passed to mip_parser_init.
///@param Packet A pointer to the MIP packet. Do not store this pointer as it will be invalidated after the callback returns.
///@param timestamp The approximate time the packet was parsed.
typedef bool (*mip_packet_callback)(void* user, const struct mip_packet* packet, timestamp_type timestamp);


////////////////////////////////////////////////////////////////////////////////
///@brief MIP Parser state.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
struct mip_parser
{
    timestamp_type       _start_time;                           ///<@private The timestamp when the first byte was observed by the parser.
    timestamp_type       _timeout;                              ///<@private Duration to wait for the rest of the data in a packet.
    uint8_t              _result_buffer[MIP_PACKET_LENGTH_MAX]; ///<@private Buffer used to output MIP packets to the callback.
    packet_length        _expected_length;                      ///<@private Expected length of the packet currently being parsed. Keeps track of parser state. Always 1, MIP_HEADER_LENGTH, or at least MIP_PACKET_LENGTH_MAX.
    struct ByteRingState _ring;                                 ///<@private Ring buffer which holds data being parsed. User-specified backing buffer and size.
    mip_packet_callback  _callback;                             ///<@private Callback called when a valid packet is parsed. Can be NULL.
    void*                _callback_object;                      ///<@private User-specified pointer passed to the callback function.
};



#define MIPPARSER_UNLIMITED_PACKETS   0   ///< Specifies no limit when used as the max_packets argument to mip_parser_parse.
#define MIPPARSER_DEFAULT_TIMEOUT_MS 100  ///< Specifies the default timeout for a MIP parser, assuming timestamps are in milliseconds.


void mip_parser_init(struct mip_parser* parser, uint8_t* buffer, size_t buffer_size, mip_packet_callback callback, void* callback_object, timestamp_type timeout);
bool mip_parser_parse_one_packet_from_ring(struct mip_parser* parser, struct mip_packet* packet_out, timestamp_type timestamp);
remaining_count mip_parser_parse(struct mip_parser* parser, const uint8_t* input_buffer, size_t input_count, timestamp_type timestamp, unsigned int max_packets);

void mip_parser_reset(struct mip_parser* parser);

size_t mip_parser_get_write_ptr(struct mip_parser* parser, uint8_t** ptr_out);
void mip_parser_process_written(struct mip_parser* parser, size_t count, timestamp_type timestamp, unsigned int max_packets);

//
// Accessors
//

timeout_type mip_parser_timeout(const struct mip_parser* parser);
void mip_parser_set_timeout(struct mip_parser* parser, timeout_type timeout);

void mip_parser_set_callback(struct mip_parser* parser, mip_packet_callback callback, void* callback_object);
mip_packet_callback mip_parser_callback(const struct mip_parser* parser);
void* mip_parser_callback_object(const struct mip_parser* parser);

timestamp_type mip_parser_last_packet_timestamp(const struct mip_parser* parser);

//
// Misc
//

timeout_type mip_timeout_from_baudrate(uint32_t baudrate);

///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif
