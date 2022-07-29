#pragma once

#include "../types.h"

#ifdef __cplusplus
namespace mscl{
namespace C {
extern "C" {
#endif



////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_packet mip_packet - Functions for handling MIP packets.
///
/// A MIP Packet is represented by the mip_packet struct.
///
///~~~
/// +-------+-------+------+------+------------+-----/ /----+------------+----
/// | SYNC1 | SYNC2 | DESC | PLEN |   Field    |     ...    |  Checksum  |  remaining buffer space...
/// +-------+-------+------+------+------------+-----/ /----+------------+----
///~~~
///
///@{


typedef uint_least16_t packet_length;  ///< Type used for the length of a MIP packet.

////////////////////////////////////////////////////////////////////////////////
///@brief Structure representing a MIP Packet.
///
/// Use to inspect received packets or construct new ones.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
struct mip_packet
{
    uint8_t*       _buffer;        ///<@private Pointer to the packet data.
    uint_least16_t _buffer_length;  ///<@private Length of the buffer (NOT the packet length!).
};


////////////////////////////////////////////////////////////////////////////////
///@defgroup PacketBuilding  Packet Building - Functions for building new MIP packets.
///
/// Use these functions to create a new packet, add fields, and write the
/// checksum.
///
///@{

void mip_packet_create(struct mip_packet* packet, uint8_t* buffer, size_t buffer_size, uint8_t descriptor_set);

bool            mip_packet_add_field(struct mip_packet* packet, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length);
remaining_count mip_packet_alloc_field(struct mip_packet* packet, uint8_t field_descriptor, uint8_t payload_length, uint8_t** payload_ptr_out);
remaining_count mip_packet_realloc_last_field(struct mip_packet* packet, uint8_t* payload_ptr, uint8_t new_payload_length);
remaining_count mip_packet_cancel_last_field(struct mip_packet* packet, uint8_t* payload_ptr);

void mip_packet_finalize(struct mip_packet* packet);

void mip_packet_reset(struct mip_packet* packet, uint8_t descriptor_set);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup Accessors  Accessors - Functions for accessing information about an existing MIP packet.
///
/// Use these functions to get information about a MIP packet after it has been
/// parsed. Generally, first the descriptor set would be inspected followed by
/// iterating the fields using the MipFieldIteration functions.
///
/// With the exception of mip_packet_checksum_value() (and any function which
/// calls it, e.g. mip_packet_is_valid()), these functions may also be used on
/// packets which are under construction via the PacketBuilding functions.
///
///@{

void mip_packet_from_buffer(struct mip_packet* packet, uint8_t* buffer, size_t length);

uint8_t         mip_packet_descriptor_set(const struct mip_packet* packet);
packet_length   mip_packet_total_length(const struct mip_packet* packet);
uint8_t         mip_packet_payload_length(const struct mip_packet* packet);
const uint8_t*  mip_packet_pointer(const struct mip_packet* packet);
const uint8_t*  mip_packet_payload(const struct mip_packet* packet);
uint16_t        mip_packet_checksum_value(const struct mip_packet* packet);
uint16_t        mip_packet_compute_checksum(const struct mip_packet* packet);


bool            mip_packet_is_sane(const struct mip_packet* packet);
bool            mip_packet_is_valid(const struct mip_packet* packet);
bool            mip_packet_is_empty(const struct mip_packet* packet);

packet_length   mip_packet_buffer_size(const struct mip_packet* packet);
remaining_count mip_packet_remaining_space(const struct mip_packet* packet);

bool            mip_packet_is_data(const struct mip_packet* packet);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mscl
} // namespace C
} // extern "C"
#endif
