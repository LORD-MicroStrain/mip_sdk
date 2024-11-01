
#include "mip_serialization.h"
#include "mip_offsets.h"

#include <assert.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a serialization struct for creation of a new field at the
///       end of the packet.
///
///@note Call mip_serializer_finish_new_field after the data has been serialized.
///
///@note Only one new field per packet can be in progress at a time.
///
///@param serializer
///@param packet
///       Allocate the new field on the end of this packet.
///@param field_descriptor
///       Field descriptor of the new field.
///
void mip_serializer_init_new_field(mip_serializer* serializer, mip_packet_view* packet, uint8_t field_descriptor)
{
    assert(packet);

    serializer->_buffer      = NULL;
    serializer->_buffer_size = 0;
    serializer->_offset      = 0;

    const int length = mip_packet_create_field(packet, field_descriptor, 0, &serializer->_buffer);

    if( length >= 0 )
        serializer->_buffer_size = length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Call this after a new field allocated by mip_serializer_init_new_field
///       has been written.
///
/// This will either finish the field, or abort it if the serializer failed.
///
///@param serializer Must be created from mip_serializer_init_new_field.
///@param packet     Must be the original packet.
///
void mip_serializer_finish_new_field(const mip_serializer* serializer, mip_packet_view* packet)
{
    assert(packet);

    if(microstrain_serializer_is_ok(serializer) )
    {
        assert(serializer->_offset <= MIP_FIELD_LENGTH_MAX);  // Payload too long!
        mip_packet_update_last_field_length(packet, serializer->_buffer, (uint8_t) serializer->_offset);
    }
    else if( serializer->_buffer )
        mip_packet_cancel_last_field(packet, serializer->_buffer);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a serialization struct from a microstrain field payload.
///
///@param serializer
///@param field
///
void microstrain_serializer_init_from_field(mip_serializer* serializer, const mip_field_view* field)
{
    microstrain_serializer_init_extraction(serializer, mip_field_payload(field), mip_field_payload_length(field));
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif
