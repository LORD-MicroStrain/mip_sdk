#pragma once

#include <stdint.h>
#include <stddef.h>

#include "mip_parser.h"
#include "mip_cmdqueue.h"
#include "mip_dispatch.h"

#ifdef __cplusplus
namespace mip{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_interface  High-level functions for controlling a MIP device.
///
///- Sending commands
///- Receiving Data
///
/// There are two ways to handle input data:
/// 1. Call mip_interface_receive_bytes (and/or mip_interface/process_unparsed_packets)
///    from mip_interface_user_update(), or
/// 2. Run a separate thread which feeds data from the port into
///    mip_interface_receive_bytes(). mip_interface_user_update() would then just do
///    nothing or sleep/yield.
///
/// Example of the first approach:
///@code{.c}
/// bool mip_interface_user_update(struct mip_interface* device)
/// {
///     size_t count;
///     uint8_t buffer[N];
///     if( user_read_port(buffer, sizeof(buffer), &count) == ERROR )
///         return false;  // Abort further processing if the port is closed.
///
///     mip_interface_receive_bytes(device, buffer, count, user_get_time(), MIPPARSER_UNLIMITED_PACKETS);
/// }
///@endcode
///
/// Example of the second approach:
///@code{.c}
/// bool mip_interface_user_update(struct mip_interface* device)
/// {
///     user_sleep();
///     return true;
/// }
///
/// // In another thread
/// for(;;)
/// {
///     user_wait_for_data();
///     mip_interface_receive_bytes(device, buffer, count, user_get_time(), MIPPARSER_UNLIMITED_PACKETS);
/// }
///@endcode
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief State of the interface for communicating with a MIP device.
///
struct mip_interface
{
    struct mip_parser     _parser;          ///<@private MIP Parser for incoming MIP packets.
    struct mip_cmd_queue  _queue;           ///<@private Queue for checking command replies.
    struct mip_dispatcher _dispatcher;      ///<@private Dispatcher for data callbacks.
    unsigned int          _max_update_pkts; ///<@private Max number of MIP packets to parse at once.
    void*                 _user_pointer;    ///<@private Optional user-specified data pointer.
};


void mip_interface_init(struct mip_interface* device, uint8_t* parse_buffer, size_t parse_buffer_size, timeout_type parse_timeout, timeout_type base_reply_timeout);

//
// Communications
//

remaining_count mip_interface_receive_bytes(struct mip_interface* device, const uint8_t* data, size_t length, timestamp_type timestamp);
void mip_interface_process_unparsed_packets(struct mip_interface* device);
bool mip_interface_update(struct mip_interface* device);

bool mip_interface_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length);

bool mip_interface_parse_callback(void* device, const struct mip_packet* packet, timestamp_type timestamp);
void mip_interface_receive_packet(struct mip_interface* device, const struct mip_packet* packet, timestamp_type timestamp);

//
// Commands
//

mip_cmd_result mip_interface_wait_for_reply(struct mip_interface* device, const struct mip_pending_cmd* cmd);
mip_cmd_result mip_interface_run_command(struct mip_interface* device, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length);
mip_cmd_result mip_interface_run_command_with_response(struct mip_interface* device, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length, uint8_t response_descriptor, uint8_t* response_data, uint8_t* response_length_inout);
mip_cmd_result mip_interface_run_command_packet(struct mip_interface* device, const struct mip_packet* packet, struct mip_pending_cmd* cmd);

//
// Data Callbacks
//

void mip_interface_register_packet_callback(struct mip_interface* device, struct mip_dispatch_handler* handler, uint8_t descriptor_set, mip_dispatch_packet_callback callback, void* user_data);
void mip_interface_register_field_callback(struct mip_interface* device, struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* user_data);

//
// Accessors
//

void mip_interface_set_user_pointer(struct mip_interface* device, void* pointer);
void mip_interface_set_max_packets_per_update(struct mip_interface* device, unsigned int max_packets);
unsigned int mip_interface_max_packets_per_update(const struct mip_interface* device);

void* mip_interface_user_pointer(const struct mip_interface* device);
struct mip_parser*    mip_interface_parser(struct mip_interface* device);
struct mip_cmd_queue* mip_interface_cmd_queue(struct mip_interface* device);



////////////////////////////////////////////////////////////////////////////////
///@defgroup UserFunctions  User-implemented callback functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@copydoc mip_interface_update
///
///@note If a limit is placed on the max number of packets to parse at once,
///      Make sure to call mip_interface_receive_bytes(), or at least
///      pip_parser_parse_one_packet_from_ring() on the parser, even if no data is
///      available. Otherwise command replies and data may not get through the
///      system quickly enough.
///
///@warning Do not block indefinitely as this will stall the system beyond the
///         normal command timeout. Use a sensible timeout (i.e. 1/10th of the
///         base reply timeout) or only sleep for a minimal amount of time.
///
extern bool mip_interface_user_update(struct mip_interface* device);


////////////////////////////////////////////////////////////////////////////////
///@copydoc mip_interface_send_to_device
///
///@note This is a good place to put logging code for debugging device
///      communications at the byte level.
///
///@note There are cases where the data will not be a MIP packet.
///
extern bool mip_interface_user_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length);


///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif
