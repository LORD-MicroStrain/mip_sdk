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
///@addtogroup mip_c
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_interface_c  Mip Interface [C]
///
///@brief High-level C functions for controlling a MIP device.
///
/// This module contains functions and classes for communicating with a
/// MIP device in C.
///
///
///@li Sending commands
///@li Receiving Data
///
///@{

struct mip_interface;

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom update behavior.
///
///@param device   The mip_interface object being updated.
///@param blocking True if called from within a blocking command function.
///
///@returns False if an error occurs and the port cannot be read (e.g. if the
///         port is closed). Returning false will cause any pending commands to
///         fail with a status error code.
///@returns True if successful (even if no data is received).
///
typedef bool (*mip_update_callback)(struct mip_interface* device, bool blocking);

////////////////////////////////////////////////////////////////////////////////
///@brief State of the interface for communicating with a MIP device.
///
struct mip_interface
{
    struct mip_parser     _parser;          ///<@private MIP Parser for incoming MIP packets.
    struct mip_cmd_queue  _queue;           ///<@private Queue for checking command replies.
    struct mip_dispatcher _dispatcher;      ///<@private Dispatcher for data callbacks.
    unsigned int          _max_update_pkts; ///<@private Max number of MIP packets to parse at once.
    mip_update_callback   _update_function; ///<@private Optional function to call during updates.
    void*                 _user_pointer;    ///<@private Optional user-specified data pointer.
};


void mip_interface_init(struct mip_interface* device, uint8_t* parse_buffer, size_t parse_buffer_size, timeout_type parse_timeout, timeout_type base_reply_timeout);

//
// Communications
//

remaining_count mip_interface_receive_bytes(struct mip_interface* device, const uint8_t* data, size_t length, timestamp_type timestamp);
void mip_interface_process_unparsed_packets(struct mip_interface* device);
bool mip_interface_update(struct mip_interface* device, bool blocking);
bool mip_interface_default_update(struct mip_interface* device, bool blocking);

bool mip_interface_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length);

bool mip_interface_parse_callback(void* device, const struct mip_packet* packet, timestamp_type timestamp);
void mip_interface_receive_packet(struct mip_interface* device, const struct mip_packet* packet, timestamp_type timestamp);

//
// Commands
//

enum mip_cmd_result mip_interface_wait_for_reply(struct mip_interface* device, const struct mip_pending_cmd* cmd);
enum mip_cmd_result mip_interface_run_command(struct mip_interface* device, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length);
enum mip_cmd_result mip_interface_run_command_with_response(struct mip_interface* device, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length, uint8_t response_descriptor, uint8_t* response_data, uint8_t* response_length_inout);
enum mip_cmd_result mip_interface_run_command_packet(struct mip_interface* device, const struct mip_packet* packet, struct mip_pending_cmd* cmd);

bool mip_interface_start_command_packet(struct mip_interface* device, const struct mip_packet* packet, struct mip_pending_cmd* cmd);

//
// Data Callbacks
//

void mip_interface_register_packet_callback(struct mip_interface* device, struct mip_dispatch_handler* handler, uint8_t descriptor_set, bool after_fields, mip_dispatch_packet_callback callback, void* user_data);
void mip_interface_register_field_callback(struct mip_interface* device, struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* user_data);
void mip_interface_register_extractor(struct mip_interface* device, struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_extractor callback, void* field_ptr);

//
// Accessors
//

void mip_interface_set_update_function(struct mip_interface* device, mip_update_callback function);
void mip_interface_set_user_pointer(struct mip_interface* device, void* pointer);
void mip_interface_set_max_packets_per_update(struct mip_interface* device, unsigned int max_packets);
unsigned int mip_interface_max_packets_per_update(const struct mip_interface* device);

mip_update_callback mip_interface_update_function(struct mip_interface* device);
void* mip_interface_user_pointer(const struct mip_interface* device);
struct mip_parser*    mip_interface_parser(struct mip_interface* device);
struct mip_cmd_queue* mip_interface_cmd_queue(struct mip_interface* device);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup user_callbacks  User callback functions [C/CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Receives new data from the device. Called repeatedly
///       by mip_interface_update() while waiting for command responses.
///
///@param device        The mip interface object
///@param buffer        Buffer to fill with data. Should be allocated before
///                     calling this function
///@param max_length    Max number of bytes that can be read into the buffer.
///@param out_length    Number of bytes actually read into the buffer.
///@param timestamp_out Timestamp of the data was received.
///
///@returns true if operation should continue, or false if the device cannot be
///         updated (e.g. if the serial port is not open).
///
///@note Except in case of error (i.e. returning false), the timestamp must be
///      set even if no data is received. This is required to allow commands
///      to time out.
///
///@note On systems where it makes sense, this is a good place to call sleep
///      or enter a low-power state until data arrives at the port. Typically
///      this function will wait a few milliseconds before returning.
///
///@warning Do not block indefinitely as this will stall the system beyond the
///         normal command timeout. Use a sensible timeout (i.e. 1/10th of the
///         base reply timeout) or only sleep for a minimal amount of time.
///
extern bool mip_interface_user_recv_from_device(struct mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, timestamp_type* timestamp_out);


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
////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif
