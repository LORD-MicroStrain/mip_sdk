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
///@defgroup mip_interface Mip Device Interface
///@{
///
/// This module contains functions and classes for communicating with a MIP
/// device.
///
/// The application should call mip_interface_update() periodically to process
/// data sent by the device. This update function will call
/// mip_interface_user_recv_from_device to parse packets. When a data packet is
/// received, the list of packet and data callbacks is checked, and any
/// matching callbacks are invoked. The update function should be called at
/// a high enough rate to avoid overflowing the connection buffers. The
/// precision of the reception timestamp is dependent on the update rate.
///
/// The command functions in @ref MipCommands (e.g. mip_write_message_format() / writeMessageFormat())
/// will block execution until the command completes. Either the device will
/// respond with an ack/nack code, or the command will time out. During this
/// time, the system must be able to receive data from the device in order for
/// command replies to be detected. This occurs via the mip_interface_update()
/// function as well.
///
///@par Single-threaded applications
///
/// For single-threaded applications, data can be read from the port directly
/// from within the command function. While the command is waiting (status code
/// MIP_STATUS_WAITING / CmdResult::STATUS_WAITING), repeated calls to the
/// update function will be made. By default, the update function calls
/// mip_interface_user_recv_from_device(). Because the function is called from
/// within a loop, it should sleep for a short time to wait for data if none
/// has been received yet. Doing so prevents excessive CPU usage and lowers
/// power consumption.
///
/// The following diagram shows the typical control flow for a single-threaded
/// application. First, the device is configured by setting the message format.
/// Execution flows down into the command processing functions until
/// mip_interface_wait_for_reply() is called. This will repeatedly call
/// mip_interface_update() to pump packets from the device through the system,
/// until either an ack/nack is received or the command times out.
/// Once the device acknowledges the command, control is returned to the
/// application which then registers some data or packet callbacks. It finally
/// goes into a loop in collect_data(). Inside this loop, the update function
/// is called to process data packets.
///
/// Notice that the same update function is called from both the command
/// function and the data collection loop. If any data packets are received
/// while waiting for a command reply, associated callbacks may be executed.
/// This is why this example application registers its callbacks after the
/// format is configured properly.
///
///@image html device_update.svg
///
///@par Multi-threaded applications
///
/// For some applications, it may be desirable to run all of the data collection
/// from a separate thread. In this case, the command functions must not
/// call the update function as that would cause a race condition between the
/// command thread and the data thread. Instead, the command thread should
/// simply sleep or yield and let the data thread process the ack/nack packet.
///
/// To allow this behavior, the update function takes a boolean parameter which
/// is true when waiting on a command and false when processing data. The
/// default update function, mip_interface_default_update(), ignores this flag,
/// but applications may override it via mip_interface_set_update_function(). In
/// this case, a wrapper function can be created which implements the above
/// behavior:
///@code{.c}
/// bool user_update_function(struct mip_device* device, bool blocking)
/// {
///     // If called from the data thread, do the normal processing.
///     if( !blocking )
///         return mip_interface_default_update(device, blocking);
///
///     // Otherwise, sleep and let the data thread process the reply.
///     std::this_thread::sleep_for(std::chrono::milliseconds(10));
///     return true;
/// }
///
/// mip_interface_set_update_function(device, &user_update_function);
///@endcode
///
///@image html device_update_threaded.svg
///
/// See the threading demo for an example application.
///
///@par Other thread-safety concerns
///
///@li Data transmission to the device (for sending commands) is thread-safe
///    within the MIP SDK. If multiple threads will send to the device, the
///    application should ensure that mip_interface_user_send_to_device() is
///    thread-safe (e.g. by using a mutex).
///
///@li It is up to the application to ensure that sending and receiving from
///    separate threads is safe. This is true for the built-in serial and TCP
///    connections on most operating systems.
///
///@par Using a custom update function for other purposes
///
/// An alternate update function may be used for single-threaded
/// applications, too:
///@li To update a progress bar while waiting for commands to complete
///@li To process data from other devices
///@li To avoid blocking inside mip_interface_user_recv_from_device() when
///    called from a data processing loop.
///@li To push data through the system in a different way (e.g. without using
///    mip_interface_user_recv_from_device())
///
/// Data may be pushed into the system by calling any of these functions:
///@li mip_interface_default_update() - this is the default behavior.
///@li mip_interface_receive_bytes() - process bytes, given a buffer.
///@li mip_interface_receive_packet() - process pre-parsed packets.
///@li mip_interface_process_unparsed_packets() - continue parsing buffered data.
///
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_mip_interface  High-level C functions for controlling a MIP device.
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
////////////////////////////////////////////////////////////////////////////////
///@defgroup user_callbacks User callback functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Receives new data from the device. Called repeatedly
///       by mip_interface_update() while waiting for command responses.
///
///@param device        The mip interface object
///@param buffer        Buffer to fill with data. Should be allocated before
///                     calling this function
///@param max_lengh     Max number of bytes that can be read into the buffer.
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
///@}
////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif
