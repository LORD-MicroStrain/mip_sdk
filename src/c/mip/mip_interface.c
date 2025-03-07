
#include "mip_interface.h"

#include "mip_field.h"

#include "mip_descriptors.h"

#include <assert.h>

#include <stdio.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@typedef mip::C::mip_send_callback
///
///@brief Called from mip_interface_send_to_device() to send data to the device port.
///       The application should forward the data to the device port (e.g. a
///       serial port, TCP connection, etc).
///
///@param device
///       A pointer to the device interface. Applications can use the user data
///       pointer to access additional information such as the port handle.
///
///@param data
///       Buffer containing the data to be transmitted to the device.
///
///@param length
///       Length of data to transmit.
///
///@return True if all of the data was successfully transmitted.
///@return False if an error occurred and some or all data was definitely unable
///        to be transmitted.
///@return Applications should prefer returning true if success is uncertain
///        since command timeouts will help detect failed transmissions. If this
///        function returns false, the associated command will fail with
///        CmdResult::STATUS_ERROR. Commands will time
///        out in case of undetected transmission failures, so false positives are
///        OK. False negatives will cause commands to immediately fail even if
///        sent successfully however, and the unexpected ack/nack replies increase
///        the risk of confusing which replies go with which commands.
///

////////////////////////////////////////////////////////////////////////////////
///@typedef mip::C::mip_recv_callback
///
///@brief Called from mip_interface_recv_from_device() to receive data from the
///       device port.
///
/// This is called indirectly through mip_interface_update() to poll for new
/// data and command responses. For single-threaded applications, it will be
/// called while waiting for command replies.
///
///@param device
///       A pointer to the device interface. Applications can use the user data
///       pointer to access additional information such as the port handle.
///
///@param wait_time
///       Time to wait for data from the device. The actual time waited may
///       be less than wait_time, but it should not significantly exceed this value.
///
///@param from_cmd
///       If true, this call is a result of waiting for a command
///       to complete. Otherwise, this call is a regularly-
///       scheduled poll for data.
///
///@param timestamp_out
///       Timestamp of the data that was received.
///
///@returns True if successful, even if no data is received.
///@returns False if the port cannot be read or some other error occurs (e.g.
///         if the port is closed).
///         If this function returns false when from_cmd is true, the
///         corresponding command will fail with MIP_STATUS_ERROR.
///
///@note Except in case of error (i.e. returning false), the timestamp must be
///      set even if no data is received. This is required to allow commands
///      to time out when no response is received.
///
///@note Applications may sleep the thread or enter a low-power state while
///      waiting for data. On posix-like (e.g. desktop) systems, applications
///      should call read() with a maximum timeout of wait_time.
///      If the actual wait time is less than the requested duration, this
///      function may be called again by the MIP SDK to wait the remaining time.
///      If the actual wait time exceeds wait_time, command timeouts may take
///      longer than intended.
///
///@see mip_interface_recv_from_device
///

////////////////////////////////////////////////////////////////////////////////
///@typedef mip::C::mip_update_callback
///
///@brief Callback function typedef for custom update behavior.
///
/// This function is called whenever data should be parsed from the port:
///@li While waiting for command responses
///@li To check for new data packets
///
/// Generally an application should call mip_interface_recv_from_device() from
/// within this callback and pass the data to mip_interface_input_bytes_from_device().
/// Many applications can set this callback to mip_interface_default_update().
///
///@param device
///       The mip_interface object being updated.
///@param timeout
///       Approximate amount of time to wait for data from the device. This may
///       be an underestimate, but applications should not wait significantly
///       longer as this may cause commands to take longer to time out.
///@param from_cmd
///       If true, this call is a result of waiting for a command to complete.
///       Otherwise, this call is a regularly-scheduled poll for data. Typically
///       an application will not wait for more data if this is false, or may
///       sleep while waiting if this is true.
///
///@returns True if successful (even if no data is received).
///@returns False if an error occurs and the port cannot be read (e.g. if the
///         port is closed). Returning false will cause any pending commands to
///         fail with a status error code.
///
///@note This update call should not return false (i.e. failure) when from_cmd
///      is true and reception is handled by another thread, unless that thread
///      is not currently running. Otherwise a race condition may occur in
///      the command queue.
///

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the mip_interface components.
///
///@param device
///
///@param parse_timeout
///       Maximum length of time to wait for the end of a MIP packet. See mip_parser_init().
///@param base_reply_timeout
///       Minimum time for all commands. See mip_cmd_queue_init().
///@param send
///       A callback which is called to send data to the device.
///@param recv
///       A callback which is called when data needs to be read from the device.
///@param update
///       Optional callback which is called to perform routine tasks such as
///       checking for command timeouts. Defaults to mip_interface_default_update.
///@param user_pointer
///       Optional pointer which is passed to the send, recv, and update callbacks.
///
void mip_interface_init(
    mip_interface* device,
    mip_timeout parse_timeout, mip_timeout base_reply_timeout,
    mip_send_callback send, mip_recv_callback recv,
    mip_update_callback update, void* user_pointer)
{
    mip_parser_init(&device->_parser, &mip_interface_parse_callback, device, parse_timeout);

    device->_send_callback   = send;
    device->_recv_callback   = recv;
    device->_update_callback = update;
    device->_user_pointer    = user_pointer;

    mip_cmd_queue_init(&device->_queue, base_reply_timeout);

    mip_dispatcher_init(&device->_dispatcher);
}


////////////////////////////////////////////////////////////////////////////////
//
// Accessors
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
///@brief Sets the send callback function.
///
///@param device
///
///@param callback
///       Function which sends raw bytes to the device. This can be NULL if no
///       commands will be issued (they would fail).
///
void mip_interface_set_send_function(mip_interface* device, mip_send_callback callback)
{
    device->_send_callback = callback;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the send function pointer.
///
///@param device
///
///@returns The send callback function. May be NULL.
///
mip_send_callback mip_interface_send_function(const mip_interface* device)
{
    return device->_send_callback;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Sets the receive callback function.
///
///@param device
///
///@param callback
///       Function which gets data from the device connection.
///       If this is NULL then commands will fail and no data will be received.
///
void mip_interface_set_recv_function(mip_interface* device, mip_recv_callback callback)
{
    device->_recv_callback = callback;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the receive function pointer.
///
///@param device
///
///@returns The receive callback function. May be NULL.
///
mip_recv_callback mip_interface_recv_function(const mip_interface* device)
{
    return device->_recv_callback;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Sets the update function.
///
/// By default, the update function is mip_interface_default_update.
///
///@see mip_update_callback
///@see mip_interface_update
///
///@param device
///
///@param callback
///       Update function to call when polling the device for data.
///       If this is NULL, then update calls will fail and no data or
///       or command replies will be received.
///
void mip_interface_set_update_function(mip_interface* device, mip_update_callback callback)
{
    device->_update_callback = callback;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Gets the update function pointer.
///
///@returns The update function. Defaults to mip_interface_default_update. May
///         be NULL.
///
mip_update_callback mip_interface_update_function(const mip_interface* device)
{
    return device->_update_callback;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Sets an optional user data pointer which can be retrieved later.
///
///@param device
///@param pointer
///
void mip_interface_set_user_pointer(mip_interface* device, void* pointer)
{
    device->_user_pointer = pointer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Retrieves the pointer set by mip_interface_set_user_pointer().
///
///@param device
///
///@returns The pointer value.
///
void* mip_interface_user_pointer(const mip_interface* device)
{
    return device->_user_pointer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the MIP parser for the device.
///
mip_parser* mip_interface_parser(mip_interface* device)
{
    return &device->_parser;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the commmand queue for the device.
///
mip_cmd_queue* mip_interface_cmd_queue(mip_interface* device)
{
    return &device->_queue;
}


////////////////////////////////////////////////////////////////////////////////
//
// Communications
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
///@brief Sends data to the port (i.e. from this library to the physical device).
///
///@param device The mip_interface object.
///@param data   Data to be sent.
///@param length Length of data.
///
///@returns True if the data was sent successfully.
///@returns False if the send callback is NULL.
///@returns False if some or all data could not be sent.
///
/// This is called whenever bytes must be sent to the physical device.
///
bool mip_interface_send_to_device(mip_interface* device, const uint8_t* data, size_t length)
{
    return device->_send_callback && device->_send_callback(device, data, length);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Checks for data at the port and reads it into buffer.
///
///@param device
///@param buffer        A place to store the data.
///@param max_length    Maximum number of bytes to read into buffer.
///@param wait_time     Maximum time to wait for data. May be 0.
///@param length_out    The number of bytes successfully read into buffer.
///@param timestamp_out The timestamp of the received data.
///
///@returns True if successful (even if 0 bytes were read).
///@returns False if the receive callback is NULL.
///@returns False if the receive callback failed (i.e. if it returned false).
///
bool mip_interface_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, bool from_cmd, size_t* length_out, mip_timestamp* timestamp_out)
{
    return device->_recv_callback && device->_recv_callback(device, buffer, max_length, wait_time, from_cmd, length_out, timestamp_out);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Call to process data from the device.
///
/// This function is also called while waiting for command replies.
///
/// Call this periodically to process packets received from the device. It
/// should be called at a suitably high rate to prevent the connection buffers
/// from overflowing. The update rate affects the reception timestamp resolution.
///
///@param device
///
///@param wait_time
///       Time to wait for data from the device. This will be nonzero when
///       waiting for command replies. Applications calling this function
///       can pass 0 to avoid blocking when checking for new data.
///
///@param from_cmd
///       If true, this call is a result of waiting for a command to complete.
///       Otherwise, this call is a regularly-scheduled poll for data. User
///       code calling this function should generally set this to false.
///
///@returns true if operation should continue, or false if the device cannot be
///         updated (e.g. if the serial port is not open).
///

bool mip_interface_update(mip_interface* device, mip_timeout wait_time, bool from_cmd)
{
    return device->_update_callback && device->_update_callback(device, wait_time, from_cmd);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Polls the port for new data or command replies.
///
/// This is the default choice for the user update function. It ignores the
/// from_cmd flag and always tries to read data from the device.
///
///@warning This function is provided for convenience and quick setup. It will
///         work for most applications, but it is not optimized for your
///         application. It may be unsuitable for some situations such as
///         small microcontrollers (due to the fixed-size stack-allocated data
///         buffer) or in combination with multi-threading.
///
///@param device
///
///@param wait_time
///       Time to wait for data from the device. This will be nonzero when
///       waiting for command replies. Applications calling this function
///       can pass 0 to avoid blocking when checking for new data.
///
///@param from_cmd
///       If true, this call is a result of waiting for a command to complete.
///       Otherwise, this call is a regularly-scheduled poll for data. User
///       code calling this function should generally set this to false.
///
///@returns The value returned by mip_interface_user_recv_from_device.
///
bool mip_interface_default_update(mip_interface* device, mip_timeout wait_time, bool from_cmd)
{
    // Allocate a buffer to hold received data.
    // Note: bigger buffers will parse faster, but may overflow the stack.
    // 512 bytes seems to be a good compromise on both desktop and embedded devices.
    uint8_t buffer[512];

    return mip_interface_default_update_ext_buffer(device, wait_time, from_cmd, buffer, sizeof(buffer));
}


////////////////////////////////////////////////////////////////////////////////
///@brief Polls the port for new data or command replies using a supplied buffer.
///
/// This function is suitable for most single-threaded use cases where
/// performance is not critical. In performance-sensitive applications, it's
/// best to either read directly into the mip parser buffer, or call
/// mip_interface_input_bytes from your own update function (bypassing
/// mip_interface_recv_from_device entirely).
///
///@param device
///
///@param wait_time
///       Time to wait for data from the device. This will be nonzero when
///       waiting for command replies. Applications calling this function
///       can pass 0 to avoid blocking when checking for new data.
///
///@param from_cmd
///       If true, this call is a result of waiting for a command to complete.
///       Otherwise, this call is a regularly-scheduled poll for data. User
///       code calling this function should generally set this to false.
///
///@param buffer
///       Buffer to hold data read from the device connection. At least 512
///       bytes are recommended for better performance, with a few kB being
///       the point of diminishing return.
///
///@param buffer_size
///       Size of the buffer.
///
///@returns The value returned by mip_interface_user_recv_from_device.
///
bool mip_interface_default_update_ext_buffer(mip_interface* device, mip_timeout wait_time, bool from_cmd, uint8_t* buffer, size_t buffer_size)
{
    if( !device->_recv_callback )
        return false;

    size_t count = 0;
    mip_timestamp timestamp;

    // Try to read data from the port into a temporary buffer.
    if ( !(device->_recv_callback)(device, buffer, buffer_size, wait_time, from_cmd, &count, &timestamp) )
        return false;

    mip_interface_input_bytes_andor_time(device, buffer, count, timestamp);

    return true;
}


////////////////////////////////////////////////////////////////////////////////
///@brief This function takes care of processing received data and updating the
///       current time.
///
/// User-defined update functions should either call this function after
/// getting data from the connection, or do the equivalent manually:
///@li Input bytes to the parser, via mip_interface_input_bytes_from_device, and
///@li Update the current time via mip_interface_update_time
///
void mip_interface_input_bytes_andor_time(mip_interface* device, const uint8_t* received_data, size_t data_length, mip_timestamp now)
{
    // Pass the data to the MIP parser.
    mip_interface_input_bytes_from_device(device, received_data, data_length, now);

    // Update the command queue to see if any commands have timed out.
    mip_interface_update_time(device, now);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Passes data from the device into the parser.
///
///@param device
///
///@param data
///       Input data buffer. May be NULL if length == 0.
///@param length
///       Length of the input buffer. Must be 0 if data is NULL.
///@param timestamp
///       Time of the received data.
///
void mip_interface_input_bytes_from_device(mip_interface* device, const uint8_t* data, size_t length, mip_timestamp timestamp)
{
    mip_parser_parse(&device->_parser, data, length, timestamp);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Processes a pre-parsed packet for command replies and data.
///
///@param device
///
///@param packet
///       The received MIP packet.
///@param timestamp
///       Timestamp of the received MIP packet.
///
void mip_interface_input_packet_from_device(mip_interface* device, const mip_packet_view* packet, mip_timestamp timestamp)
{
    mip_cmd_queue_process_packet(&device->_queue, packet, timestamp);
    mip_dispatcher_dispatch_packet(&device->_dispatcher, packet, timestamp);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Call this to ensure that pending commands time out properly.
///
/// This function should be called regularly (i.e. from within the update
/// callback). Otherwise, unacknowledged commands may never time out. This can
/// happen even if data is being sent through mip_interface_input_bytes_from_device
/// because the device may be streaming data. See the implementation of
/// mip_interface_default_update for an example.
///
///param device
///param timestamp Current time, as if timestamping received data.
///
void mip_interface_update_time(mip_interface* device, mip_timestamp timestamp)
{
    mip_cmd_queue_update(mip_interface_cmd_queue(device), timestamp);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Wrapper around mip_interface_input_packet_from_device for use with mip_parser.
///
///@param device    Void pointer to the device. Must be a mip_interface pointer.
///@param packet    MIP Packet from the parser.
///@param timestamp Timestamp of the packet.
///
void mip_interface_parse_callback(void* device, const mip_packet_view* packet, mip_timestamp timestamp)
{
    mip_interface_input_packet_from_device(device, packet, timestamp);
}


////////////////////////////////////////////////////////////////////////////////
//
// Command and data processing
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
///@brief Blocks until the pending command completes or times out.
///
///@param device
///@param cmd
///
///@returns The final status of the command.
///
enum mip_cmd_result mip_interface_wait_for_reply(mip_interface* device, mip_pending_cmd* cmd)
{
    enum mip_cmd_result status;
    while( !mip_cmd_result_is_finished(status = mip_pending_cmd_status(cmd)) )
    {
        if( !mip_interface_update(device, 0, true) )
        {
            // When this function returns the pending command may be deallocated and the
            // queue will have a dangling pointer. Therefore, the command must be manually
            // errored out and de-queued.
            //
            // Note: This fix can still cause a race condition in multithreaded apps if the
            // update thread happens to run right before the cmd is dequeued. The user is
            // advised to not fail the update callback when another thread is handling
            // reception, unless that thread is not running. Generally such updates shouldn't
            // fail as long as the other thread is working normally anyway.

            mip_cmd_queue_dequeue(mip_interface_cmd_queue(device), cmd);
            cmd->_status = MIP_STATUS_ERROR;

            return MIP_STATUS_ERROR;
        }
    }
    return status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Runs a command using a pre-serialized payload.
///
///@param device
///@param descriptor_set
///       Command descriptor set.
///@param cmd_descriptor
///       Command field descriptor.
///@param cmd_data
///       Optional payload data. May be NULL if cmd_length == 0.
///@param cmd_length
///       Length of the command payload (parameters).
///
///@return mip_cmd_result
///        MIP_ACK_OK - Command completed successfully.
///        MIP_NACK_* - Device rejected the command.
///        MIP_STATUS_* - An error occured (e.g. timeout).
///
enum mip_cmd_result mip_interface_run_command(mip_interface* device, uint8_t descriptor_set, uint8_t cmd_descriptor, const uint8_t* cmd_data, uint8_t cmd_length)
{
    return mip_interface_run_command_with_response(device, descriptor_set, cmd_descriptor, cmd_data, cmd_length, MIP_INVALID_FIELD_DESCRIPTOR, NULL, NULL);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Runs a command using a pre-serialized payload.
///
///@param device
///@param descriptor_set
///       Command descriptor set.
///@param cmd_descriptor
///       Command field descriptor.
///@param cmd_data
///       Optional payload data. May be NULL if cmd_length == 0.
///@param cmd_length
///       Length of the command payload (parameters).
///@param response_descriptor
///       Descriptor of the response data. May be MIP_INVALID_FIELD_DESCRIPTOR
///       if no response is expected.
///@param response_buffer
///       Buffer to hold response data. Can be the same as the command data buffer.
///       Can be NULL if response_descriptor is MIP_INVALID_FIELD_DESCRIPTOR.
///@param[in,out] response_length_inout
///       As input, the size of response buffer and max response length.
///       As output, returns the actual length of the response data.
///
///@returns mip_cmd_result
///
enum mip_cmd_result mip_interface_run_command_with_response(mip_interface* device,
    uint8_t descriptor_set, uint8_t cmd_descriptor, const uint8_t* cmd_data, uint8_t cmd_length,
    uint8_t response_descriptor, uint8_t* response_buffer, uint8_t* response_length_inout)
{
    assert((response_descriptor == MIP_INVALID_FIELD_DESCRIPTOR) || ((response_buffer != NULL) && (response_length_inout != NULL)) );

    uint8_t buffer[MIP_PACKET_LENGTH_MAX];

    mip_packet_view packet;
    mip_packet_create(&packet, buffer, sizeof(buffer), descriptor_set);
    mip_packet_add_field(&packet, cmd_descriptor, cmd_data, cmd_length);
    mip_packet_finalize(&packet);

    mip_pending_cmd cmd;
    const uint8_t response_length = response_length_inout ? *response_length_inout : 0;
    mip_pending_cmd_init_with_response(&cmd, descriptor_set, cmd_descriptor, response_descriptor, response_buffer, response_length);

    enum mip_cmd_result result = mip_interface_run_command_packet(device, &packet, &cmd);

    if( response_length_inout )
        *response_length_inout = mip_pending_cmd_response_length(&cmd);

    return result;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Similar to mip_interface_start_command_packet but waits for the
///       command to complete.
///
///@param device
///@param packet
///       A MIP packet containing the command.
///@param cmd
///       The command status tracker. No lifetime requirement.
///
enum mip_cmd_result mip_interface_run_command_packet(mip_interface* device, const mip_packet_view* packet, mip_pending_cmd* cmd)
{
    if( !mip_interface_start_command_packet(device, packet, cmd) )
        return MIP_STATUS_ERROR;

    return mip_interface_wait_for_reply(device, cmd);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Queues the command and sends the packet. Does not wait for completion.
///
///@param device
///@param packet
///       A MIP packet containing the command.
///@param cmd
///       The command status tracker. Must be valid while the command executes.
///
///@returns True if successful. Cmd must remain valid until the command finishes.
///@returns False on error sending the packet. No cleanup is necessary and cmd
///         can be destroyed immediately afterward in this case.
///
bool mip_interface_start_command_packet(mip_interface* device, const mip_packet_view* packet, mip_pending_cmd* cmd)
{
    mip_cmd_queue_enqueue(mip_interface_cmd_queue(device), cmd);

    if( !mip_interface_send_to_device(device, mip_packet_pointer(packet), mip_packet_total_length(packet)) )
    {
        mip_cmd_queue_dequeue(mip_interface_cmd_queue(device), cmd);
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Registers a callback for packets of the specified descriptor set.
///
///@param device
///
///@param handler
///       An uninitialized mip_dispatch_handler object. This call will initialize it.
///@param descriptor_set
///@param after_fields
///@param callback
///@param user_data
///
///@see mip_dispatch_handler_init_packet_handler for details.
///
void mip_interface_register_packet_callback(
    mip_interface* device, mip_dispatch_handler* handler,
    uint8_t descriptor_set, bool after_fields, mip_dispatch_packet_callback callback, void* user_data)
{
    mip_dispatch_handler_init_packet_handler(handler, descriptor_set, after_fields, callback, user_data);
    mip_dispatcher_add_handler(&device->_dispatcher, handler);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Registers a callback for packets of the specified descriptor set.
///
///@param device
///
///@param handler
///       An uninitialized mip_dispatch_handler object. This call will initialize it.
///@param descriptor_set
///@param field_descriptor
///@param callback
///@param user_data
///
///@see mip_dispatch_handler_init_field_handler for details.
///
void mip_interface_register_field_callback(
    mip_interface* device, mip_dispatch_handler* handler,
    uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* user_data)
{
    mip_dispatch_handler_init_field_handler(handler, descriptor_set, field_descriptor, callback, user_data);
    mip_dispatcher_add_handler(&device->_dispatcher, handler);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Registers a callback for packets of the specified descriptor set.
///
///@param device
///
///@param handler
///       An uninitialized mip_dispatch_handler object. This call will initialize it.
///@param descriptor_set
///@param field_descriptor
///@param extractor
///@param field_ptr
///
///@see mip_dispatch_handler_init_extract_handler for details.
///
void mip_interface_register_extractor(
    mip_interface* device, mip_dispatch_handler* handler,
    uint8_t descriptor_set, uint8_t field_descriptor,
    mip_dispatch_extractor extractor, void* field_ptr)
{
    mip_dispatch_handler_init_extractor(handler, descriptor_set, field_descriptor, extractor, field_ptr);
    mip_dispatcher_add_handler(&device->_dispatcher, handler);
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif
