
#include "mip_interface.h"

#include "mip_field.h"

#include "definitions/descriptors.h"

#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Wrapper around mip_interface_receive_packet for use with mip_parser.
///
///@param device    Void pointer to the device. Must be a mip_interface pointer.
///@param packet    MIP Packet from the parser.
///@param timestamp timestamp_type of the packet.
///
///@returns True
///
bool mip_interface_parse_callback(void* device, const struct mip_packet* packet, timestamp_type timestamp)
{
    mip_interface_receive_packet(device, packet, timestamp);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the mip_interface components.
///
///@param device
///
///@param parse_buffer
///       A working buffer for the MIP parser. See mip_parser_init().
///@param parse_buffer_size
///       Size of the parsing buffer. Must be at least MIP_PACKET_LENGTH_MAX.
///@param parse_timeout
///       Maximum length of time to wait for the end of a MIP packet. See mip_parser_init().
///@param base_reply_timeout
///       Minimum time for all commands. See mip_cmd_queue_init().
///
void mip_interface_init(struct mip_interface* device, uint8_t* parse_buffer, size_t parse_buffer_size, timeout_type parse_timeout, timeout_type base_reply_timeout)
{
    mip_parser_init(&device->_parser, parse_buffer, parse_buffer_size, &mip_interface_parse_callback, device, parse_timeout);

    device->_max_update_pkts = MIPPARSER_UNLIMITED_PACKETS;

    mip_cmd_queue_init(&device->_queue, base_reply_timeout);

    mip_dispatcher_init(&device->_dispatcher);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Sets an optional user data pointer which can be retrieved later.
///
///@param device
///@param pointer
///
void mip_interface_set_user_pointer(struct mip_interface* device, void* pointer)
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
void* mip_interface_user_pointer(const struct mip_interface* device)
{
    return device->_user_pointer;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the maximum number of packets to parser per update call.
///
unsigned int mip_interface_max_packets_per_update(const struct mip_interface* device)
{
    return device->_max_update_pkts;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a limit on the number of packets which can be processed in one
///       call to the mip_interface_receive_bytes() function.
///
/// Use this when receiving data in bursts to smooth out the processing
/// load over time.
///
///@note Make sure the parsing buffer is large enough to hold the
///      data in between receive calls.
///
///@note Make sure mip_interface_user_update() executes the parser even when no new
///      input data is available so that unparsed data is pushed through.
///
///@param device
///
///@param max_packets
///       Maximum number of packets to parse at once.
///
void mip_interface_set_max_packets_per_update(struct mip_interface* device, unsigned int max_packets)
{
    device->_max_update_pkts = max_packets;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Polls the port for new data. Called repeatedly while waiting for
///       acknowledgements to pending commands.
///
///@param device The mip_interface object.
///
///@returns true if operation should continue, or false if the device cannot be
///         updateed (e.g. if the serial port is not open)
///
/// Data from the port should be read and pushed into
/// mip_interface_receive_bytes() for parsing. On systems where it makes
/// sense, this is a good place to call sleep or enter a low-power state until
/// data arrives at the port. Typically this function will wait at most a few
/// milliseconds before returning.
///
/// This function is called in a loop, so returning true even when no bytes
/// have been received is always safe.
///
bool mip_interface_update(struct mip_interface* device)
{
    return mip_interface_user_update(device);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Sends data to the port (i.e. from this library to the physical device).
///
///@param device The mip_interface object.
///@param data   Data to be sent.
///@param length Length of data.
///
///@returns True if the data was sent successfully, false if some or all data
///         could not be sent.
///
/// This is called whenever bytes must be sent to the physical device.
///
bool mip_interface_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length)
{
    return mip_interface_user_send_to_device(device, data, length);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Receive data from the port (i.e. the physical device) into the parser.
///
/// Call this when data has been received. In some applications, this will be
/// from the mip_interface_user_update() function, while in others a dedicated
/// thread will manage receiving data.
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
///@returns The amount of data which couldn't be processed due to the limit on
///         number of packets per parse call. Normally the result is 0.
///
remaining_count mip_interface_receive_bytes(struct mip_interface* device, const uint8_t* data, size_t length, timestamp_type timestamp)
{
    return mip_parser_parse(&device->_parser, data, length, timestamp, device->_max_update_pkts);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Process more packets from the internal buffer.
///
/// This is an alternative to mip_pinterface_receive_bytes() for the case when
/// no new input data is available and max_packets is nonzero. The timestamp is
/// reused from the last call to receive_bytes.
///
/// This function obeys the max_packets_per_update setting.
///
///@note Calling this function when max_packets_per_update is zero is unnecessary
///      and has no effect.
///
void mip_interface_process_unparsed_packets(struct mip_interface* device)
{
    mip_parser_parse(&device->_parser, NULL, 0, device->_max_update_pkts, mip_parser_last_packet_timestamp(&device->_parser));
}

////////////////////////////////////////////////////////////////////////////////
///@brief Processes a pre-parsed packet for command replies and data.
///
///@param device
///
///@param packet
///       The received MIP packet.
///@param timestamp
///       timestamp_type of the received MIP packet.
///
void mip_interface_receive_packet(struct mip_interface* device, const struct mip_packet* packet, timestamp_type timestamp)
{
    mip_cmd_queue_process_packet(&device->_queue, packet, timestamp);
    mip_dispatcher_dispatch_packet(&device->_dispatcher, packet, timestamp);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the MIP parser for the device.
///
struct mip_parser* mip_interface_parser(struct mip_interface* device)
{
    return &device->_parser;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the commmand queue for the device.
///
struct mip_cmd_queue* mip_interface_cmd_queue(struct mip_interface* device)
{
    return &device->_queue;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Blocks until the pending command completes or times out.
///
///@param device
///@param cmd
///
///@returns The final status of the command.
///
enum mip_cmd_result mip_interface_wait_for_reply(struct mip_interface* device, const struct mip_pending_cmd* cmd)
{
    enum mip_cmd_result status;
    while( !mip_cmd_result_is_finished(status = mip_pending_cmd_status(cmd)) )
    {
        if( !mip_interface_update(device) )
            return MIP_STATUS_ERROR;
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
enum mip_cmd_result mip_interface_run_command(struct mip_interface* device, uint8_t descriptor_set, uint8_t cmd_descriptor, const uint8_t* cmd_data, uint8_t cmd_length)
{
    return mip_interface_run_command_with_response(device, descriptor_set, cmd_descriptor, cmd_data, cmd_length, MIP_INVALID_FIELD_DESCRIPTOR, NULL, NULL);
}

////////////////////////////////////////////////////////////////////////////////
///@copydoc mip_interface_run_command
///
///@param response_descriptor
///       Descriptor of the response data. May be MIP_INVALID_FIELD_DESCRIPTOR
///       if no response is expected.
///
enum mip_cmd_result mip_interface_run_command_with_response(struct mip_interface* device,
    uint8_t descriptor_set, uint8_t cmd_descriptor, const uint8_t* cmd_data, uint8_t cmd_length,
    uint8_t response_descriptor, uint8_t* response_buffer, uint8_t* response_length_inout)
{
    assert((response_descriptor == MIP_INVALID_FIELD_DESCRIPTOR) || ((response_buffer != NULL) && (response_length_inout != NULL)) );

    uint8_t buffer[MIP_PACKET_LENGTH_MAX];

    struct mip_packet packet;
    mip_packet_create(&packet, buffer, sizeof(buffer), descriptor_set);
    mip_packet_add_field(&packet, cmd_descriptor, cmd_data, cmd_length);
    mip_packet_finalize(&packet);

    struct mip_pending_cmd cmd;
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
enum mip_cmd_result mip_interface_run_command_packet(struct mip_interface* device, const struct mip_packet* packet, struct mip_pending_cmd* cmd)
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
bool mip_interface_start_command_packet(struct mip_interface* device, const struct mip_packet* packet, struct mip_pending_cmd* cmd)
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
    struct mip_interface* device, struct mip_dispatch_handler* handler,
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
    struct mip_interface* device, struct mip_dispatch_handler* handler,
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
    struct mip_interface* device, struct mip_dispatch_handler* handler,
    uint8_t descriptor_set, uint8_t field_descriptor,
    mip_dispatch_extractor extractor, void* field_ptr)
{
    mip_dispatch_handler_init_extractor(handler, descriptor_set, field_descriptor, extractor, field_ptr);
    mip_dispatcher_add_handler(&device->_dispatcher, handler);
}