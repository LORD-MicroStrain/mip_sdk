#include "mip/mip_interface.h"

#include "mip/mip_descriptors.h"
#include "mip/mip_field.h"

#include <assert.h>
#include <stdio.h>

#ifdef __cplusplus
using microstrain::C::microstrain_embedded_timestamp;

namespace mip {
namespace C {
#endif // __cplusplus

void mip_interface_init(mip_interface* _device, const mip_timeout _timeout, const mip_timeout _base_reply_timeout,
    const mip_send_callback _send_callback, const mip_recv_callback _receive_callback,
    const mip_update_callback _update_callback, void* _connection, void* _user_pointer)
{
    mip_parser_init(&_device->parser, &mip_interface_parse_callback, _device, _timeout);

    _device->connection      = _connection;
    _device->send_callback   = _send_callback;
    _device->recv_callback   = _receive_callback;
    _device->update_callback = _update_callback;
    _device->user_pointer    = _user_pointer;

    mip_cmd_queue_init(&_device->queue, _base_reply_timeout);

    mip_dispatcher_init(&_device->dispatcher);
}

bool mip_interface_send_to_device(const mip_interface* _device, const uint8_t* _data, const size_t _byte_count,
    size_t* _bytes_written_out)
{
    return _device->send_callback && _device->send_callback(_device, _data, _byte_count, _bytes_written_out);
}

bool mip_interface_recv_from_device(const mip_interface* _device, uint8_t* _data_out, const size_t _byte_count,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command)
{
    return _device->recv_callback &&
        _device->recv_callback(_device, _data_out, _byte_count, _wait_time, _bytes_read_out, _timestamp_out,
            _from_command);
}

bool mip_interface_update(mip_interface* _device, const mip_timeout _wait_time, const bool _from_command)
{
    return _device->update_callback && _device->update_callback(_device, _wait_time, _from_command);
}

bool mip_interface_default_update(mip_interface* _device, const mip_timeout _wait_time, const bool _from_command)
{
    // Allocate a buffer to hold received data.
    // Note: bigger buffers will parse faster, but may overflow the stack.
    // 512 bytes seems to be a good compromise on both desktop and embedded devices.
    uint8_t buffer[512];

    return mip_interface_default_update_ext_buffer(_device, buffer, sizeof(buffer), _wait_time, _from_command);
}

bool mip_interface_default_update_ext_buffer(mip_interface* _device, uint8_t* _buffer, const size_t _byte_count,
    const mip_timeout _wait_time, const bool _from_command)
{
    if (!_device->recv_callback)
    {
        return false;
    }

    size_t count = 0;
    mip_timestamp timestamp;

    // Try to read data from the port into a temporary buffer.
    if (!_device->recv_callback(_device, _buffer, _byte_count, (uint32_t)_wait_time, &count, &timestamp, _from_command))
    {
        return false;
    }

    mip_interface_input_bytes_andor_time(_device, _buffer, count, timestamp);

    return true;
}

void mip_interface_input_bytes_andor_time(mip_interface* _device, const uint8_t* _received_data,
    const size_t _data_length, const mip_timestamp _timestamp)
{
    // Pass the data to the MIP parser.
    mip_interface_input_bytes_from_device(_device, _received_data, _data_length, _timestamp);

    // Update the command queue to see if any commands have timed out.
    mip_interface_update_time(_device, _timestamp);
}

void mip_interface_input_bytes_from_device(mip_interface* _device, const uint8_t* _data, const size_t _length,
    const mip_timestamp _timestamp)
{
    mip_parser_parse(&_device->parser, _data, _length, _timestamp);
}

void mip_interface_input_packet_from_device(mip_interface* _device, const mip_packet_view* _packet_view,
    const mip_timestamp _timestamp)
{
    mip_cmd_queue_process_packet(&_device->queue, _packet_view, _timestamp);
    mip_dispatcher_dispatch_packet(&_device->dispatcher, _packet_view, _timestamp);
}

void mip_interface_update_time(mip_interface* _device, const mip_timestamp _timestamp)
{
    mip_cmd_queue_update(mip_interface_cmd_queue(_device), _timestamp);
}

bool mip_interface_parse_callback(void* _device, const mip_packet_view* _packet, const mip_timestamp _timestamp)
{
    mip_interface_input_packet_from_device(_device, _packet, _timestamp);

    return true;
}

mip_cmd_result mip_interface_wait_for_reply(mip_interface* _device, mip_pending_cmd* _pending_command)
{
    mip_cmd_result status;
    while (!mip_cmd_result_is_finished(status = mip_pending_cmd_status(_pending_command)))
    {
        if (!mip_interface_update(_device, 0, true))
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

            mip_cmd_queue_dequeue(mip_interface_cmd_queue(_device), _pending_command);
            _pending_command->_status = MIP_STATUS_ERROR;

            return MIP_STATUS_ERROR;
        }
    }

    return status;
}

mip_cmd_result mip_interface_run_command(mip_interface* _device, const uint8_t _descriptor_set,
    const uint8_t _field_descriptor, const uint8_t* _payload, const uint8_t _payload_length)
{
    return mip_interface_run_command_with_response(_device, _descriptor_set, _field_descriptor, _payload,
        _payload_length, MIP_INVALID_FIELD_DESCRIPTOR, NULL, NULL);
}

mip_cmd_result mip_interface_run_command_with_response(mip_interface* _device, const uint8_t _descriptor_set,
    const uint8_t _field_descriptor, const uint8_t* _payload, const uint8_t _payload_length,
    const uint8_t _response_descriptor, uint8_t* _response_data, uint8_t* _response_length_inout)
{
    assert(_response_descriptor == MIP_INVALID_FIELD_DESCRIPTOR ||
        ((_response_data != NULL) && (_response_length_inout != NULL)) );

    uint8_t buffer[MIP_PACKET_LENGTH_MAX];

    mip_packet_view packet;
    mip_packet_create(&packet, buffer, sizeof(buffer), _descriptor_set);
    mip_packet_add_field(&packet, _field_descriptor, _payload, _payload_length);
    mip_packet_finalize(&packet);

    mip_pending_cmd cmd;
    const uint8_t response_length = _response_length_inout ? *_response_length_inout : 0;
    mip_pending_cmd_init_with_response(&cmd, _descriptor_set, _field_descriptor, _response_descriptor, _response_data,
        response_length);

    const mip_cmd_result result = mip_interface_run_command_packet(_device, &packet, &cmd);

    if (_response_length_inout)
    {
        *_response_length_inout = mip_pending_cmd_response_length(&cmd);
    }

    return result;
}

mip_cmd_result mip_interface_run_command_packet(mip_interface* _device, const mip_packet_view* _packet_view,
    mip_pending_cmd* _pending_command)
{
    if (!mip_interface_start_command_packet(_device, _packet_view, _pending_command))
    {
        return MIP_STATUS_ERROR;
    }

    return mip_interface_wait_for_reply(_device, _pending_command);
}

bool mip_interface_start_command_packet(mip_interface* _device, const mip_packet_view* _packet_view,
    mip_pending_cmd* _pending_command)
{
    mip_cmd_queue_enqueue(mip_interface_cmd_queue(_device), _pending_command);

    size_t bytes_written;
    if (!mip_interface_send_to_device(_device, mip_packet_pointer(_packet_view),
        (size_t)mip_packet_total_length(_packet_view), &bytes_written))
    {
        mip_cmd_queue_dequeue(mip_interface_cmd_queue(_device), _pending_command);
        return false;
    }

    return true;
}

void mip_interface_register_packet_callback(mip_interface* _device, mip_dispatch_handler* _dispatch_handler,
    const uint8_t _descriptor_set, const bool _after_fields,
    const mip_dispatch_packet_callback _dispatch_packet_callback, void* _user_data)
{
    mip_dispatch_handler_init_packet_handler(_dispatch_handler, _descriptor_set, _after_fields,
        _dispatch_packet_callback, _user_data);
    mip_dispatcher_add_handler(&_device->dispatcher, _dispatch_handler);
}

void mip_interface_register_field_callback(mip_interface* _device, mip_dispatch_handler* _dispatch_handler,
    const uint8_t _descriptor_set, const uint8_t _field_descriptor,
    const mip_dispatch_field_callback _dispatch_field_callback, void* _user_data)
{
    mip_dispatch_handler_init_field_handler(_dispatch_handler, _descriptor_set, _field_descriptor,
        _dispatch_field_callback, _user_data);
    mip_dispatcher_add_handler(&_device->dispatcher, _dispatch_handler);
}

void mip_interface_register_extractor(mip_interface* _device, mip_dispatch_handler* _dispatch_handler,
    const uint8_t _descriptor_set, const uint8_t _field_descriptor, const mip_dispatch_extractor _dispatch_extractor,
    void* _field_data)
{
    mip_dispatch_handler_init_extractor(_dispatch_handler, _descriptor_set, _field_descriptor, _dispatch_extractor,
        _field_data);
    mip_dispatcher_add_handler(&_device->dispatcher, _dispatch_handler);
}

void mip_interface_set_recv_function(mip_interface* _device, const mip_recv_callback _receive_callback)
{
    _device->recv_callback = _receive_callback;
}

void mip_interface_set_send_function(mip_interface* _device, const mip_send_callback _send_callback)
{
    _device->send_callback = _send_callback;
}

void mip_interface_set_update_function(mip_interface* _device, const mip_update_callback _update_callback)
{
    _device->update_callback = _update_callback;
}

void mip_interface_set_user_pointer(mip_interface* _device, void* _user_data)
{
    _device->user_pointer = _user_data;
}

mip_recv_callback mip_interface_recv_function(const mip_interface* _device)
{
    return _device->recv_callback;
}

mip_send_callback mip_interface_send_function(const mip_interface* _device)
{
    return _device->send_callback;
}

mip_update_callback mip_interface_update_function(const mip_interface* _device)
{
    return _device->update_callback;
}

void* mip_interface_user_pointer(const mip_interface* _device)
{
    return _device->user_pointer;
}

void* mip_interface_connection_pointer(const mip_interface* _device)
{
    return _device->connection;
}

mip_parser* mip_interface_parser(mip_interface* _device)
{
    return &_device->parser;
}

mip_cmd_queue* mip_interface_cmd_queue(mip_interface* _device)
{
    return &_device->queue;
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
#endif // __cplusplus
