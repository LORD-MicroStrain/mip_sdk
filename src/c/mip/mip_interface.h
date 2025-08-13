#pragma once

#include "mip/mip_cmdqueue.h"
#include "mip/mip_dispatch.h"
#include "mip/mip_parser.h"

#include <stdint.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup mip_c
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @defgroup mip_interface_c  Mip Interface
///
/// @brief High-level %C functions for controlling a MIP device
///
/// This module contains functions and classes for communicating with a MIP
/// device in C
///
/// @li Sending commands
/// @li Receiving Data
///
/// @{
///

struct mip_interface;

////////////////////////////////////////////////////////////////////////////////
/// @brief Called from mip_interface_send_to_device() to send data to the device
///        connection
///
/// The application should forward the data to the device connection (e.g. a
/// serial port, TCP connection, etc)
///
/// @param _device A pointer to the device interface. Applications can use the
///                user data pointer to access additional information such as
///                the port handle
/// @param _data Buffer containing the data to be transmitted to the device
/// @param _byte_count Length of data to transmit
/// @param _bytes_written_out Number of actual bytes written
///
/// @return True if all the data was successfully transmitted.
/// @return False if an error occurred and some or all data was definitely
///         unable to be transmitted
/// @return Applications should prefer returning true if success is uncertain
///         since command timeouts will help detect failed transmissions. If
///         this function returns false, the associated command will fail with
///         CmdResult::STATUS_ERROR. Commands will time out in case of
///         undetected transmission failures, so false positives are OK. False
///         negatives will cause commands to immediately fail even if sent
///         successfully, however, and the unexpected ack/nack replies increase
///         the risk of confusing which replies go with which commands
///
typedef bool (*mip_send_callback)(const struct mip_interface* _device, const uint8_t* _data, const size_t _byte_count,
    size_t* _bytes_written_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Called from mip_interface_recv_from_device() to receive data from the
///       device port
///
/// This is called indirectly through mip_interface_update() to poll for new
/// data and command responses. For single-threaded applications, it will be
/// called while waiting for command replies
///
/// @param _device A pointer to the device interface. Applications can use the
///                user data pointer to access additional information such as
///                the port handle
/// @param _buffer Buffer to store received data
/// @param _buffer_size Max size of the buffer to store the data
/// @param _wait_time Time to wait for data from the device. The actual time
///                   waited may be less than wait_time, but it should not
///                   significantly exceed this value
/// @param _bytes_read_out Number of actual bytes read
/// @param _timestamp_out Timestamp of the data that was received
/// @param _from_command If true, this call is a result of waiting for a command
///                      to complete. Otherwise, this call is a regularly
///                      scheduled poll for data
///
/// @returns True if successful, even if no data is received.
/// @returns False if the port cannot be read or some other error occurs (e.g.
///          if the port is closed). If this function returns false when
///          _from_command is true, the corresponding command will fail with
///          MIP_STATUS_ERROR
///
/// @note Except in case of error (i.e. returning false), the timestamp must be
///       set even if no data is received. This is required to allow commands to
///       time out when no response is received
///
/// @note Applications may sleep the thread or enter a low-power state while
///       waiting for data. On posix-like (e.g. desktop) systems, applications
///       should call read() with a maximum timeout of wait_time. If the actual
///       wait time is less than the requested duration, this function may be
///       called again by the MIP SDK to wait the remaining time. If the actual
///       wait time exceeds wait_time, command timeouts may take longer than
///       intended
///
/// @see mip_interface_recv_from_device
///
#ifdef __cplusplus
typedef bool (*mip_recv_callback)(const struct mip_interface* _device, uint8_t* _buffer, const size_t _buffer_size,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain::C::microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command);
#else
typedef bool (*mip_recv_callback)(const struct mip_interface* _device, uint8_t* _buffer, const size_t _buffer_size,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command);
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
/// @brief Callback function typedef for custom update behavior
///
/// This function is called whenever data should be parsed from the port:
///     @li While waiting for command responses
///     @li To check for new data packets
///
/// Generally an application should call mip_interface_recv_from_device() from
/// within this callback and pass the data to
/// mip_interface_input_bytes_from_device(). Many applications can set this
/// callback to mip_interface_default_update()
///
/// @param _device The mip_interface object being updated
/// @param _wait_time Approximate amount of time to wait for data from the
///                   device. This may be an underestimate, but applications
///                   should not wait significantly longer as this may cause
///                   commands to take longer to time out
/// @param _from_command If true, this call is a result of waiting for a command
///                      to complete. Otherwise, this call is a regularly
///                      scheduled poll for data. Typically, an application will
///                      not wait for more data if this is false, or may sleep
///                      while waiting if this is true
///
/// @returns True if successful (even if no data is received)
/// @returns False if an error occurs and the port cannot be read (e.g. if the
///          port is closed). Returning false will cause any pending commands to
///          fail with a status error code
///
/// @note This update call should not return false (i.e. failure) when from_cmd
///       is true and reception is handled by another thread, unless that thread
///       is not currently running. Otherwise, a race condition may occur in
///       the command queue
///
typedef bool (*mip_update_callback)(struct mip_interface* _device, const mip_timeout _wait_time,
    const bool _from_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief State of the interface for communicating with a MIP device
///
typedef struct mip_interface
{
    /// @brief MIP Parser for incoming MIP packets
    mip_parser parser;

    /// @brief Queue for checking command replies
    mip_cmd_queue queue;

    /// @brief Dispatcher for data callbacks
    mip_dispatcher dispatcher;

    /// @brief Connection pointer
    void* connection;

    /// @brief Optional function which is called to send raw bytes to the device
    mip_send_callback send_callback;

    /// @brief Optional function which is called to receive raw bytes from the device
    mip_recv_callback recv_callback;

    /// @brief Optional function to call during updates
    mip_update_callback update_callback;

    /// @brief Optional user-specified data pointer
    void* user_pointer;
} mip_interface;


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize the mip_interface components
///
/// @param _device
/// @param _timeout Maximum length of time to wait for the end of a MIP packet.
///                 See mip_parser_init()
/// @param _base_reply_timeout Minimum time for all commands. See
///                            mip_cmd_queue_init()
/// @param _send_callback A callback which is called to send data to the device
/// @param _receive_callback A callback which is called when data needs to be
///                          read from the device
/// @param _update_callback Optional callback which is called to perform routine
///                         tasks such as checking for command timeouts.
///                         Defaults to mip_interface_default_update
/// @param _connection Pointer to the connection interface for the device
/// @param _user_pointer Optional pointer which is passed to the send, receive,
///                      and update callbacks
///
void mip_interface_init(mip_interface* _device, mip_timeout _timeout, mip_timeout _base_reply_timeout,
    mip_send_callback _send_callback, mip_recv_callback _receive_callback, mip_update_callback _update_callback,
    void* _connection, void* _user_pointer);

////////////////////////////////////////////////////////////////////////////////
/// @brief Sends data to the port (i.e. from this library to the physical
///        device)
///
/// @param _device The mip_interface object
/// @param _data Pointer to the data buffer containing bytes to transmit
/// @param _byte_count Number of bytes to write from the buffer
/// @param _bytes_written_out Output variable for the number of actual bytes
///                           written to the port
///
/// @returns True if the data was sent successfully
/// @returns False if the send callback is NULL
/// @returns False if some or all data could not be sent
///
/// This is called whenever bytes must be sent to the physical device
///
bool mip_interface_send_to_device(const mip_interface* _device, const uint8_t* _data, const size_t _byte_count,
    size_t* _bytes_written_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks for data at the port and reads it into the buffer
///
/// @param _device
/// @param _data_out A place to store the data.
/// @param _byte_count Maximum number of bytes to read into the buffer
/// @param _wait_time Maximum time to wait for data. Can be 0
/// @param _bytes_read_out The number of bytes successfully read into the buffer
/// @param _timestamp_out The timestamp of the received data.
/// @param _from_command If true, this call is a result of waiting for a command
///                      to complete. Otherwise, this call is a regularly
///                      scheduled poll for data. User code calling this
///                      function should generally set this to false.
///
/// @returns True if successful (even if 0 bytes were read)
/// @returns False if the receive callback is NULL
/// @returns False if the receive callback failed (i.e. if it returned false)
///
#ifdef __cplusplus
bool mip_interface_recv_from_device(const mip_interface* _device, uint8_t* _data_out, const size_t _byte_count,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain::C::microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command);
#else
bool mip_interface_recv_from_device(const mip_interface* _device, uint8_t* _data_out, const size_t _byte_count,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command);
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
/// @brief Call to process data from the device
///
/// This function is also called while waiting for command replies
///
/// Call this periodically to process packets received from the device. It
/// should be called at a suitably high rate to prevent the connection buffers
/// from overflowing. The update rate affects the reception timestamp resolution
///
/// @param _device
/// @param _wait_time Time to wait for data from the device. This will be
///                   nonzero when waiting for command replies. Applications
///                   calling this function can pass 0 to avoid blocking when
///                   checking for new data
/// @param _from_command If true, this call is a result of waiting for a command
///                      to complete. Otherwise, this call is a regularly
///                      scheduled poll for data. User code calling this
///                      function should generally set this to false
///
/// @returns true if operation should continue, or false if the device cannot be
///          updated (e.g. if the serial port is not open)
///
bool mip_interface_update(mip_interface* _device, const mip_timeout _wait_time, const bool _from_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief Polls the port for new data or command replies
///
/// This is the default choice for the user update function. It ignores the
/// from_cmd flag and always tries to read data from the device
///
/// @warning This function is provided for convenience and quick setup. It will
///          work for most applications, but it is not optimized for your
///          application. It may be unsuitable for some situations, such as
///          small microcontrollers (due to the fixed-size stack-allocated data
///          buffer) or in combination with multi-threading
///
/// @param _device
/// @param _wait_time Time to wait for data from the device. This will be
///                   nonzero when waiting for command replies. Applications
///                   calling this function can pass 0 to avoid blocking when
///                   checking for new data
/// @param _from_command If true, this call is a result of waiting for a command
///                      to complete. Otherwise, this call is a regularly
///                      scheduled poll for data. User code calling this
///                      function should generally set this to false
///
/// @returns The value returned by mip_interface_user_recv_from_device
///
bool mip_interface_default_update(mip_interface* _device, const mip_timeout _wait_time, const bool _from_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief Polls the port for new data or command replies using a supplied
///        buffer
///
/// @details This function is suitable for most single-threaded use cases where
///          performance is not critical. In performance-sensitive applications,
///          it's best to either read directly into the mip parser buffer or
///          call mip_interface_input_bytes from your own update function
///          (bypassing mip_interface_recv_from_device entirely)
///
/// @param _device
///
/// @param _buffer
///       Buffer to hold data read from the device connection. At least 512
///       bytes are recommended for better performance, with a few kB being
///       the point of diminishing return.
///
/// @param _byte_count
///       Size of the buffer.
///
/// @param _wait_time
///       Time to wait for data from the device. This will be nonzero when
///       waiting for command replies. Applications calling this function
///       can pass 0 to avoid blocking when checking for new data.
///
/// @param _from_command
///       If true, this call is a result of waiting for a command to complete.
///       Otherwise, this call is a regularly-scheduled poll for data. User
///       code calling this function should generally set this to false.
///
/// @returns The value returned by mip_interface_user_recv_from_device.
///
bool mip_interface_default_update_ext_buffer(mip_interface* _device, uint8_t* _buffer, const size_t _byte_count,
    const mip_timeout _wait_time, const bool _from_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief This function takes care of processing received data and updating the
///        current time
///
/// User-defined update functions should either call this function after
/// getting data from the connection, or do the equivalent manually:
///     @li Input bytes to the parser, via mip_interface_input_bytes_from_device
///     @li Update the current time via mip_interface_update_time
///
void mip_interface_input_bytes_andor_time(mip_interface* _device, const uint8_t* _received_data, size_t _data_length,
    mip_timestamp _timestamp);

////////////////////////////////////////////////////////////////////////////////
///@brief Passes data from the device into the parser.
///
/// @param _device
/// @param _data Input data buffer. May be NULL if length == 0
/// @param _length Length of the input buffer. Must be 0 if data is NULL
/// @param _timestamp Time of the received data
///
void mip_interface_input_bytes_from_device(mip_interface* _device, const uint8_t* _data, size_t _length,
    mip_timestamp _timestamp);

////////////////////////////////////////////////////////////////////////////////
/// @brief Processes a pre-parsed packet for command replies and data
///
/// @param _device
/// @param _packet_view The received MIP packet
/// @param _timestamp Timestamp of the received MIP packet
///
void mip_interface_input_packet_from_device(mip_interface* _device, const mip_packet_view* _packet_view,
    mip_timestamp _timestamp);

////////////////////////////////////////////////////////////////////////////////
/// @brief Call this to ensure that pending commands time out properly
///
/// @details This function should be called regularly (i.e. from within the
///          update callback). Otherwise, unacknowledged commands may never time
///          out. This can happen even if data is being sent through
///          mip_interface_input_bytes_from_device because the device may be
///          streaming data. See the implementation of
///          mip_interface_default_update for an example
///
/// @param _device
/// @param _timestamp Current time, as if timestamping received data
///
void mip_interface_update_time(mip_interface* _device, mip_timestamp _timestamp);

////////////////////////////////////////////////////////////////////////////////
/// @brief Wrapper around mip_interface_input_packet_from_device for use with
///        mip_parser
///
/// @param _device Void pointer to the device. Must be a mip_interface pointer
/// @param _packet MIP Packet from the parser
/// @param _timestamp Timestamp of the packet
///
void mip_interface_parse_callback(void* _device, const mip_packet_view* _packet, mip_timestamp _timestamp);

////////////////////////////////////////////////////////////////////////////////
/// @brief Blocks until the pending command completes or times out
///
/// @param _device
/// @param _pending_command
///
/// @returns The final status of the command.
///
mip_cmd_result mip_interface_wait_for_reply(mip_interface* _device, mip_pending_cmd* _pending_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief Runs a command using a pre-serialized payload
///
/// @param _device
/// @param _descriptor_set Command descriptor set
/// @param _field_descriptor Command field descriptor
/// @param _payload Optional payload data. May be NULL if cmd_length == 0
/// @param _payload_length Length of the command payload (parameters)
///
/// @return mip_cmd_result
///         @li MIP_ACK_OK - Command completed successfully
///         @li MIP_NACK_* - Device rejected the command
///         @li MIP_STATUS_* - An error occurred (e.g. timeout)
///
mip_cmd_result mip_interface_run_command(mip_interface* _device, uint8_t _descriptor_set, uint8_t _field_descriptor,
    const uint8_t* _payload, uint8_t _payload_length);

////////////////////////////////////////////////////////////////////////////////
/// @brief Runs a command using a pre-serialized payload
///
/// @param _device
/// @param _descriptor_set Command descriptor set
/// @param _field_descriptor Command field descriptor
/// @param _payload Optional payload data. May be NULL if cmd_length == 0
/// @param _payload_length Length of the command payload (parameters)
/// @param _response_descriptor Descriptor of the response data. Can be
///                             MIP_INVALID_FIELD_DESCRIPTOR if no response is
///                             expected
/// @param _response_data Buffer to hold response data. Can be the same as the
///                       command data buffer. Can be NULL if
///                       _response_descriptor is MIP_INVALID_FIELD_DESCRIPTOR
/// @param[in,out] _response_length_inout As input, the size of response buffer
///                                       and max response length. As output,
///                                       returns the actual length of the
///                                       response data
///
/// @returns mip_cmd_result
///
mip_cmd_result mip_interface_run_command_with_response(mip_interface* _device, uint8_t _descriptor_set,
    uint8_t _field_descriptor, const uint8_t* _payload, uint8_t _payload_length, uint8_t _response_descriptor,
    uint8_t* _response_data, uint8_t* _response_length_inout);

////////////////////////////////////////////////////////////////////////////////
/// @brief Similar to mip_interface_start_command_packet but waits for the
///        command to complete
///
/// @param _device
/// @param _packet_view A MIP packet containing the command
/// @param _pending_command The command status tracker. No lifetime requirement
///
mip_cmd_result mip_interface_run_command_packet(mip_interface* _device, const mip_packet_view* _packet_view,
    mip_pending_cmd* _pending_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief Queues the command and sends the packet. Does not wait for completion
///
/// @param _device
/// @param _packet_view A MIP packet containing the command
/// @param _pending_command The command status tracker. Must be valid while the
///                         command executes
///
/// @returns True if successful. Cmd must remain valid until the command
///          finishes.
/// @returns False on error sending the packet. No cleanup is necessary and cmd
///          can be destroyed immediately afterward in this case
///
bool mip_interface_start_command_packet(mip_interface* _device, const mip_packet_view* _packet_view,
    mip_pending_cmd* _pending_command);

////////////////////////////////////////////////////////////////////////////////
/// @brief Registers a callback for packets of the specified descriptor set
///
/// @param _device
/// @param _dispatch_handler An uninitialized mip_dispatch_handler object. This
///                          call will initialize it
/// @param _descriptor_set
/// @param _after_fields
/// @param _dispatch_packet_callback
/// @param _user_data
///
/// @see mip_dispatch_handler_init_packet_handler for details.
///
void mip_interface_register_packet_callback(mip_interface* _device, mip_dispatch_handler* _dispatch_handler,
    uint8_t _descriptor_set, bool _after_fields, mip_dispatch_packet_callback _dispatch_packet_callback,
    void* _user_data);

////////////////////////////////////////////////////////////////////////////////
/// @brief Registers a callback for packets of the specified descriptor set
///
/// @param _device
/// @param _dispatch_handler An uninitialized mip_dispatch_handler object. This
///                          call will initialize it
/// @param _descriptor_set
/// @param _field_descriptor
/// @param _dispatch_field_callback
/// @param _user_data
///
/// @see mip_dispatch_handler_init_field_handler for details
///
void mip_interface_register_field_callback(mip_interface* _device, mip_dispatch_handler* _dispatch_handler,
    uint8_t _descriptor_set, uint8_t _field_descriptor, mip_dispatch_field_callback _dispatch_field_callback,
    void* _user_data);

////////////////////////////////////////////////////////////////////////////////
/// @brief Registers a callback for packets of the specified descriptor set.
///
/// @param _device
/// @param _dispatch_handler An uninitialized mip_dispatch_handler object. This
///                          call will initialize it
/// @param _descriptor_set
/// @param _field_descriptor
/// @param _dispatch_extractor
/// @param _field_data
///
/// @see mip_dispatch_handler_init_extract_handler for details
///
void mip_interface_register_extractor(mip_interface* _device, mip_dispatch_handler* _dispatch_handler,
    uint8_t _descriptor_set, uint8_t _field_descriptor, mip_dispatch_extractor _dispatch_extractor, void* _field_data);

////////////////////////////////////////////////////////////////////////////////
/// @brief Sets the receive callback function
///
/// @param _device
/// @param _receive_callback Function which gets data from the device
///                          connection. If this is NULL, then commands will
///                          fail and no data will be received
///
void mip_interface_set_recv_function(mip_interface* _device, mip_recv_callback _receive_callback);

////////////////////////////////////////////////////////////////////////////////
/// @brief Sets the send callback function
///
/// @param _device
/// @param _send_callback Function which sends raw bytes to the device. This can
///                       be NULL if no commands are issued (they would fail)
///
void mip_interface_set_send_function(mip_interface* _device, mip_send_callback _send_callback);

////////////////////////////////////////////////////////////////////////////////
/// @brief Sets the update function
///
/// By default, the update function is mip_interface_default_update
///
/// @see mip_update_callback
/// @see mip_interface_update
///
/// @param _device
/// @param _update_callback Update function to call when polling the device for
///                         data. If this is NULL, then update calls will fail
///                         and no data or command replies will be received.
///
void mip_interface_set_update_function(mip_interface* _device, mip_update_callback _update_callback);

////////////////////////////////////////////////////////////////////////////////
/// @brief Sets an optional user data pointer which can be retrieved later
///
/// @param _device
/// @param _user_data
///
void mip_interface_set_user_pointer(mip_interface* _device, void* _user_data);

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the receive function pointer
///
/// @param _device
///
/// @returns The receive callback function. Can be NULL
///
mip_recv_callback mip_interface_recv_function(const mip_interface* _device);

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the send function pointer
///
/// @param _device
///
/// @returns The send callback function. Can be NULL
///
mip_send_callback mip_interface_send_function(const mip_interface* _device);

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the update function pointer
///
/// @returns The update function. Defaults to mip_interface_default_update. Can
///          be NULL
///
mip_update_callback mip_interface_update_function(const mip_interface* _device);

////////////////////////////////////////////////////////////////////////////////
/// @brief Retrieves the pointer set by mip_interface_set_user_pointer()
///
/// @param _device
///
/// @returns The pointer value
///
void* mip_interface_user_pointer(const mip_interface* _device);

////////////////////////////////////////////////////////////////////////////////
/// @brief Retrieves the pointer set by mip_interface_set_connection_pointer()
///
/// @param _device
///
/// @returns The pointer value
///
void* mip_interface_connection_pointer(const mip_interface* _device);

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the MIP parser for the device
///
mip_parser* mip_interface_parser(mip_interface* _device);

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the command queue for the device
///
mip_cmd_queue* mip_interface_cmd_queue(mip_interface* _device);

///
/// @}
////////////////////////////////////////////////////////////////////////////////
///
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus
