#pragma once

#include <microstrain/embedded_time.h>
#include <microstrain/platform.h>
#include <microstrain/connections/recording/recording_connection.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
#include <winsock2.h>
#endif // MICROSTRAIN_PLATFORM_WINDOWS

#ifdef __cplusplus
namespace microstrain {
namespace C {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup microstrain_platform
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @defgroup microstrain_tcp TCP Client
///
/// @brief A simple implementation for reading and writing to/from a TCP client
///
/// @{
///

#if defined MICROSTRAIN_PLATFORM_WINDOWS
typedef SOCKET socket_handle_t;
#else // MICROSTRAIN_PLATFORM_UNIX
typedef int socket_handle_t;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

typedef struct tcp_client
{
    /// @brief System handle for the TCP client socket connection
    socket_handle_t handle;

    /// @brief Hostname or IP address of the remote server to connect to
    const char* hostname;

    /// @brief Port number on the remote server for the connection
    uint16_t port;

    /// @brief Connection timeout in milliseconds for socket operations
    uint32_t timeout;

    /// @brief Optional data recording functionality
    recording_connection* recording_connection;
} tcp_client;

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes a TCP client structure
///
/// @details Sets up the TCP client structure with the specified hostname and
///          port. Initializes the socket handle to an invalid state and
///          prepares the recording connection for optional data logging.
///
/// @param _tcp_client Pointer to the TCP client structure to initialize
/// @param _hostname Hostname or IP address of the remote server to connect to
/// @param _port Port number on the remote server to connect to
/// @param _recording_connection Optional connection pointer for recording
///                              incoming and/or outdoing data
///
/// @note The socket is not opened by this function - use tcp_client_open() to
///       establish the connection
///
void tcp_client_init(tcp_client* _tcp_client, const char* _hostname, const uint16_t _port,
    recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Opens the TCP client connection
///
/// @details Establishes a TCP connection to the configured hostname and port
///          with the specified timeout. Performs DNS resolution if needed and
///          sets up the socket for data transfer.
///
/// @param _tcp_client Pointer to the initialized TCP client structure
/// @param _timeout_ms Connection timeout in milliseconds
///
/// @return true if the connection was successfully established, false on failure
///
/// @note The TCP client must be initialized before calling this function
/// @note On failure, check network connectivity and server availability
///
bool tcp_client_open(tcp_client* _tcp_client, const uint32_t _timeout_ms);

////////////////////////////////////////////////////////////////////////////////
/// @brief Reopens the TCP client with new parameters
///
/// @details Closes the current connection (if open) and reopens the TCP client
///          with the specified hostname, port, and timeout. This is useful for
///          changing connection parameters without recreating the structure.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _hostname New hostname or IP address to connect to
/// @param _port New port number to connect to
/// @param _timeout_ms Connection timeout in milliseconds
///
/// @return true if the connection was successfully reopened, false on failure
///
/// @note Any existing connection will be closed before attempting to reopen
///
bool tcp_client_reopen(tcp_client* _tcp_client, const char* _hostname, const uint16_t _port,
    const uint32_t _timeout_ms);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes the TCP client connection
///
/// @details Safely closes the TCP connection and releases system resources. The
///          client structure remains valid and can be reopened if needed.
///
/// @param _tcp_client Pointer to the TCP client structure
///
/// @return true if the connection was successfully closed, false on failure
///
/// @note It is safe to call this function on an already closed connection
/// @note Recording connections are not affected by this operation
///
bool tcp_client_close(tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if the TCP client is currently connected
///
/// @details Returns the current connection status of the TCP client without
///          affecting the connection state.
///
/// @param _tcp_client Pointer to the TCP client structure
///
/// @return true if the client is connected and ready for communication, false
///         otherwise
///
bool tcp_client_is_open(const tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Reads data from the TCP connection
///
/// @details Attempts to read the specified number of bytes from the TCP
///          connection within the given timeout period. Returns the actual
///          number of bytes read and provides a timestamp of the read operation.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _buffer Buffer to store the received data
/// @param _byte_count Maximum number of bytes to read
/// @param _wait_time Maximum time to wait for data in milliseconds
/// @param _bytes_read_out Pointer to store the actual number of bytes read
/// @param _timestamp_out Pointer to store the timestamp of the read operation
///
/// @return true if the read operation was successful, false on failure
///
/// @note The function may return fewer bytes than requested
/// @note If recording is enabled, received data will be automatically logged
///
bool tcp_client_read(tcp_client* _tcp_client, uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time,
    size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Writes data to the TCP connection
///
/// @details Sends the specified data buffer to the connected server through the
///          TCP connection. Returns the actual number of bytes written.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _buffer Buffer containing the data to send
/// @param _byte_count Number of bytes to write from the buffer
/// @param _bytes_written_out Pointer to store the actual number of bytes
///                           written
///
/// @return true if the write operation was successful, false on failure
///
/// @note The function may write fewer bytes than requested
/// @note If recording is enabled, sent data will be automatically logged
///
bool tcp_client_write(tcp_client* _tcp_client, const uint8_t* _buffer, const size_t _byte_count,
    size_t* _bytes_written_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Updates the port number of the TCP client
///
/// @details Changes the target port number for future connections. The client
///          must be reopened for this change to take effect.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _port New port number to connect to
///
/// @return true if the port was successfully updated, false on failure
///
/// @note This only updates the stored port number; the connection must be
///       reopened to use the new port
///
bool tcp_client_update_port(tcp_client* _tcp_client, const uint16_t _port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes receive recording with a pre-opened stream
///
/// @details Sets up the TCP client to record all received data to the
///          specified file stream. The stream must be opened in binary
///          write mode by the caller.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _receive_stream Pre-opened file stream for recording received data
///
/// @note The caller is responsible for managing the lifecycle of the stream
///
void tcp_client_init_receive_recording_stream(const tcp_client* _tcp_client, FILE* _receive_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording of received data to a file
///
/// @details Configures the TCP client to automatically record all data
///          received from the remote connection to the specified file. This
///          feature is useful for debugging, protocol analysis, and data
///          logging. The recording operates transparently alongside normal read
///          operations.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _receive_file_name Path to the file where received data will be
///                           recorded. The file will be created if it does not
///                           exist, or truncated if it already exists
///
/// @note Recording begins immediately and continues until the socket is closed
///       or recording is explicitly disabled
///
/// @warning Ensure sufficient disk space is available as continuous logging can
///          generate large files over time
///
void tcp_client_init_receive_recording_file(const tcp_client* _tcp_client, const char* _receive_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes the receive recording stream for a TCP client
///
/// @details Stops recording data received by this TCP client to any open
///          recording streams.
///
/// @param _tcp_client Pointer to the TCP client structure
///
void tcp_client_close_receive_recording_stream(const tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if receive recording is enabled for a TCP client
///
/// @details Determines whether data received by this TCP client is being
///          recorded.
///
/// @param _tcp_client Pointer to the TCP client structure
///
/// @return True if receive recording is enabled, false otherwise
///
bool tcp_client_receive_recording_enabled(const tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes send recording with a pre-opened stream
///
/// @details Sets up the TCP client to record all sent data to the
///          specified file stream. The stream must be opened in binary
///          write mode by the caller.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _send_stream Pre-opened file stream for recording sent data
///
/// @note The caller is responsible for managing the lifecycle of the stream
///
void tcp_client_init_send_recording_stream(const tcp_client* _tcp_client, FILE* _send_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording of transmitted data to a file
///
/// @details Configures the TCP client to automatically record all data
///          transmitted to the remote connection to the specified file. This
///          feature enables monitoring of outgoing communication for debugging
///          and protocol verification purposes. The recording operates
///          transparently alongside normal write operations.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _send_file_name Path to the file where transmitted data will be
///                        recorded. The file will be created if it does not
///                        exist, or truncated if it already exists
///
/// @note Recording begins immediately and continues until the socket is closed
///       or recording is explicitly disabled
///
/// @warning Continuous transmission recording can consume significant disk
///          space depending on data volume and transmission frequency
///
void tcp_client_init_send_recording_file(const tcp_client* _tcp_client, const char* _send_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes the send recording stream for a TCP client
///
/// @details Stops recording data sent by this TCP client to any open recording
///          streams.
///
/// @param _tcp_client Pointer to the TCP client structure
///
void tcp_client_close_send_recording_stream(const tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if send recording is enabled for a TCP client
///
/// @details Determines whether data sent by this TCP client is being recorded.
///
/// @param _tcp_client Pointer to the TCP client structure
///
/// @return True if send recording is enabled, false otherwise
///
bool tcp_client_send_recording_enabled(const tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording with pre-opened streams for both directions
///
/// @details Convenience function that sets up recording for both received
///          and sent data using existing file streams. Both streams must
///          be opened in binary write mode by the caller.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _receive_stream Pre-opened file stream for recording received data
/// @param _send_stream Pre-opened file stream for recording sent data
///
/// @note Either stream parameter can be NULL to disable recording for that direction
/// @note The caller is responsible for managing the lifecycle of the streams
///
void tcp_client_init_recording_streams(const tcp_client* _tcp_client, FILE* _receive_stream, FILE* _send_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes bidirectional recording of serial communication
///
/// @details Configures the serial port to automatically record both received
///          and transmitted data to separate files. This provides complete
///          communication logging for debugging, protocol analysis, and
///          compliance recording. Both recording streams operate independently
///          and transparently with normal I/O operations.
///
/// @param _tcp_client Pointer to the TCP client structure
/// @param _receive_file_name Path to the file for recording received data.
///                           Created or truncated as needed
/// @param _send_file_name Path to the file for recording transmitted data.
///                        Created or truncated as needed
///
/// @note Both recording streams begin immediately and continue until the socket
///       is closed or recording is explicitly disabled
///
/// @note The receive and send files are independent and can be analyzed
///       separately or combined for complete communication reconstruction
///
/// @warning Bidirectional recording doubles the disk space requirements
///          compared to single-direction recording
///
void tcp_client_init_recording_files(const tcp_client* _tcp_client, const char* _receive_file_name,
    const char* _send_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes both receive and send recording streams for a TCP client
///
/// @details Stops recording data received or sent by this TCP client to any
///          open recording streams.
///
/// @param _tcp_client Pointer to the TCP client structure
///
void tcp_client_close_recording_streams(const tcp_client* _tcp_client);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if either receive or send recording is enabled for a TCP
///        client
///
/// @details Determines whether data received or sent by this TCP client is
///          being recorded.
///
/// @param _tcp_client Pointer to the TCP client structure
///
/// @return True if any form of recording (receive or send) is enabled, false
///         otherwise
///
bool tcp_client_recording_enabled(const tcp_client* _tcp_client);

///
/// @}
////////////////////////////////////////////////////////////////////////////////
///
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace microstrain
#endif // __cplusplus
