#pragma once

#include <microstrain/embedded_time.h>
#include <microstrain/platform.h>
#include <microstrain/connections/recording/recording_connection.h>

#if defined MICROSTRAIN_PLATFORM_WINDOWS
typedef void* HANDLE;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

#include <stdbool.h>
#include <stdint.h>

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
/// @defgroup microstrain_serial Serial Port
///
/// @brief A simple implementation for reading and writing to/from a serial port
///
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @brief Platform-specific handle type for system resources
///
/// @details Abstract handle type that represents system-specific identifiers
///          for resources such as file descriptors, device handles, or other
///          operating system objects. The actual underlying type varies by
///          platform but provides a unified interface for resource management.
///
#if defined MICROSTRAIN_PLATFORM_WINDOWS
typedef HANDLE serial_handle_t;
#else // Unix
typedef int serial_handle_t;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

////////////////////////////////////////////////////////////////////////////////
/// @brief Serial port communication structure
///
/// @details Represents a serial port connection with configuration parameters
///          and system handle. Supports cross-platform serial communication
///          with optional data recording capabilities for debugging and
///          protocol analysis.
///
typedef struct serial_port
{
    /// @brief System handle for the serial port connection
    serial_handle_t handle;

    /// @brief Name/path of the serial port device
    const char* port_name;

    /// @brief Communication speed in bits per second
    uint32_t baudrate;

    /// @brief Optional data recording functionality
    recording_connection* recording_connection;
} serial_port;

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes a serial port structure with the specified parameters
///
/// @details Sets up the serial port structure with the provided port name and
///          baudrate. The handle is initialized to an invalid state, and the
///          port must be opened using serial_port_open() before use.
///
/// @param _serial_port Pointer to the serial port structure to initialize
/// @param _port_name Name of the serial port to connect to
/// @param _baudrate Communication speed in bits per second
/// @param _recording_connection Optional connection pointer for recording
///                              incoming and/or outdoing data
///
/// @note This function does not open the port or validate the parameters. Call
///       serial_port_open() after initialization to establish the connection
///
void serial_port_init(serial_port* _serial_port, const char* _port_name, const uint32_t _baudrate,
    recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Opens the serial port for communication
///
/// @details Establishes a connection to the serial port and configures
///          communication parameters including timeouts, parity, stop bits,
///          and flow control for reliable communication. The port is configured
///          with 8 data bits, no parity, and 1 stop bit.
///
/// @param _serial_port Pointer to an initialized serial port structure
///
/// @return True if the port was successfully opened and configured, false if
///         the operation failed due to invalid parameters, port unavailability,
///         or configuration errors
///
/// @note Ensure the port is not already in use by another application before
///       calling this function
///
bool serial_port_open(serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes and reopens the serial port with new configuration parameters
///
/// @details Safely transitions the serial port to new settings by closing the
///          current connection and reopening it with the specified port name
///          and baudrate. This operation is atomic - if reopening fails, the
///          port remains closed. This function is useful for switching to a
///          different port or changing communication parameters that require
///          a full reconnection.
///
/// @param _serial_port Pointer to the serial port structure to reopen
/// @param _port_name New name of the serial port to connect to
/// @param _baudrate New communication speed in bits per second
///
/// @return True if the port was successfully closed and reopened with the new
///         parameters, false if the close operation failed, the new parameters
///         are invalid, or the reopen operation failed
///
/// @note If this function fails, the port will be left in a closed state and
///       must be reopened manually using serial_port_open() after correcting
///       the configuration parameters
///
/// @warning Any ongoing communication will be interrupted. Ensure all pending
///          operations are completed before calling this function
///
bool serial_port_reopen(serial_port* _serial_port, const char* _port_name, const uint32_t _baudrate);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes the serial port connection and releases system resources
///
/// @details Terminates the serial communication and frees system handles.
///          Resets the internal handle to invalid state and clears port
///          configuration parameters.
///
/// @param _serial_port Pointer to the serial port structure to close
///
/// @return True, if the port was successfully closed or was already closed,
///         false should not occur under normal circumstances
///
/// @note It is safe to call this function multiple times or on an already
///       closed port. After closing, the serial_port structure requires
///       reinitialization with serial_port_init() or serial_port_reopen()
///       before reuse
///
bool serial_port_close(serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if the serial port is currently open and operational
///
/// @details Verifies that the serial port handle is valid and the connection is
///          established. This is a simple handle validation that does not
///          perform actual communication testing.
///
/// @param _serial_port Pointer to the serial port structure to check
///
/// @return True if the port handle is valid and the port is considered open,
///         false if the port is closed or the handle is invalid
///
/// @note This function only checks handle validity and does not verify that the
///       physical device is still connected or responsive
///
bool serial_port_is_open(const serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Reads data from the serial port with timeout support
///
/// @details Attempts to read data from the serial port into the provided buffer
///          with configurable timeout. Automatically records received data if
///          receive recording is enabled and provides timestamp information for
///          the read operation.
///
/// @param _serial_port Pointer to an open serial port structure
/// @param _buffer Pointer to buffer where received data will be stored
/// @param _byte_count Maximum number of bytes to read, typically the size of
///                     the receive buffer
/// @param _wait_time Maximum time to wait for data in milliseconds. A value of
///                   0 performs a non-blocking read
/// @param _bytes_read_out Output variable for the number of actual bytes read
///                        from the port
/// @param _timestamp_out Pointer to timestamp structure that receives the time
///                       when the data was read
///
/// @return True if the read operation completed successfully (may read 0
///         bytes), false if the port is not open, a critical error occurred, or
///         the device was disconnected
///
bool serial_port_read(serial_port* _serial_port, uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time,
    size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Writes data to the serial port
///
/// @details Transmits the specified number of bytes from the buffer to the
///          serial port. Automatically records transmitted data if send
///          recording is enabled.
///
/// @param _serial_port Pointer to an open serial port structure
/// @param _buffer Pointer to the data buffer containing bytes to transmit
/// @param _byte_count Number of bytes to write from the buffer
/// @param _bytes_written_out Output variable for the number of actual bytes
///                           written to the port
///
/// @return True if all requested bytes were successfully written, false if the
///         port is not open, write operation failed, or not all bytes were
///         transmitted
///
/// @note The function ensures all requested bytes are written before returning
///       success. Partial writes are considered failures
///
/// @warning Ensure the buffer contains at least _byte_count valid bytes to
///          prevent memory access violations
///
bool serial_port_write(serial_port* _serial_port, const uint8_t* _buffer, const size_t _byte_count,
    size_t* _bytes_written_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Updates the baudrate of an open serial port connection
///
/// @details Modifies the communication speed of an already established serial
///          connection without closing and reopening the port. The internal
///          baudrate value is updated upon successful configuration.
///
/// @param _serial_port Pointer to an open serial port structure
/// @param _baudrate New communication speed in bits per second. Must be
///                  supported by the system and hardware
///
/// @return True if the baudrate was successfully updated, false if the port is
///         not open, the baudrate is unsupported, or the configuration
///         operation failed
///
bool serial_port_update_baudrate(serial_port* _serial_port, const uint32_t _baudrate);

////////////////////////////////////////////////////////////////////////////////
/// @brief Queries the number of bytes available for reading
///
/// @details Determines how many bytes are currently buffered and available for
///          immediate reading without blocking.
///
/// @param _serial_port Pointer to an open serial port structure
///
/// @return Number of bytes available for reading, or 0 if no data is available
///         or the port is not open
///
/// @note This function provides an estimate, and the actual number of bytes
///       available may change between the query and subsequent read operations
///       due to concurrent data reception
///
uint32_t serial_port_read_count(serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes receive recording with a pre-opened stream
///
/// @details Sets up the serial port to record all received data to the
///          specified file stream. The caller must open the stream in binary
///          write mode.
///
/// @param _serial_port Pointer to the serial port structure
/// @param _receive_stream Pre-opened file stream for recording received data
///
/// @note The caller is responsible for managing the lifecycle of the stream
///
void serial_port_init_receive_recording_stream(const serial_port* _serial_port, FILE* _receive_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording of received data to a file
///
/// @details Configures the serial port to automatically record all data
///          received from the remote device to the specified file. This feature
///          is useful for debugging, protocol analysis, and data logging. The
///          recording operates transparently alongside normal read operations.
///
/// @param _serial_port Pointer to the serial port structure
/// @param _receive_file_name Path to the file where received data will be
///                           recorded. The file will be created if it does not
///                           exist, or truncated if it already exists
///
/// @note Recording begins immediately and continues until the port is closed or
///       recording is explicitly disabled
///
/// @warning Ensure sufficient disk space is available as continuous logging can
///          generate large files over time
///
void serial_port_init_receive_recording_file(const serial_port* _serial_port, const char* _receive_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes receive recording stream on a serial port
///
/// @details Closes and cleans up the receive recording stream.
///
/// @param _serial_port Pointer to the serial port structure
///
void serial_port_close_receive_recording_stream(const serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if receive recording is enabled for a serial port
///
/// @details Determines whether data reception is being recorded on the serial
///          port.
///
/// @param _serial_port Pointer to the serial port structure
///
/// @return True if receive recording is enabled, false otherwise
///
bool serial_port_receive_recording_enabled(const serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes send recording with a pre-opened stream
///
/// @details Sets up the serial port to record all sent data to the specified
///          file stream. The caller must open the stream in binary write mode.
///
/// @param _serial_port Pointer to the serial port structure
/// @param _send_stream Pre-opened file stream for recording sent data
///
/// @note The caller is responsible for managing the lifecycle of the stream
///
void serial_port_init_send_recording_stream(const serial_port* _serial_port, FILE* _send_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording of transmitted data to a file
///
/// @details Configures the serial port to automatically record all data
///          transmitted to the remote device to the specified file. This
///          feature enables monitoring of outgoing communication for debugging
///          and protocol verification purposes. The recording operates
///          transparently alongside normal write operations.
///
/// @param _serial_port Pointer to the serial port structure
/// @param _send_file_name Path to the file where transmitted data will be
///                        recorded. The file will be created if it does not
///                        exist, or truncated if it already exists
///
/// @note Recording begins immediately and continues until the port is closed or
///       recording is explicitly disabled
///
/// @warning Continuous transmission recording can consume significant disk
///          space depending on data volume and transmission frequency
///
void serial_port_init_send_recording_file(const serial_port* _serial_port, const char* _send_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes send recording stream on a serial port
///
/// @details Closes and cleans up the send recording stream.
///
/// @param _serial_port Pointer to the serial port structure
///
void serial_port_close_send_recording_stream(const serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if send recording is enabled for a serial port
///
/// @details Determines whether data transmission is being recorded on the serial port.
///
/// @param _serial_port Pointer to the serial port structure
///
/// @return True if send recording is enabled, false otherwise
///
bool serial_port_send_recording_enabled(const serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording with pre-opened streams for both directions
///
/// @details Convenience function that sets up recording for both received and
///          sent data using existing file streams. The caller must open both
///          streams in binary write mode.
///
/// @param _serial_port Pointer to the serial port structure
/// @param _receive_stream Pre-opened file stream for recording received data
/// @param _send_stream Pre-opened file stream for recording sent data
///
/// @note Either stream parameter can be NULL to disable recording for that
///       direction
/// @note The caller is responsible for managing the lifecycle of the streams
///
void serial_port_init_recording_streams(const serial_port* _serial_port, FILE* _receive_stream, FILE* _send_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes bidirectional recording of serial communication
///
/// @details Configures the serial port to automatically record both received
///          and transmitted data to separate files. This provides complete
///          communication logging for debugging, protocol analysis, and
///          compliance recording. Both recording streams operate independently
///          and transparently with normal I/O operations.
///
/// @param _serial_port Pointer to the serial port structure
/// @param _receive_file_name Path to the file for recording received data.
///                           Created or truncated as needed
/// @param _send_file_name Path to the file for recording transmitted data.
///                        Created or truncated as needed
///
/// @note Both recording streams begin immediately and continue until the port
///       is closed or recording is explicitly disabled
///
/// @note The receive and send files are independent and can be analyzed
///       separately or combined for complete communication reconstruction
///
/// @warning Bidirectional recording doubles the disk space requirements
///          compared to single-direction recording
///
void serial_port_init_recording_files(const serial_port* _serial_port, const char* _receive_file_name,
    const char* _send_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes both receive and send recording streams on a serial port
///
/// @details Closes any open receive or send recording file streams associated
///          with the specified serial port.
///
/// @param _serial_port Pointer to the serial port structure
///
void serial_port_close_recording_streams(const serial_port* _serial_port);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if either receive or send recording is enabled for a serial
///        port
///
/// @details Determines whether data reception or transmission is being recorded
///          on the serial port.
///
/// @param _serial_port Pointer to the serial port structure
///
/// @return True if any form of recording (receive or send) is enabled, false
///         otherwise
///
bool serial_port_recording_enabled(const serial_port* _serial_port);

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
