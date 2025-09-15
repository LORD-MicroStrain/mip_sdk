#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup microstrain_platform
///
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @defgroup recording_connection Recording Connection
///
/// @brief A simple implementation for writing received and sent bytes to files
///
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @brief Structure for managing file-based recording of communication data
///
/// @details Provides dual-file recording capability for device communication:
///          - Separate files for received and sent data streams
///          - Automatic byte counting for both directions
///          - Binary file format for exact data preservation
///
/// @note Files are opened in write-binary mode ("wb") and will overwrite
///       existing files with the same name.
///
typedef struct recording_connection
{
    /// @brief Output file stream for received bytes
    FILE* receive_file;

    /// @brief Output file stream for sent bytes
    FILE* send_file;

    /// @brief Total number received bytes written to the file stream
    uint64_t receive_bytes_written;

    /// @brief Total number sent bytes written to the file stream
    uint64_t send_bytes_written;
} recording_connection;

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes a recording connection structure
///
/// @details Sets all file pointers to NULL and resets byte counters to zero.
///          This prepares the structure for use with recording operations.
///
/// @param _recording_connection Pointer to the recording connection structure
///
void recording_connection_init(recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording connection with a pre-opened receive stream
///
/// @details Sets up the recording connection to use an existing file stream
///          for recording received data. Resets the received byte counter to
///          zero. The send stream remains uninitialized.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _receive_stream Pre-opened file stream for recording received data
///
/// @note The caller is responsible for managing the lifecycle of the stream
///
void recording_connection_init_receive_stream(recording_connection* _recording_connection, FILE* _receive_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Opens a file for recording received data bytes
///
/// @details Opens the specified file in write-binary mode for recording
///          incoming data from the device. Resets the received byte counter
///          to zero and logs the file opening operation.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _receive_file_name Path/name of the file to create for received data
///
/// @note Function will assert on failure - ensure valid parameters are provided
///
/// @warning Will overwrite existing files with the same name
///
void recording_connection_open_receive_file(recording_connection* _recording_connection,
    const char* _receive_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes the stream used for recording received data bytes
///
/// @details Safely closes the stream for received data if it exists and sets
///          the stream pointer to NULL. Logs warnings if attempting to close a
///          non-existent stream.
///
/// @param _recording_connection Pointer to the recording connection structure
///
void recording_connection_close_receive_stream(recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Writes received data bytes to the recording file
///
/// @details Writes the provided buffer contents to the file stream for received
///          data and updates the total received bytes counter. Optionally
///          returns the number of bytes actually written to the file.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _receive_buffer Buffer containing data received from the device
/// @param _byte_count Number of bytes to write from the buffer
/// @param _bytes_written_out Optional output parameter for bytes actually
///                           written (can be NULL)
///
/// @note No operation performed if the file for received data is not open
///
void recording_connection_write_received_bytes(recording_connection* _recording_connection,
    const uint8_t* _receive_buffer, size_t _byte_count, size_t* _bytes_written_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording connection with a pre-opened send stream
///
/// @details Sets up the recording connection to use an existing file stream for
///          recording sent data. Resets the sent byte counter to zero. The
///          receive stream remains uninitialized.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _send_stream Pre-opened file stream for recording sent data
///
/// @note The caller is responsible for managing the lifecycle of the stream
///
void recording_connection_init_send_stream(recording_connection* _recording_connection, FILE* _send_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Opens a file for recording sent data bytes
///
/// @details Opens the specified file in write-binary mode for recording
///          outgoing data to the device. Resets the byte counter for sent data
///          to zero and logs the file opening operation.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _send_file_name Path/name of the file to create for sent data
///
/// @note Function will assert on failure - ensure valid parameters are provided
///
/// @warning Will overwrite existing files with the same name
///
void recording_connection_open_send_file(recording_connection* _recording_connection, const char* _send_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes the stream used for recording sent data bytes
///
/// @details Safely closes the stream for sent data if it exists and sets the
///          stream pointer to NULL. Logs warnings if attempting to close a
///          non-existent stream.
///
/// @param _recording_connection Pointer to the recording connection structure
///
void recording_connection_close_send_stream(recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Writes sent data bytes to the recording file
///
/// @details Writes the provided buffer contents to the file stream for sent
///          data and updates the total sent bytes counter. Optionally returns
///          the number of bytes actually written to the file.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _send_buffer Buffer containing data sent to the device
/// @param _byte_count Number of bytes to write from the buffer
/// @param _bytes_written_out Optional output parameter for bytes actually
///                           written (can be NULL)
///
/// @note No operation performed if the file for sent data is not open
///
void recording_connection_write_sent_bytes(recording_connection* _recording_connection, const uint8_t* _send_buffer,
    size_t _byte_count, size_t* _bytes_written_out);

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes recording connection with pre-opened file streams
///
/// @details Convenience function that initializes a recording connection using
///          already opened file streams for both received and sent data. Resets
///          both byte counters to zero. This is useful when you have existing
///          file streams that you want to use for recording instead of opening
///          new files.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _receive_stream Pre-opened file stream for recording received data
/// @param _send_stream Pre-opened file stream for recording sent data
///
/// @note Both streams can be NULL if only partial recording is needed
/// @note The caller is responsible for managing the lifecycle of the streams
///
void recording_connection_init_streams(recording_connection* _recording_connection, FILE* _receive_stream,
    FILE* _send_stream);

////////////////////////////////////////////////////////////////////////////////
/// @brief Opens both receive and send files for recording
///
/// @details Convenience function that opens both recording files
///          simultaneously. Will open individual files only if their filenames
///          are non-empty. At least one valid filename must be provided.
///
/// @param _recording_connection Pointer to the recording connection structure
/// @param _receive_file_name Path/name of the file to create for received data
/// @param _send_file_name Path/name of the file to create for sent data
///
/// @warning Will overwrite existing files with the same names
///
void recording_connection_open_files(recording_connection* _recording_connection, const char* _receive_file_name,
    const char* _send_file_name);

////////////////////////////////////////////////////////////////////////////////
/// @brief Closes both receive and send recording streams
///
/// @details Convenience function that closes any open recording streams.
///          Safely handles cases where only one stream type is open.
///          Logs warnings if no streams are open to close.
///
/// @param _recording_connection Pointer to the recording connection structure
///
void recording_connection_close_streams(recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if recording of received data is enabled
///
/// @details Returns true if the receive file stream is open and available for
///          recording incoming data from the device.
///
/// @param _recording_connection Pointer to the recording connection structure
///
/// @return true if receive recording is enabled, false otherwise
///
bool recording_connection_receive_recording_enabled(const recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if recording of sent data is enabled
///
/// @details Returns true if the send file stream is open and available for
///          recording outgoing data to the device.
///
/// @param _recording_connection Pointer to the recording connection structure
///
/// @return true if send recording is enabled, false otherwise
///
bool recording_connection_send_recording_enabled(const recording_connection* _recording_connection);

////////////////////////////////////////////////////////////////////////////////
/// @brief Checks if any recording functionality is enabled
///
/// @details Returns true if either receive or send recording (or both) is
///          currently enabled and available for recording data.
///
/// @param _recording_connection Pointer to the recording connection structure
///
/// @return true if any recording is enabled, false if no recording is active
///
bool recording_connection_enabled(const recording_connection* _recording_connection);

///
/// @}
////////////////////////////////////////////////////////////////////////////////
///
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
