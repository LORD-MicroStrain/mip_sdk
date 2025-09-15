#pragma once

#include "microstrain/connections/connection.hpp"
#include "microstrain/connections/tcp/tcp_socket.h"

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        /// @addtogroup microstrain_platform
        /// @{
        ///

        ////////////////////////////////////////////////////////////////////////////////
        /// @brief A TCP client connection for establishing and managing connections
        ///        with a server
        ///
        /// @details Provides a unified interface for TCP client communications,
        ///          including options for setting up recording of received and sent
        ///          data. The class manages the lifecycle of the TCP socket, handles
        ///          connecting to the specified server, reading from and writing to the
        ///          socket, and properly cleaning up resources during destruction.
        ///
        /// @note This class is designed as a final implementation of Connection,
        ///       providing specific functionality for TCP communication.
        ///
        class TcpClientConnection final : public Connection, protected C::tcp_client
        {
        public:
            /// @brief Connection type identifier for TCP client connections
            static constexpr const char* TYPE = "TCP Client";

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Destructor that properly cleans up the TCP connection and recording
            ///        resources
            ///
            /// @details Performs a complete cleanup of the TCP connection by:
            ///          - Disconnecting from the TCP server if still connected
            ///          - Closing managed receive recording streams and files
            ///          - Closing managed send recording streams and files
            ///
            /// @note Only recording streams/files that were opened by the connection
            ///       (through initializeReceiveRecordingFile or
            ///       initializeSendRecordingFile) are automatically closed. User-provided
            ///       streams are not managed.
            ///
            ~TcpClientConnection() override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a TCP client connection to the specified server
            ///
            /// @param _hostname Hostname or IP address of the TCP server to connect to
            /// @param _port Port number on the server to connect to
            /// @param _timeoutMs Connection timeout in milliseconds
            ///
            explicit TcpClientConnection(const char* _hostname, const uint16_t _port, const uint32_t _timeoutMs = 3000);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a TCP client connection with recording capabilities
            ///
            /// @param _hostname Hostname or IP address of the TCP server to connect to
            /// @param _port Port number on the server to connect to
            /// @param _receiveRecordingFileName File path for recording received data
            /// @param _sendRecordingFileName File path for recording sent data
            /// @param _timeoutMs Connection timeout in milliseconds
            ///
            explicit TcpClientConnection(const char* _hostname, const uint16_t _port,
                const char* _receiveRecordingFileName, const char* _sendRecordingFileName,
                const uint32_t _timeoutMs = 3000);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a TCP client connection with pre-opened recording streams
            ///
            /// @param _hostname Hostname or IP address of the TCP server to connect to
            /// @param _port Port number on the server to connect to
            /// @param _receiveRecordingStream Pre-opened stream for recording received
            ///                                data
            /// @param _sendRecordingStream Pre-opened stream for recording sent data
            /// @param _timeoutMs Connection timeout in milliseconds
            ///
            explicit TcpClientConnection(const char* _hostname, const uint16_t _port, FILE* _receiveRecordingStream,
                FILE* _sendRecordingStream, const uint32_t _timeoutMs = 3000);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Establishes a TCP connection to the configured server
            ///
            /// @return True if the connection was successfully established, false if the
            ///         connection failed due to network issues, server unavailability, or
            ///         timeout
            ///
            bool connect() override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Closes the TCP connection
            ///
            /// @return True if the connection was successfully closed or was already
            ///         closed, false if an error occurred during disconnection
            ///
            bool disconnect() override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Checks if the TCP connection is currently active
            ///
            /// @return True if the TCP socket is connected and operational, false if the
            ///         connection is closed or in an error state
            ///
            bool isConnected() const override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Reads data from the TCP connection with timeout support
            ///
            /// @param _buffer Buffer to store the received data
            /// @param _byte_count Maximum number of bytes to read
            /// @param _wait_time_ms Maximum time to wait for data in milliseconds
            /// @param _bytes_read_out Number of bytes actually read from the socket
            /// @param _timestamp_out Timestamp when the data was received
            ///
            /// @return True if the read operation completed successfully, false if the
            ///         connection is not open or an error occurred
            ///
            bool read(uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time_ms, size_t& _bytes_read_out,
                EmbeddedTimestamp& _timestamp_out) override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Writes data to the TCP connection
            ///
            /// @param _data Buffer containing the data to transmit
            /// @param _byte_count Number of bytes to write from the buffer
            /// @param _bytes_written_out Number of bytes actually written to the socket
            ///
            /// @return True if the write operation completed successfully, false if the
            ///         connection is not open or an error occurred
            ///
            bool write(const uint8_t* _data, const size_t _byte_count, size_t& _bytes_written_out) override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the hostname or IP address for this connection
            ///
            /// @return String containing the hostname or IP address of the server
            ///
            const char* interfaceName() const override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the port number for this connection
            ///
            /// @return Port number as an unsigned 16-bit integer cast to 32-bit
            ///
            uint32_t parameter() const override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the hostname or IP address for this connection
            ///
            /// @return String containing the hostname or IP address of the server
            ///
            const char* hostname() const;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the port number for this connection
            ///
            /// @return Port number
            ///
            uint16_t port() const;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Updates the port number for this connection
            ///
            /// @param _port New port number to use for connections
            ///
            /// @return True if the port was successfully updated, false if the update
            ///         failed or the connection is currently active
            ///
            bool updatePort(const uint16_t _port);
        };
        ///
        /// @}
        ////////////////////////////////////////////////////////////////////////////////
    } // namespace connections
} // namespace microstrain
