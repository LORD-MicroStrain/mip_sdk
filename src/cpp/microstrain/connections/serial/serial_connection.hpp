#pragma once

#include "microstrain/connections/connection.hpp"
#include "microstrain/connections/serial/serial_port.h"

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        /// @addtogroup microstrain_platform
        /// @{
        ///

        ////////////////////////////////////////////////////////////////////////////////
        /// @brief Cross-platform serial communication connection implementation
        ///
        /// @details Provides a high-level C++ interface for serial port communication
        ///          with comprehensive features including:
        ///          - Cross-platform support (Windows, Linux, macOS)
        ///          - Configurable baudrates and communication parameters
        ///          - Automatic connection lifecycle management
        ///          - Built-in data recording capabilities for debugging
        ///          - Timeout-based I/O operations with timestamping
        ///          - Thread-safe operation with proper resource cleanup
        ///
        /// @details The connection uses standard serial port configurations:
        ///          - 8 data bits, no parity, 1 stop bit (8N1)
        ///          - No hardware or software flow control
        ///          - Configurable baudrate (must match device settings)
        ///          - Binary data transmission with no character translation
        ///
        /// @details Recording functionality allows transparent logging of all
        ///          communication for protocol analysis, debugging, and compliance
        ///          purposes. Both receive and transmit directions can be recorded
        ///          independently to separate files or streams.
        ///
        /// @note This class is designed for communicating with MicroStrain devices
        ///       but can be used for general-purpose serial communication
        ///
        /// @see UsbSerialConnection, TcpClientConnection, Connection
        ///
        class SerialConnection : public Connection, protected C::serial_port
        {
        public:
            /// @brief Connection type identifier for serial connections
            static constexpr const char* TYPE = "Serial";

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Destructor that properly cleans up the serial connection and
            ///        recording resources
            ///
            /// @details Performs a complete cleanup of the serial connection by:
            ///          - Disconnecting from the serial port if still connected
            ///          - Closing managed receive recording streams and files
            ///          - Closing managed send recording streams and files
            ///
            /// @note Only recording streams/files that were opened by the connection
            ///       (through initializeReceiveRecordingFile or
            ///       initializeSendRecordingFile) are automatically closed. User-provided
            ///       streams are not managed.
            ///
            ~SerialConnection() override;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a connection that will communicate with a device over serial
            ///
            /// @param _portName Path to the port to connect to. This usually looks like
            ///                  "COM<N>" on Windows, and "/dev/tty<N>" on Unix
            /// @param _baudrate Baudrate to open the device at
            ///
            /// @note The connection baudrate needs to match the device baudrate for a
            ///       successful connection
            ///
            explicit SerialConnection(const char* _portName, const uint32_t _baudrate);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a connection that will communicate with a device over serial
            ///        and initializes recording streams
            ///
            /// @param _portName Path to the port to connect to. This usually looks like
            ///                  "COM<N>" on Windows, and "/dev/tty<N>" on Unix
            /// @param _baudrate Baudrate to open the device at
            /// @param _receiveRecordingFileName The name of the file to open for received
            ///                                  data recording
            /// @param _sendRecordingFileName The name of the file to open for sent data
            ///                               recording
            ///
            /// @note The connection baudrate needs to match the device baudrate for a
            ///       successful connection
            ///
            explicit SerialConnection(const char* _portName, const uint32_t _baudrate,
                const char* _receiveRecordingFileName, const char* _sendRecordingFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a connection that will communicate with a device over serial
            ///        and initializes recording streams
            ///
            /// @param _portName Path to the port to connect to. This usually looks like
            ///                  "COM<N>" on Windows, and "/dev/tty<N>" on Unix
            /// @param _baudrate Baudrate to open the device at
            /// @param _receiveRecordingStream A stream for received data recording
            /// @param _sendRecordingStream A stream for sent data recording
            ///
            /// @note The connection baudrate needs to match the device baudrate for a
            ///       successful connection
            ///
            explicit SerialConnection(const char* _portName, const uint32_t _baudrate, FILE* _receiveRecordingStream,
                FILE* _sendRecordingStream);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Connect to the port
            ///
            /// @return true if the connection was successfully established, false on
            ///         failure
            ///
            bool connect() final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Disconnect from the port
            ///
            /// @return true if the disconnection was successful, false on failure
            ///
            bool disconnect() final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Check if the port is connected
            ///
            /// @return true if the port is currently connected, false otherwise
            ///
            bool isConnected() const final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Reads bytes from the serial port
            ///
            /// @param _buffer Buffer to store the read data in
            /// @param _byte_count Max number of bytes to read. Typically, the size of the
            ///                    buffer
            /// @param _wait_time_ms Time to wait for data in milliseconds
            /// @param _bytes_read_out Number of bytes actually read
            /// @param _timestamp_out Timestamp of when the data was read
            ///
            /// @return True if the read was successful, otherwise false
            ///
            bool read(uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time_ms, size_t& _bytes_read_out,
                EmbeddedTimestamp& _timestamp_out) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Writes bytes over the serial port
            ///
            /// @param _data The data to write
            /// @param _byte_count The number of bytes to write from the buffer
            /// @param _bytes_written_out The number of bytes actually written
            ///
            /// @return True if the write was successful, otherwise false
            ///
            bool write(const uint8_t* _data, const size_t _byte_count, size_t& _bytes_written_out) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Get the port name for the connection
            ///
            /// @return Port name string
            ///
            const char* interfaceName() const final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Get the baudrate for the connection
            ///
            /// @return Current baudrate value in bits per second
            ///
            uint32_t parameter() const final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Get the baudrate for the connection
            ///
            /// @return Current baudrate value in bits per second
            ///
            uint32_t baudrate() const;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Update the baudrate for the connection
            ///
            /// @param _baudrate New baudrate value in bits per second
            ///
            /// @return true if the baudrate was successfully updated, false on failure
            ///
            bool updateBaudrate(const uint32_t _baudrate);
        };

        ////////////////////////////////////////////////////////////////////////////////
        /// @brief Specialized serial connection for USB-connected devices
        ///
        /// @details Extends SerialConnection with USB-specific optimizations and
        ///          identification. Provides identical functionality to
        ///          SerialConnection but with enhanced type identification for USB
        ///          serial devices such as CDC-ACM or FTDI-based connections.
        ///
        /// @details This class is particularly suited for:
        ///          - USB-to-serial converters (FTDI, Prolific, etc.)
        ///          - CDC-ACM devices (native USB serial)
        ///          - MicroStrain devices with USB connectivity
        ///          - Any serial device connected via USB interface
        ///
        /// @details Inherits all capabilities from SerialConnection including:
        ///          - Cross-platform USB serial port support
        ///          - Automatic baudrate negotiation
        ///          - Built-in recording and debugging features
        ///          - Robust connection management
        ///
        /// @note The baudrate parameter may be ignored by some USB serial devices
        ///       that use virtual COM ports, but should still match the device's
        ///       expected communication speed for proper operation
        ///
        /// @see SerialConnection, TcpClientConnection, Connection
        ///
        class UsbSerialConnection final : public SerialConnection
        {
        public:
            /// @brief Connection type identifier for USB serial connections
            static constexpr const char* TYPE = "USB";

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Destructor that properly cleans up the serial connection and
            ///        recording resources
            ///
            /// @details Performs a complete cleanup of the serial connection by:
            ///          - Disconnecting from the serial port if still connected
            ///          - Closing managed receive recording streams and files
            ///          - Closing managed send recording streams and files
            ///
            /// @note Only recording streams/files that were opened by the connection
            ///       (through initializeReceiveRecordingFile or
            ///       initializeSendRecordingFile) are automatically closed. User-provided
            ///       streams are not managed.
            ///
            ~UsbSerialConnection() override = default;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a connection that will communicate over serial USB
            ///
            /// @param _portName Path to the port to connect to. This usually looks like
            ///                  "COM<N>" on Windows, and "/dev/tty<N>" on Unix
            /// @param _baudrate Baudrate to open the connection at
            ///
            explicit UsbSerialConnection(const char* _portName, const uint32_t _baudrate);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a connection that will communicate over serial USB and
            ///        initializes recording streams
            ///
            /// @param _portName Path to the port to connect to. This usually looks like
            ///                  "COM<N>" on Windows, and "/dev/tty<N>" on Unix
            /// @param _baudrate Baudrate to open the device at
            /// @param _receiveRecordingFileName The name of the file to open for received
            ///                                  data recording
            /// @param _sendRecordingFileName The name of the file to open for sent data
            ///                               recording
            ///
            explicit UsbSerialConnection(const char* _portName, const uint32_t _baudrate,
                const char* _receiveRecordingFileName, const char* _sendRecordingFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a connection that will communicate over serial USB and
            ///        initializes recording streams
            ///
            /// @param _portName Path to the port to connect to. This usually looks like
            ///                  "COM<N>" on Windows, and "/dev/tty<N>" on Unix
            /// @param _baudrate Baudrate to open the device at
            /// @param _receiveRecordingStream A stream for received data recording
            /// @param _sendRecordingStream A stream for sent data recording
            ///
            explicit UsbSerialConnection(const char* _portName, const uint32_t _baudrate, FILE* _receiveRecordingStream,
                FILE* _sendRecordingStream);
        };
        ///
        /// @}
        ////////////////////////////////////////////////////////////////////////////////
    } // namespace connections
} // namespace microstrain
