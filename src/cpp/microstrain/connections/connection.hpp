#pragma once

#include "microstrain/connections/recording/recording_connection.hpp"

#include <microstrain/embedded_time.hpp>
#include <microstrain/span.hpp>

#include <cstdint>
#include <cstdio>

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        /// @group
        /// @addtogroup microstrain_platform
        /// @{
        ///

        ////////////////////////////////////////////////////////////////////////////////
        /// @brief Abstract base class for communication connections
        ///
        /// @details Provides a unified interface for different connection types such as
        ///          serial, TCP, and USB connections. This class defines the common
        ///          operations that all connection types must implement, enabling
        ///          polymorphic usage of different transport mechanisms.
        ///
        /// @see SerialConnection, UsbSerialConnection, TcpConnection
        ///
        class Connection
        {
        public:
            /// @brief Default connection type identifier for the base class
            static constexpr const char* TYPE = "None";

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Deleted default constructor - connections must specify a type
            ///
            Connection() = delete;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Deleted copy constructor - connections are non-copyable resources
            ///
            Connection(Connection&) = delete;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Deleted assignment operator - connections are non-assignable
            ///
            void operator=(Connection&) = delete;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Virtual destructor for proper cleanup of derived classes
            ///
            virtual ~Connection() = default;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Establishes the connection to the target device or endpoint
            ///
            /// @details Attempts to open and configure the connection using the parameters
            ///          specified during construction. The specific behavior depends on the
            ///          connection type (e.g., opening a serial port, establishing a TCP
            ///          socket connection).
            ///
            /// @return True if the connection was successfully established and configured,
            ///         false if the operation failed due to invalid parameters, resource
            ///         unavailability, or configuration errors
            ///
            /// @note Multiple calls to connect() on an already connected instance should
            ///       return true without side effects
            ///
            /// @see disconnect(), isConnected()
            ///
            virtual bool connect() = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Closes the connection and releases associated resources
            ///
            /// @details Terminates the active connection and frees any system resources
            ///          such as file handles, socket descriptors, or buffers. After calling
            ///          this method, the connection cannot be used for I/O operations until
            ///          connect() is called again.
            ///
            /// @return True if the connection was successfully closed or was already
            ///         closed, false if an error occurred during cleanup
            ///
            /// @note It is safe to call this method multiple times or on an already closed
            ///       connection
            ///
            /// @see connect(), isConnected()
            ///
            virtual bool disconnect() = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Checks if the connection is currently active and operational
            ///
            /// @details Verifies that the connection is established and ready for I/O
            ///          operations. This is typically a lightweight check of the internal
            ///          state and does not perform actual communication with the remote
            ///          endpoint.
            ///
            /// @return True if the connection is active and ready for use, false if the
            ///         connection is closed or in an error state
            ///
            /// @note This method only checks the local connection state and does not verify
            ///       that the remote endpoint is still reachable or responsive
            ///
            /// @see connect(), disconnect()
            ///
            virtual bool isConnected() const = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Reads data from the connection with timeout support
            ///
            /// @details Attempts to read up to the specified number of bytes from the
            ///          connection into the provided buffer. The operation will block for
            ///          up to the specified timeout waiting for data to become available.
            ///          Returns the actual number of bytes read and a timestamp indicating
            ///          when the read operation completed.
            ///
            /// @param _buffer Pointer to the buffer where received data will be stored.
            ///                Must be at least _byte_count bytes in size
            /// @param _byte_count Maximum number of bytes to read, typically the size of
            ///                    the buffer. The actual number read may be less
            /// @param _wait_time_ms Maximum time to wait for data in milliseconds. A value
            ///                      of 0 performs a non-blocking read
            /// @param _bytes_read_out Reference to receive the actual number of bytes read
            ///                        from the connection
            /// @param _timestamp_out Reference to receive the timestamp when the data was
            ///                       read, useful for protocol timing analysis
            ///
            /// @return True if the read operation completed successfully (may read 0
            ///         bytes), false if the connection is not open, an error occurred, or
            ///         the connection was lost
            ///
            /// @warning Ensure the buffer is at least _byte_count bytes to prevent buffer
            ///          overflow
            ///
            /// @see write()
            ///
            virtual bool read(uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time_ms,
                size_t& _bytes_read_out, EmbeddedTimestamp& _timestamp_out) = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Writes data to the connection
            ///
            /// @details Transmits the specified number of bytes from the buffer to the
            ///          connected endpoint. The operation attempts to write all requested
            ///          bytes and returns the actual number successfully transmitted.
            ///
            /// @param _data Pointer to the data buffer containing bytes to transmit. Must
            ///              contain at least _byte_count valid bytes
            /// @param _byte_count Number of bytes to write from the buffer. Must not exceed
            ///                    the buffer size
            /// @param _bytes_written_out Reference to receive the actual number of bytes
            ///                           successfully written to the connection
            ///
            /// @return True if the write operation completed successfully, false if the
            ///         connection is not open, the operation failed, or not all bytes could
            ///         be transmitted
            ///
            /// @note Some connection types may buffer data internally, so successful
            ///       completion does not guarantee the data has reached the remote endpoint
            ///
            /// @warning Ensure the buffer contains at least _byte_count valid bytes to
            ///          prevent reading invalid memory
            ///
            /// @see read()
            ///
            virtual bool write(const uint8_t* _data, const size_t _byte_count, size_t& _bytes_written_out) = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the connection-specific interface identifier
            ///
            /// @details Returns a string that identifies the specific interface or endpoint
            ///          for this connection. The format and content depend on the
            ///          connection type:
            ///          - Serial connections: port name (e.g., "COM1", "/dev/ttyACM0")
            ///          - TCP connections: hostname or IP address (e.g., "127.0.0.1")
            ///
            /// @return A string containing the interface name
            ///
            /// @see parameter(), info(), type()
            ///
            virtual const char* interfaceName() const = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the connection-specific parameter value
            ///
            /// @details Returns a numeric parameter that provides additional information
            ///          about the connection configuration. The meaning depends on the
            ///          connection type:
            ///          - Serial connections: baudrate (e.g., 115200)
            ///          - TCP connections: port number (e.g., 8080)
            ///
            /// @return Connection-specific parameter value as an unsigned 32-bit integer
            ///
            /// @see interfaceName(), info(), type()
            ///
            virtual uint32_t parameter() const = 0;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes receive recording with a pre-opened stream
            ///
            /// @details Sets up the connection to record all received data to the specified
            ///          file stream. The caller must open the stream in binary write mode.
            ///
            /// @param _receiveStream Pre-opened file stream for recording received data
            ///
            /// @note The caller is responsible for managing the lifecycle of the stream
            ///
            virtual void initializeReceiveRecordingStream(FILE* _receiveStream) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes recording of received data to a file
            ///
            /// @details Configures the connection to automatically record all data received
            ///          from the remote device to the specified file. This feature is
            ///          useful for debugging, protocol analysis, and data logging.
            ///
            /// @param _receiveFileName Path to the file where received data will be
            ///                         recorded. The file will be created if it does not
            ///                         exist, or truncated if it already exists
            ///
            /// @note Recording begins immediately and continues until the connection is
            ///       closed or recording is explicitly disabled
            ///
            /// @warning Ensure sufficient disk space is available as continuous logging
            ///          can generate large files over time
            ///
            virtual void initializeReceiveRecordingFile(const char* _receiveFileName) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Stops recording of received data and closes the receive stream
            ///
            /// @details Closes the receive recording stream if it is open, regardless of
            ///          how it was initialized. This includes streams opened through
            ///          constructor parameters, initialization methods, or user-provided
            ///          streams.
            ///
            /// @warning This will close user-provided receive streams. Ensure any
            ///          external references to this stream are handled appropriately
            ///          before calling this function
            ///
            virtual void closeReceiveRecordingStream() final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes send recording with a pre-opened stream
            ///
            /// @details Sets up the connection to record all sent data to the specified
            ///          file stream. The caller must open the stream in binary write mode.
            ///
            /// @param _sendStream Pre-opened file stream for recording sent data
            ///
            /// @note The caller is responsible for managing the lifecycle of the stream
            ///
            virtual void initializeSendRecordingStream(FILE* _sendStream) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes recording of transmitted data to a file
            ///
            /// @details Configures the connection to automatically record all data
            ///          transmitted to the remote device to the specified file. This
            ///          feature enables monitoring of outgoing communication for debugging
            ///          and protocol verification purposes.
            ///
            /// @param _sendFileName Path to the file where transmitted data will be
            ///                      recorded. The file will be created if it does not
            ///                      exist, or truncated if it already exists
            ///
            /// @note Recording begins immediately and continues until the connection is
            ///       closed or recording is explicitly disabled
            ///
            /// @warning Continuous transmission recording can consume significant disk
            ///          space depending on data volume and transmission frequency
            ///
            virtual void initializeSendRecordingFile(const char* _sendFileName) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Stops recording of transmitted data and closes the send stream
            ///
            /// @details Closes the send recording stream if it is open, regardless of
            ///          how it was initialized. This includes streams opened through
            ///          constructor parameters, initialization methods, or user-provided
            ///          streams.
            ///
            /// @warning This will close user-provided send streams. Ensure any external
            ///          references to this stream are handled appropriately before
            ///          calling this function
            ///
            virtual void closeSendRecordingStream() final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes recording with pre-opened streams for both directions
            ///
            /// @details Convenience function that sets up recording for both received and
            ///          sent data using existing file streams. The caller must open both
            ///          streams in binary write mode.
            ///
            /// @param _receiveStream Pre-opened file stream for recording received data
            /// @param _sendStream Pre-opened file stream for recording sent data
            ///
            /// @note Either stream parameter can be NULL to disable recording for that
            ///       direction
            /// @note The caller is responsible for managing the lifecycle of the streams
            ///
            virtual void initializeRecordingStreams(FILE* _receiveStream, FILE* _sendStream) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes bidirectional recording of communication
            ///
            /// @details Configures the connection to automatically record both received and
            ///          transmitted data to separate files. This provides complete
            ///          communication logging for debugging, protocol analysis, and
            ///          compliance recording.
            ///
            /// @param _receiveFileName Path to the file for recording received data.
            ///                         Created or truncated as needed
            /// @param _sendFileName Path to the file for recording transmitted data.
            ///                      Created or truncated as needed
            ///
            /// @note Both recording streams begin immediately and continue until the
            ///       connection is closed or recording is explicitly disabled
            ///
            /// @note The receive and send files are independent and can be analyzed
            ///       separately or combined for complete communication reconstruction
            ///
            /// @warning Bidirectional recording doubles the disk space requirements
            ///          compared to single-direction recording
            ///
            virtual void initializeRecordingFiles(const char* _receiveFileName, const char* _sendFileName) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Stops recording of received and transmitted data and closes all
            ///        recording streams
            ///
            /// @details Closes both receive and send recording streams if they are open,
            ///          regardless of how they were initialized. This includes streams
            ///          opened through constructor parameters, initialization methods, or
            ///          user-provided streams.
            ///
            /// @warning This will close user-provided streams. Ensure any external
            ///          references to these streams are handled appropriately before
            ///          calling this function
            ///
            virtual void closeRecordingStreams() final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Retrieves both interface name and parameter in a single call
            ///
            /// @details Convenience method that returns both the interface identifier and
            ///          parameter value simultaneously, equivalent to calling
            ///          interfaceName() and parameter() separately.
            ///
            /// @param _interfaceName Reference to receive the interface name
            /// @param _parameter Reference to receive the parameter value
            ///
            /// @see interfaceName(), parameter(), type()
            ///
            virtual void info(const char*& _interfaceName, uint32_t& _parameter) const final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the connection type identifier string
            ///
            /// @details Returns a string that identifies the general type of this
            ///          connection (e.g., "Serial", "TCP", "USB"). This is useful for
            ///          logging, debugging, and type identification in generic code.
            ///
            /// @return A string containing the connection type
            ///
            /// @see interfaceName(), parameter(), info()
            ///
            virtual const char* type() const final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Reads data using a Span interface with timeout support
            ///
            /// @details Convenience wrapper around read() that uses a Span for type-safe
            ///          buffer management. Provides the same functionality as read() but
            ///          with automatic size management through the Span interface.
            ///
            /// @param _buffer Span representing the buffer where data will be stored. The
            ///                span size determines the maximum bytes to read
            /// @param _waitTimeMs Maximum time to wait for data in milliseconds
            /// @param _bytesReadOut Reference to receive the actual number of bytes read
            /// @param _timestampOut Reference to receive the read timestamp
            ///
            /// @return True if the read operation completed successfully, false otherwise
            ///
            /// @see read(), readSpanUpdate()
            ///
            virtual bool readSpan(const Span<uint8_t>& _buffer, const uint32_t _waitTimeMs,
                size_t& _bytesReadOut, EmbeddedTimestamp& _timestampOut) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Writes data using a Span interface
            ///
            /// @details Convenience wrapper around write() that uses a Span for type-safe
            ///          buffer management. The span size automatically determines the
            ///          number of bytes to write.
            ///
            /// @param _data Span containing the data to transmit. All bytes in the span
            ///              will be written
            /// @param _bytes_written_out Reference to receive the actual number of bytes
            ///                           successfully written
            ///
            /// @return True if the write operation completed successfully, false otherwise
            ///
            /// @see write(), readSpan()
            ///
            virtual bool writeSpan(const Span<const uint8_t>& _data, size_t& _bytes_written_out) final;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Reads data and updates the span to reflect actual bytes read
            ///
            /// @details Performs a read operation and automatically adjusts the provided
            ///          span to contain only the bytes that were actually read. This is
            ///          useful when you want to process exactly the data that was received
            ///          without needing to track the length separately.
            ///
            /// @param _buffer Reference to a span that will be updated to contain only the
            ///                bytes that were read. The span size will be reduced to match
            ///                the actual bytes read
            /// @param _wait_time_ms Maximum time to wait for data in milliseconds
            /// @param _timestamp_out Reference to receive the read timestamp
            ///
            /// @return True if the read operation completed successfully, false otherwise.
            ///         On success, _buffer will contain exactly the data that was read
            ///
            /// @note If the read fails, the span remains unchanged
            ///
            /// @see readSpan(), read()
            ///
            virtual bool readSpanUpdate(Span<uint8_t>& _buffer, const uint32_t _wait_time_ms,
                EmbeddedTimestamp& _timestamp_out) final;

        protected:
            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Protected constructor for derived classes
            ///
            /// @details Initializes the connection with the specified type identifier. This
            ///          constructor is only accessible to derived classes and ensures that
            ///          every connection has a valid type identifier.
            ///
            /// @param _type A string identifying the connection type. This should be one of
            ///              the constants defined in the derived connection classes
            ///
            explicit Connection(const char* _type);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Protected constructor for derived classes
            ///
            /// @param _type A string identifying the connection type. This should be one of
            ///              the constants defined in the derived connection classes
            /// @param _receiveRecordingFileName File path for recording received data
            /// @param _sendRecordingFileName File path for recording sent data
            ///
            explicit Connection(const char* _type, const char* _receiveRecordingFileName,
                const char* _sendRecordingFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Protected constructor for derived classes
            ///
            /// @param _type A string identifying the connection type. This should be one of
            ///              the constants defined in the derived connection classes
            /// @param _receiveRecordingStream Pre-opened stream for recording received
            ///                                data
            /// @param _sendRecordingStream Pre-opened stream for recording sent data
            ///
            explicit Connection(const char* _type, FILE* _receiveRecordingStream, FILE* _sendRecordingStream);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Retrieves a pointer to the internal recording connection structure
            ///
            /// @details This method provides access to the recording_connection object that
            ///          manages data recording operations within the TCP client. The
            ///          recording_connection is essential for configuring and controlling
            ///          recording behavior, such as initializing streams,
            ///          enabling/disabling recording, and closing streams when no longer
            ///          needed.
            ///
            /// @return A pointer to the internal recording_connection structure.
            ///
            /// @note This is only used for connection creation to simplify initialization
            ///
            recording_connection* recordingConnection();

            /// @brief Connection type identifier, set during construction
            const char* mType = TYPE;

        private:
            /// @brief Recording connection interface, set during construction
            Recording mRecordingConnection = {};
        };
        ///
        /// @}
        ////////////////////////////////////////////////////////////////////////////////
    } // namespace connections
} // namespace microstrain
