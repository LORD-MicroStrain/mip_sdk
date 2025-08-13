#pragma once

#include "microstrain/connections/recording/recording_connection.h"

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        /// @addtogroup microstrain_platform
        /// @{
        ///

        ////////////////////////////////////////////////////////////////////////////////
        /// @brief C++ wrapper for recording connection data streams
        ///
        /// @details Provides a high-level interface for recording bidirectional
        ///          communication data to files or streams. This class manages both
        ///          receive and send recording independently, allowing selective
        ///          recording of communication directions.
        ///
        class Recording final : protected recording_connection
        {
        public:
            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Deleted copy constructor - recordings are non-copyable resources
            ///
            Recording(Recording&) = delete;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Deleted assignment operator - recordings are non-assignable
            ///
            void operator=(Recording&) = delete;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Destructor that closes managed recording streams
            ///
            /// @details Automatically closes any recording streams opened by this
            ///          Recording instance (through openReceiveFile or openSendFile).
            ///          User-provided streams are not closed.
            ///
            ~Recording();

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a recording instance with pre-opened streams
            ///
            /// @param _receiveStream Pre-opened file stream for recording received data.
            ///                       Can be NULL to disable receive recording
            /// @param _sendStream Pre-opened file stream for recording sent data. Can be
            ///                    NULL to disable send recording
            ///
            /// @note The caller is responsible for managing the lifecycle of the provided
            ///       streams
            ///
            explicit Recording(FILE* _receiveStream, FILE* _sendStream);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Creates a recording instance and opens files for recording
            ///
            /// @param _receiveFileName Path to file for recording received data. Created or
            ///                         truncated as needed
            /// @param _sendFileName Path to file for recording sent data. Created or
            ///                      truncated as needed
            ///
            /// @note The Recording instance manages the lifecycle of files opened through
            ///       this constructor
            ///
            explicit Recording(const char* _receiveFileName, const char* _sendFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes receive recording with a pre-opened stream
            ///
            /// @details Sets up recording for received data using an existing file stream.
            ///          The stream must be opened in binary write mode for proper data
            ///          recording.
            ///
            /// @param _receiveStream Pre-opened file stream for recording received data
            ///
            /// @note The caller is responsible for managing the stream lifecycle
            ///
            void initializeReceiveStream(FILE* _receiveStream);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Opens a file for recording received data
            ///
            /// @details Creates or truncates the specified file and initializes it for
            ///          recording received data. The Recording instance will manage the
            ///          lifecycle of this file.
            ///
            /// @param _receiveFileName Path to the file for recording received data
            ///
            /// @note The file will be automatically closed when the Recording instance is
            ///       destroyed
            ///
            void openReceiveFile(const char* _receiveFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Closes the receive recording file stream
            ///
            /// @details Closes the receive recording stream if it is currently open. Safe
            ///          to call multiple times or when no stream is open.
            ///
            void closeReceiveFile();

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Records received data to the active receive stream
            ///
            /// @details Writes the specified data to the receive recording stream if
            ///          recording is active. This function is typically called
            ///          automatically by connection read operations.
            ///
            /// @param _receiveBuffer Pointer to the received data to record
            /// @param _byteCount Number of bytes to write from the buffer
            /// @param _bytesWrittenOut Optional output parameter for the actual number of
            ///                         bytes written to the stream
            ///
            void writeReceivedBytes(const uint8_t* _receiveBuffer, const size_t _byteCount,
                size_t* _bytesWrittenOut = nullptr);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the total number of received bytes written to the stream
            ///
            /// @return Total number of bytes written to the receive recording stream since
            ///         initialization
            ///
            uint64_t receivedBytesWritten() const;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes send recording with a pre-opened stream
            ///
            /// @details Sets up recording for sent data using an existing file stream. The
            ///          stream must be opened in binary write mode for proper data
            ///          recording.
            ///
            /// @param _sendStream Pre-opened file stream for recording sent data
            ///
            /// @note The caller is responsible for managing the stream lifecycle
            ///
            void initializeSendStream(FILE* _sendStream);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Opens a file for recording sent data
            ///
            /// @details Creates or truncates the specified file and initializes it for
            ///          recording sent data. The Recording instance will manage the
            ///          lifecycle of this file.
            ///
            /// @param _sendFileName Path to the file for recording sent data
            ///
            /// @note The file will be automatically closed when the Recording instance is
            ///       destroyed
            ///
            void openSendFile(const char* _sendFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Closes the send recording file stream
            ///
            /// @details Closes the send recording stream if it is currently open. Safe to
            ///          call multiple times or when no stream is open.
            ///
            void closeSendFile();

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Records sent data to the active send stream
            ///
            /// @details Writes the specified data to the send recording stream if recording
            ///          is active. This function is typically called automatically by
            ///          connection write operations.
            ///
            /// @param _sendBuffer Pointer to the sent data to record
            /// @param _bufferSize Number of bytes to write from the buffer
            /// @param _bytesWrittenOut Optional output parameter for the actual number of
            ///                         bytes written to the stream
            ///
            void writeSentBytes(const uint8_t* _sendBuffer, const size_t _bufferSize,
                size_t* _bytesWrittenOut = nullptr);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the total number of sent bytes written to the stream
            ///
            /// @return Total number of bytes written to the send recording stream since
            ///         initialization
            ///
            uint64_t sentBytesWritten() const;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Initializes recording with pre-opened streams for both directions
            ///
            /// @details Convenience function that sets up recording for both received and
            ///          sent data using existing file streams. Either stream can be NULL to
            ///          disable recording for that direction.
            ///
            /// @param _receiveStream Pre-opened file stream for recording received data.
            ///                       Can be NULL
            /// @param _sendStream Pre-opened file stream for recording sent data. Can be
            ///                    NULL
            ///
            /// @note The caller is responsible for managing the lifecycle of the provided
            ///       streams
            ///
            void initializeStreams(FILE* _receiveStream, FILE* _sendStream);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Opens files for bidirectional recording
            ///
            /// @details Convenience function that opens separate files for recording both
            ///          received and sent data. The Recording instance manages the
            ///          lifecycle of both files.
            ///
            /// @param _receiveFileName Path to file for recording received data
            /// @param _sendFileName Path to file for recording sent data
            ///
            /// @note Both files will be automatically closed when the Recording instance is
            ///       destroyed
            ///
            void openFiles(const char* _receiveFileName, const char* _sendFileName);

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Closes both recording streams
            ///
            /// @details Closes both receive and send recording streams if they are
            ///          currently open. Safe to call when streams are not open.
            ///
            void closeStreams();

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Gets the total bytes written for both recording directions
            ///
            /// @details Convenience function that retrieves the byte counts for both
            ///          receive and send recording streams simultaneously.
            ///
            /// @param _receiveBytesWrittenOut Reference to receive the total received bytes
            ///                                written
            /// @param _sendBytesWrittenOut Reference to receive the total sent bytes
            ///                             written
            ///
            void bytesWritten(uint64_t& _receiveBytesWrittenOut, uint64_t& _sendBytesWrittenOut) const;

        private:
            // Allow the connection class direct access to the default constructor
            friend class Connection;

            ////////////////////////////////////////////////////////////////////////////////
            /// @brief Default constructor - recording requires stream setup
            ///
            /// @details Allows the Connection class to create an empty recording object.
            ///
            /// @note Recording requires stream setup and this should not be used aside
            ///       from the Connection class
            ///
            Recording() = default;

            /// @brief This manages the receive stream lifecycle
            bool mManageReceiveStream = false;

            /// @brief This manages the send stream lifecycle
            bool mManageSendStream = false;
        };
        ///
        /// @}
        ////////////////////////////////////////////////////////////////////////////////
    } // namespace connections
} // namespace microstrain
