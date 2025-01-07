#include "recording_connection.hpp"

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Creates a RecordingConnection that will write received bytes to
        ///       recvStream, and sent bytes to sendStream
        ///
        ///@param connection Connection object that will actually communicate with the
        ///                  device
        ///
        ///@param recvStream The stream to write to when bytes are received. Null if
        ///                  received bytes should not be written to a stream
        ///
        ///@param sendStream The stream to write to when bytes are sent. Null if sent
        ///                  bytes should not be written to a stream
        ///
        RecordingConnection::RecordingConnection(Connection* connection, std::ostream* recvStream, std::ostream* sendStream) :
            mConnection(connection), mRecvFile(recvStream), mSendFile(sendStream)
        {
            mType = TYPE;
        }

        ///@copydoc microstrain::Connection::sendToDevice
        bool RecordingConnection::sendToDevice(const uint8_t* data, size_t length)
        {
            const bool ok = mConnection->sendToDevice(data, length);

            if (ok && mSendFile != nullptr)
            {
                mSendFile->write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(length));
                mSendFileWritten += length;
            }

            return ok;
        }

        ///@copydoc microstrain::Connection::recvFromDevice
        bool RecordingConnection::recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out)
        {
            const bool ok = mConnection->recvFromDevice(buffer, max_length, wait_time_ms, length_out, timestamp_out);

            if (ok && mRecvFile != nullptr)
            {
                mRecvFile->write(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(*length_out));
                mRecvFileWritten += *length_out;
            }

            return ok;
        }
    } // namespace connections
} // namespace microstrain
