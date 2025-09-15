#include "microstrain/connections/recording/recording_connection.hpp"

namespace microstrain
{
    namespace connections
    {
        Recording::~Recording()
        {
            if (mManageReceiveStream)
            {
                recording_connection_close_receive_stream(this);
            }

            if (mManageSendStream)
            {
                recording_connection_close_send_stream(this);
            }
        }

        Recording::Recording(FILE* _receiveStream, FILE* _sendStream)
        {
            initializeStreams(_receiveStream, _sendStream);
        }

        Recording::Recording(const char* _receiveFileName, const char* _sendFileName)
        {
            openFiles(_receiveFileName, _sendFileName);
        }

        void Recording::initializeReceiveStream(FILE* _receiveStream)
        {
            recording_connection_init_receive_stream(this, _receiveStream);

            mManageReceiveStream = false;
        }

        void Recording::openReceiveFile(const char* _receiveFileName)
        {
            recording_connection_open_receive_file(this, _receiveFileName);

            mManageReceiveStream = recording_connection_receive_recording_enabled(this);
        }

        void Recording::closeReceiveFile()
        {
            recording_connection_close_receive_stream(this);
        }

        void Recording::writeReceivedBytes(const uint8_t* _receiveBuffer, const size_t _bufferSize,
            size_t* _bytesWrittenOut)
        {
            recording_connection_write_received_bytes(this, _receiveBuffer, _bufferSize, _bytesWrittenOut);
        }

        uint64_t Recording::receivedBytesWritten() const
        {
            return receive_bytes_written;
        }

        void Recording::initializeSendStream(FILE* _sendStream)
        {
            recording_connection_init_send_stream(this, _sendStream);

            mManageSendStream = false;
        }

        void Recording::openSendFile(const char* _sendFileName)
        {
            recording_connection_open_send_file(this, _sendFileName);

            mManageSendStream = recording_connection_send_recording_enabled(this);
        }

        void Recording::closeSendFile()
        {
            recording_connection_close_send_stream(this);
        }

        void Recording::writeSentBytes(const uint8_t* _sendBuffer, const size_t _bufferSize, size_t* _bytesWrittenOut)
        {
            recording_connection_write_sent_bytes(this, _sendBuffer, _bufferSize, _bytesWrittenOut);
        }

        uint64_t Recording::sentBytesWritten() const
        {
            return send_bytes_written;
        }

        void Recording::initializeStreams(FILE* _receiveStream, FILE* _sendStream)
        {
            initializeReceiveStream(_receiveStream);
            initializeSendStream(_sendStream);
        }

        void Recording::openFiles(const char* _receiveFileName, const char* _sendFileName)
        {
            openReceiveFile(_receiveFileName);
            openSendFile(_sendFileName);
        }

        void Recording::closeStreams()
        {
            recording_connection_close_streams(this);
        }

        void Recording::bytesWritten(uint64_t& _receiveBytesWrittenOut, uint64_t& _sendBytesWrittenOut) const
        {
            _receiveBytesWrittenOut = receivedBytesWritten();
            _sendBytesWrittenOut    = sentBytesWritten();
        }
    } // namespace connections
} // namespace microstrain
