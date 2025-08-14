#include "microstrain/connections/connection.hpp"

#include <cstring>

namespace microstrain
{
    namespace connections
    {
        void Connection::initializeReceiveRecordingStream(FILE* _receiveStream)
        {
            mRecordingConnection.initializeReceiveStream(_receiveStream);
        }

        void Connection::initializeReceiveRecordingFile(const char* _receiveFileName)
        {
            mRecordingConnection.openReceiveFile(_receiveFileName);
        }

        void Connection::closeReceiveRecordingStream()
        {
            mRecordingConnection.closeReceiveFile();
        }

        void Connection::initializeSendRecordingStream(FILE* _sendStream)
        {
            mRecordingConnection.initializeSendStream(_sendStream);
        }

        void Connection::initializeSendRecordingFile(const char* _sendFileName)
        {
            mRecordingConnection.openSendFile(_sendFileName);
        }

        void Connection::closeSendRecordingStream()
        {
            mRecordingConnection.closeSendFile();
        }

        void Connection::initializeRecordingStreams(FILE* _receiveStream, FILE* _sendStream)
        {
            mRecordingConnection.initializeStreams(_receiveStream, _sendStream);
        }

        void Connection::initializeRecordingFiles(const char* _receiveFileName, const char* _sendFileName)
        {
            mRecordingConnection.openFiles(_receiveFileName, _sendFileName);
        }

        void Connection::closeRecordingStreams()
        {
            mRecordingConnection.closeStreams();
        }

        void Connection::info(const char*& _interfaceName, uint32_t& _parameter) const
        {
            _interfaceName = interfaceName();
            _parameter     = parameter();
        }

        const char* Connection::type() const
        {
            return mType;
        }

        bool Connection::readSpan(const Span<uint8_t>& _buffer, const uint32_t _waitTimeMs,
            size_t& _bytesReadOut, EmbeddedTimestamp& _timestampOut)
        {
            return read(_buffer.data(), _buffer.size(), _waitTimeMs, _bytesReadOut, _timestampOut);
        }

        bool Connection::writeSpan(const Span<const uint8_t>& _data, size_t& _bytes_written_out)
        {
            return write(_data.data(), _data.size(), _bytes_written_out);
        }

        bool Connection::readSpanUpdate(Span<uint8_t>& _buffer, const uint32_t _wait_time_ms,
            EmbeddedTimestamp& _timestamp_out)
        {
            size_t length = 0;

            if (!read(_buffer.data(), _buffer.size(), _wait_time_ms, length, _timestamp_out))
            {
                return false;
            }

            _buffer = _buffer.first(length);

            return true;
        }

        Connection::Connection(const char* _type) :
            mType(_type)
        {}

        Connection::Connection(const char* _type, const char* _receiveRecordingFileName,
            const char* _sendRecordingFileName) :
            Connection(_type)
        {
            if (_receiveRecordingFileName && strlen(_receiveRecordingFileName) > 0)
            {
                initializeReceiveRecordingFile(_receiveRecordingFileName);
            }

            if (_sendRecordingFileName && strlen(_sendRecordingFileName) > 0)
            {
                initializeSendRecordingFile(_sendRecordingFileName);
            }
        }

        Connection::Connection(const char* _type, FILE* _receiveRecordingStream, FILE* _sendRecordingStream) :
            Connection(_type)
        {
            if (_receiveRecordingStream)
            {
                initializeReceiveRecordingStream(_receiveRecordingStream);
            }

            if (_sendRecordingStream)
            {
                initializeSendRecordingStream(_sendRecordingStream);
            }
        }

        recording_connection* Connection::recordingConnection()
        {
            return &mRecordingConnection;
        }
    } // namespace connections
} // namespace microstrain
