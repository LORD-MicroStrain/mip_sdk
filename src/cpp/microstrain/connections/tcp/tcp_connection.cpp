#include "microstrain/connections/tcp/tcp_connection.hpp"

namespace microstrain
{
    namespace connections
    {
        TcpClientConnection::~TcpClientConnection()
        {
            TcpClientConnection::disconnect();
        }

        TcpClientConnection::TcpClientConnection(const char* _hostname, const uint16_t _port,
            const uint32_t _timeoutMs /* = 3000 */) :
            Connection(TYPE)
        {
            tcp_client_init(this, _hostname, _port, recordingConnection());

            timeout = _timeoutMs;
        }

        TcpClientConnection::TcpClientConnection(const char* _hostname, const uint16_t _port,
            const char* _receiveRecordingFileName, const char* _sendRecordingFileName,
            const uint32_t _timeoutMs /* = 3000 */) :
            Connection(TYPE, _receiveRecordingFileName, _sendRecordingFileName)
        {
            tcp_client_init(this, _hostname, _port, recordingConnection());

            timeout = _timeoutMs;
        }

        TcpClientConnection::TcpClientConnection(const char* _hostname, const uint16_t _port,
            FILE* _receiveRecordingStream, FILE* _sendRecordingStream, const uint32_t _timeoutMs /* = 3000 */) :
            Connection(TYPE, _receiveRecordingStream, _sendRecordingStream)
        {
            tcp_client_init(this, _hostname, _port, recordingConnection());

            timeout = _timeoutMs;
        }

        bool TcpClientConnection::connect()
        {
           return tcp_client_open(this, timeout);
        }

        bool TcpClientConnection::disconnect()
        {
            return tcp_client_close(this);
        }

        bool TcpClientConnection::isConnected() const
        {
            return tcp_client_is_open(this);
        }

        bool TcpClientConnection::read(uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time_ms,
            size_t& _bytes_read_out, EmbeddedTimestamp& _timestamp_out)
        {
            return tcp_client_read(this, _buffer, _byte_count, _wait_time_ms, &_bytes_read_out, &_timestamp_out);
        }

        bool TcpClientConnection::write(const uint8_t* _data, const size_t _byte_count, size_t& _bytes_written_out)
        {
            return tcp_client_write(this, _data, _byte_count, &_bytes_written_out);
        }

        const char* TcpClientConnection::interfaceName() const
        {
            return hostname();
        }

        uint32_t TcpClientConnection::parameter() const
        {
            return port();
        }

        const char* TcpClientConnection::hostname() const
        {
            return tcp_client::hostname;
        }

        uint16_t TcpClientConnection::port() const
        {
            return tcp_client::port;
        }

        bool TcpClientConnection::updatePort(const uint16_t _port)
        {
            return tcp_client_update_port(this, _port);
        }
    } // namespace connections
} // namespace microstrain
