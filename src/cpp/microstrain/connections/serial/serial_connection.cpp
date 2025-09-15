#include "microstrain/connections/serial/serial_connection.hpp"

namespace microstrain
{
    namespace connections
    {
        SerialConnection::~SerialConnection()
        {
            SerialConnection::disconnect();
        }

        SerialConnection::SerialConnection(const char* _portName, const uint32_t _baudrate) :
            Connection(TYPE)
        {
            serial_port_init(this, _portName, _baudrate, recordingConnection());
        }

        SerialConnection::SerialConnection(const char* _portName, const uint32_t _baudrate,
            const char* _receiveRecordingFileName, const char* _sendRecordingFileName) :
            Connection(TYPE, _receiveRecordingFileName, _sendRecordingFileName)
        {
            serial_port_init(this, _portName, _baudrate, recordingConnection());
        }

        SerialConnection::SerialConnection(const char* _portName, const uint32_t _baudrate,
            FILE* _receiveRecordingStream, FILE* _sendRecordingStream) :
            Connection(TYPE, _receiveRecordingStream, _sendRecordingStream)
        {
            serial_port_init(this, _portName, _baudrate, recordingConnection());
        }

        bool SerialConnection::connect()
        {
           return serial_port_open(this);
        }

        bool SerialConnection::disconnect()
        {
           return serial_port_close(this);
        }

        bool SerialConnection::isConnected() const
        {
            return serial_port_is_open(this);
        }

        bool SerialConnection::read(uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time_ms,
            size_t& _bytes_read_out, EmbeddedTimestamp& _timestamp_out)
        {
            return serial_port_read(this, _buffer, _byte_count, _wait_time_ms, &_bytes_read_out, &_timestamp_out);
        }

        bool SerialConnection::write(const uint8_t* _data, const size_t _byte_count, size_t& _bytes_written_out)
        {
            return serial_port_write(this, _data, _byte_count, &_bytes_written_out);
        }

        const char* SerialConnection::interfaceName() const
        {
            return port_name;
        }

        uint32_t SerialConnection::parameter() const
        {
            return baudrate();
        }

        uint32_t SerialConnection::baudrate() const
        {
            return serial_port::baudrate;
        }

        bool SerialConnection::updateBaudrate(const uint32_t _baudrate)
        {
            return serial_port_update_baudrate(this, _baudrate);
        }

        UsbSerialConnection::UsbSerialConnection(const char* _portName, const uint32_t _baudrate) :
            SerialConnection(_portName, _baudrate)
        {
            mType = TYPE;
        }

        UsbSerialConnection::UsbSerialConnection(const char* _portName, const uint32_t _baudrate,
            const char* _receiveRecordingFileName, const char* _sendRecordingFileName) :
            SerialConnection(_portName, _baudrate, _receiveRecordingFileName, _sendRecordingFileName)
        {
            mType = TYPE;
        }

        UsbSerialConnection::UsbSerialConnection(const char* _portName, const uint32_t _baudrate,
            FILE* _receiveRecordingStream, FILE* _sendRecordingStream) :
            SerialConnection(_portName, _baudrate, _receiveRecordingStream, _sendRecordingStream)
        {
            mType = TYPE;
        }
    } // namespace connections
} // namespace microstrain
