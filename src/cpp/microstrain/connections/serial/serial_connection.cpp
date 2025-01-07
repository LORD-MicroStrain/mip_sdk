#include "serial_connection.hpp"

#include <chrono>
#include <cstdio>
#include <stdexcept>

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Creates a Serial Connection that will communicate with a device over
        ///       serial
        ///
        ///@param portName Path to the port to connect to. On Windows, this usually
        ///                looks like "COM<N>", on linux, "/dev/tty<N>"
        ///
        ///@param baudrate Baud rate to open the device at. Note that the device needs
        ///                to be configured to
        ///
        SerialConnection::SerialConnection(std::string portName, uint32_t baudrate)
        {
            mPortName = std::move(portName);
            mBaudrate = baudrate;
            mType     = TYPE;

            serial_port_init(&mPort);
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Closes the underlying serial port
        ///
        SerialConnection::~SerialConnection()
        {
            SerialConnection::disconnect();
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Check if the port is connected
        ///
        bool SerialConnection::isConnected() const
        {
            return serial_port_is_open(&mPort);
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Connect to the port
        ///
        bool SerialConnection::connect()
        {
            if (serial_port_is_open(&mPort))
                return true;

           return serial_port_open(&mPort, mPortName.c_str(), mBaudrate);
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Disconnect from the port
        ///
        bool SerialConnection::disconnect()
        {
           if (!serial_port_is_open(&mPort))
                return true;

           return serial_port_close(&mPort);
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Change the baudrate
        ///
        bool SerialConnection::setBaudrate(uint32_t baud)
        {
            bool ok = serial_port_set_baudrate(&mPort, baud);

            if (ok)
                mBaudrate = baud;

            return ok;
        }

        ///@copydoc microstrain::Connection::recvFromDevice
        bool SerialConnection::recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out)
        {
            *timestamp_out = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

            return serial_port_read(&mPort, buffer, max_length, wait_time_ms, length_out);
        }

        ///@copydoc microstrain::Connection::sendToDevice
        bool SerialConnection::sendToDevice(const uint8_t* data, size_t length)
        {
            size_t length_out;
            return serial_port_write(&mPort, data, length, &length_out);
        }
    } // namespace connections
} // namespace microstrain
