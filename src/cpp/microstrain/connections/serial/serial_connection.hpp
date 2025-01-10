#pragma once

#include "microstrain/connections/connection.hpp"

#include <microstrain/connections/serial/serial_port.h>

#include <string>

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        ///@addtogroup microstrain_platform
        ///@{

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Can be used on Windows, OSX, or linux to communicate with a MIP device
        ///       over serial
        ///
        class SerialConnection : public microstrain::Connection
        {
        public:
            static constexpr const char* TYPE = "Serial";

            SerialConnection(std::string portName, uint32_t baudrate);
            ~SerialConnection() override;

            bool recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out) final;
            bool sendToDevice(const uint8_t* data, size_t length) final;

            bool isConnected() const override;
            bool connect() override;
            bool disconnect() override;

            void connectionInfo(std::string& name, uint32_t& baudrate) const
            {
                name     = mPortName;
                baudrate = mBaudrate;
            };

            uint32_t baudrate() const { return mBaudrate; }

            bool setBaudrate(uint32_t baud);

        private:
            serial_port mPort;
            std::string mPortName;
            uint32_t    mBaudrate;

        public:
            const char* interfaceName() const override { return mPortName.c_str(); }
            uint32_t parameter() const override { return mBaudrate; }
        };

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief A serial connection but indicates that it's actually a USB connection.
        ///
        class UsbSerialConnection : public SerialConnection
        {
        public:
            static constexpr const char* TYPE = "USB";

            UsbSerialConnection(const std::string& portName, uint32_t baudrate) : SerialConnection(portName, baudrate) { mType = TYPE; }
        };

        ///@}
        ////////////////////////////////////////////////////////////////////////////////
    } // namespace connections
} // namespace microstrain
