#pragma once

#include "microstrain/connections/connection.hpp"

#include <microstrain/connections/tcp/tcp_socket.h>

#include <string>

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        ///@addtogroup microstrain_platform
        ///@{

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Can be used on Windows, OSX, or linux to communicate with a MIP
        ///       device over TCP
        ///
        class TcpConnection : public microstrain::Connection
        {
        public:
            static constexpr const char* TYPE = "TCP";

            TcpConnection() = default;
            TcpConnection(const std::string& hostname, uint16_t port);
            ~TcpConnection() override;

            TcpConnection(const TcpConnection&) = delete;
            TcpConnection& operator=(const TcpConnection&) = delete;

            bool recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out) override;
            bool sendToDevice(const uint8_t* data, size_t length) override;

            bool isConnected() const override;
            bool connect() override;
            bool disconnect() override;

            void connectionInfo(std::string& host_name, uint32_t& port) const
            {
                host_name = mHostname;
                port      = mPort;
            };

        private:
            tcp_socket mSocket;
            std::string mHostname;
            uint16_t mPort = 0;

        public:
            const char* interfaceName() const override { return mHostname.c_str(); }
            uint32_t parameter() const override { return mPort; }
        };

    ///@}
    ////////////////////////////////////////////////////////////////////////////////
    } // namespace microstrain
} // namespace connections
