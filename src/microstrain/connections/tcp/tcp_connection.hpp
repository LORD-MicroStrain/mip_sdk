#pragma once

#include <microstrain/connections/connection.hpp>

#include "tcp_socket.h"

#include <string>

namespace microstrain
{
namespace connections
{

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_platform
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Can be used on Windows, OSX, or linux to communicate with a MIP device over TCP
///
class TcpConnection : public microstrain::Connection
{
public:
    static constexpr auto TYPE = "TCP";

    TcpConnection(const std::string& hostname, uint16_t port);
    ~TcpConnection();

    TcpConnection(const TcpConnection&) = delete;
    TcpConnection& operator=(const TcpConnection&) = delete;

    bool recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time, size_t* length_out, Timestamp* timestamp_out) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

    bool isConnected() const final;
    bool connect() final;
    bool disconnect() final;

    void connectionInfo(std::string &host_name, uint32_t &port) const
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

};  // namespace microstrain
};  // namespace connections