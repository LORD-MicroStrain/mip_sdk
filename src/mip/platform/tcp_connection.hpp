#pragma once

#include <mip/mip_device.hpp>

#include <mip/utils/tcp_socket.h>

#include <string>


extern mip::Timestamp getCurrentTimestamp();

namespace mip {
namespace platform {

class TcpConnection : public mip::Connection
{
public:
    TcpConnection() = default;
    TcpConnection(const std::string& hostname, uint16_t port);
    ~TcpConnection();

    bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

private:
    tcp_socket mSocket;
};

} // namespace platform
} // namespace mip