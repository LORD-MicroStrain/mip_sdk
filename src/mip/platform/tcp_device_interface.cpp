
#include "tcp_device_interface.hpp"

#include <stdexcept>
#include <chrono>
#include <cstdio>

namespace mip
{
namespace platform
{

TcpDeviceInterface::TcpDeviceInterface(const std::string& hostname, uint16_t port) :
    DeviceInterface(mParseBuffer, sizeof(mParseBuffer), 1000, 2000)
{
    if (!tcp_socket_open(&mSocket, hostname.c_str(), port, 3000))
        throw std::runtime_error("Unable to open TCP socket");
}

TcpDeviceInterface::~TcpDeviceInterface()
{
    tcp_socket_close(&mSocket);
}

bool TcpDeviceInterface::recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp)
{
    *timestamp = getCurrentTimestamp();
    return tcp_socket_recv(&mSocket, buffer, max_length, length_out);
}

bool TcpDeviceInterface::sendToDevice(const uint8_t* data, size_t length)
{
    size_t length_out;
    return tcp_socket_send(&mSocket, data, length, &length_out);
}

};  // namespace platform
};  // namespace mip