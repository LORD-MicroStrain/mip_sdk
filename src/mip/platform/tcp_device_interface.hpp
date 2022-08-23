#pragma once

#include <mip/mip_device.hpp>

#include <mip/utils/tcp_socket.h>

#include <string>


extern mip::Timestamp getCurrentTimestamp();

namespace mip
{
namespace platform
{

class TcpDeviceInterface : public mip::DeviceInterface
{
public:
    TcpDeviceInterface(const std::string& hostname, uint16_t port);
    ~TcpDeviceInterface();

    bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

private:
    //mip::TcpSocket mSocket;
    tcp_socket mSocket;
    uint8_t mParseBuffer[1024];
};

};  // namespace platform
};  // namespace mip