#pragma once

#include <mscl/mip/mip_device.hpp>

#include <socket/tcp.hpp>

#include <string>


extern mip::Timestamp getCurrentTimestamp();


class TcpMipDevice : public mip::DeviceInterface
{
public:
    TcpMipDevice(const std::string& hostname, uint16_t port);

    bool update() final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

private:
    mip::TcpSocket mSocket;
    uint8_t mParseBuffer[1024];
};
