#pragma once

#include <mip/mip_device.hpp>

#include <mip/utils/tcp_socket.h>

#include <string>


extern mip::Timestamp getCurrentTimestamp();


class TcpMipDevice : public mip::DeviceInterface
{
public:
    TcpMipDevice(const std::string& hostname, uint16_t port);
    ~TcpMipDevice();

    bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

private:
    //mip::TcpSocket mSocket;
    tcp_socket mSocket;
    uint8_t mParseBuffer[1024];
};
