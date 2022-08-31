#pragma once

#include <mip/mip_device.hpp>
#include <mip/utils/serial_port.h>

#include <string>


extern mip::Timestamp getCurrentTimestamp();

namespace mip
{
namespace platform
{

class SerialConnection : public mip::Connection
{
public:
    SerialConnection() = default;
    SerialConnection(const std::string& portName, uint32_t baudrate);
    ~SerialConnection();

    bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

private:
    serial_port mPort;
};

};  // namespace platform
};  // namespace mip