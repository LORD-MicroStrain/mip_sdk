#pragma once

#include <mscl/mip/mip_device.hpp>

#include <serial/serial.h>

#include <string>


extern mip::Timestamp getCurrentTimestamp();


class SerialMipDevice : public mip::DeviceInterface
{
public:
    SerialMipDevice(const std::string& portName, uint32_t baudrate);

    bool update() final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

private:
    serial::Serial mPort;
    uint8_t mParseBuffer[1024];
};
