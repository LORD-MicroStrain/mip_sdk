#pragma once

#include <mip/mip_device.hpp>

#include <serial/serial.h>

#include <string>


class MipDevice : public mscl::MipDeviceInterface
{
public:
    MipDevice(const std::string& portName, uint32_t baudrate);

    bool poll() final;
    bool sendToDevice(const uint8_t* data, size_t length) final;
    bool sendToDevice(const mscl::C::MipPacket& packet) { return sendToDevice(mscl::C::MipPacket_pointer(&packet), mscl::C::MipPacket_totalLength(&packet)); }

private:
    serial::Serial mPort;
    uint8_t mParseBuffer[1024];
};
