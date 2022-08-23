#include "serial_device_interface.hpp"

#include <stdexcept>
#include <chrono>
#include <cstdio>

namespace mip
{
namespace platform
{

SerialDeviceInterface::SerialDeviceInterface(const std::string& portName, uint32_t baudrate) :
    DeviceInterface(mParseBuffer, sizeof(mParseBuffer), mip::C::mip_timeout_from_baudrate(baudrate), 500)
{
    if (!serial_port_open(&mPort, portName.c_str(), baudrate))
        throw std::runtime_error("Unable to open serial port");
}

SerialDeviceInterface::~SerialDeviceInterface()
{
    serial_port_close(&mPort);
}

bool SerialDeviceInterface::recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp)
{
    *timestamp = getCurrentTimestamp();
    return serial_port_read(&mPort, buffer, max_length, length_out);
}

bool SerialDeviceInterface::sendToDevice(const uint8_t* data, size_t length)
{
    size_t length_out;
    return serial_port_write(&mPort, data, length, &length_out);
}

};  // namespace platform
};  // namespace mip