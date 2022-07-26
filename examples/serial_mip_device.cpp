
#include "serial_mip_device.hpp"

#include <chrono>
#include <cstdio>


SerialMipDevice::SerialMipDevice(const std::string& portName, uint32_t baudrate) :
    MipDeviceInterface(mParseBuffer, sizeof(mParseBuffer), mscl::C::mip_timeout_from_baudrate(baudrate), 500),
    mPort(portName, baudrate, serial::Timeout::simpleTimeout(10))
{
}

bool SerialMipDevice::update()
{
    try
    {
        mscl::Timestamp now = getCurrentTimestamp();
        mscl::C::mip_cmd_queue_update(&cmdQueue(), now);

        return parseFromSource( [this](uint8_t* buffer, size_t maxCount, size_t* count_out, mscl::Timestamp* timestamp_out)->bool
        {
            *count_out = mPort.read(buffer, maxCount);
            *timestamp_out = getCurrentTimestamp();
            return true;
        });
    }
    catch(const std::exception& e)
    {
        fprintf(stderr, "Error reading from port: %s\n", e.what());
        return false;
    }
}

bool SerialMipDevice::sendToDevice(const uint8_t* data, size_t length)
{
    try
    {
        mPort.write(data, length);
        return true;
    }
    catch(const std::exception& e)
    {
        fprintf(stderr, "Error writing to port: %s\n", e.what());
        return false;
    }
}
