
#include "serial_mip_device.hpp"

#include <ctime>


mscl::Timestamp getCurrentTimestamp()
{
    // Todo: this will overflow and clamp to -1 after about 25 days.
    return std::clock() * 1000 / CLOCKS_PER_SEC;
}


MipDevice::MipDevice(const std::string& portName, uint32_t baudrate) :
    MipDeviceInterface(mParseBuffer, sizeof(mParseBuffer), mscl::C::mipTimeoutFromBaudrate(baudrate), 500),
    mPort(portName, baudrate, serial::Timeout::simpleTimeout(10))
{
}

bool MipDevice::poll()
{
    try
    {
        mscl::C::MipCmdQueue_update(cmdQueue(), getCurrentTimestamp());

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

bool MipDevice::sendToDevice(const uint8_t* data, size_t length)
{
    try
    {
        mPort.write(data, length);
    }
    catch(const std::exception& e)
    {
        fprintf(stderr, "Error writing to port: %s\n", e.what());
        return false;
    }
}
