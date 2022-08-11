
#include "tcp_mip_device.hpp"

#include <chrono>
#include <cstdio>


TcpMipDevice::TcpMipDevice(const std::string& hostname, uint16_t port) :
    DeviceInterface(mParseBuffer, sizeof(mParseBuffer), 1000, 2000),
    mSocket(hostname, port)
{
}

bool TcpMipDevice::update()
{
    try
    {
        mip::Timestamp now = getCurrentTimestamp();
        mip::C::mip_cmd_queue_update(&cmdQueue(), now);

        return parseFromSource( [this](uint8_t* buffer, size_t maxCount, size_t* count_out, mip::Timestamp* timestamp_out)->bool
        {
            *count_out = mSocket.recv(buffer, maxCount);
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

bool TcpMipDevice::sendToDevice(const uint8_t* data, size_t length)
{
    try
    {
        mSocket.send(data, length);
        return true;
    }
    catch(const std::exception& e)
    {
        fprintf(stderr, "Error writing to port: %s\n", e.what());
        return false;
    }
}
