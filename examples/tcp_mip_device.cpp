
#include "tcp_mip_device.hpp"

#include <chrono>
#include <cstdio>


TcpMipDevice::TcpMipDevice(const std::string& hostname, uint16_t port) :
    DeviceInterface(mParseBuffer, sizeof(mParseBuffer), 1000, 2000),
    mSocket(hostname, port)
{
}

bool TcpMipDevice::recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, mip::Timestamp* timestamp)
{
    try
    {
        *timestamp = getCurrentTimestamp();
        *length_out = mSocket.recv(buffer, max_length);
    }
    catch(const std::exception& e)
    {
        fprintf(stderr, "Error reading from port: %s\n", e.what());
        return false;
    }
    return true;
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
