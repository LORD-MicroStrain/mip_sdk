#pragma once

#include <mip/mip_device.hpp>

#include <memory>


class Device : public mip::DeviceInterface
{
public:
    explicit Device(mip::Timeout baseTimeout = 100) :
            mip::DeviceInterface(m_buffer, sizeof(m_buffer), 100, baseTimeout)
    {
    }

    explicit Device(const std::shared_ptr<mip::Connection>& connection, mip::Timeout baseTimeout=100) :
            mip::DeviceInterface(connection.get(), m_buffer, sizeof(m_buffer), 100, baseTimeout)
    {
        m_connection = connection;
    }

    std::shared_ptr<mip::Connection> connection() const { return m_connection; }

private:
    uint8_t m_buffer[4096] = {0};
    std::shared_ptr<mip::Connection> m_connection;
};



// Result with descriptor
struct FullResult : public mip::CmdResult
{
    mip::CompositeDescriptor descriptor;

    template<class Cmd>
    FullResult(mip::TypedResult<Cmd> result) : mip::CmdResult(result), descriptor(result.descriptor()) {}
};


