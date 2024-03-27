
#include "mip_python.hpp"

#include <mip/platform/serial_connection.hpp>
#include <mip/platform/tcp_connection.hpp>

#ifdef WIN32
#define SERIAL_KEY "COM"
#else
#define SERIAL_KEY "/dev/tty"
#endif


Device* connect(const std::string& interface, uint32_t parameter)
{
    std::shared_ptr<mip::Connection> connection;
    mip::Timeout replyTimeout;

    if(interface.find(SERIAL_KEY) == 0)
    {
        connection = std::make_shared<mip::platform::SerialConnection>(interface, parameter);
        replyTimeout = 100;
    }
    else
    {
        connection = std::make_shared<mip::platform::TcpConnection>(interface, parameter);
        replyTimeout = 2000;
    }

    connection->connect();

    std::unique_ptr<Device> device(new Device(connection, replyTimeout));

    return device.release();
}

