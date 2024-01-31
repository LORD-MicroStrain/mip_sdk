#pragma once

#include "platform_connection.hpp"

#if MIP_USE_SERIAL
#include "serial_connection.hpp"
#endif

#if MIP_USE_TCP
#include "tcp_connection.hpp"
#endif

#include <memory>


namespace mip
{
namespace platform
{
    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Creates a connection object given the interface name and a parameter.
    ///
    ///@note This only creates the connection object; it does not open it. Call
    ///      `connection->connect()` to open it.
    ///
    ///@param interfaceName
    ///       This is the interface name - COM* (windows) or /dev/tty* (Linux) for
    ///       a serial port or anything else for a TCP socket.
    ///
    ///@param parameter
    ///       For serial ports, this is the baud rate.
    ///       For TCP sockets, this is the port number.
    ///
    std::unique_ptr<mip::Connection> createConnectionFromInterfaceName(std::string_view interface_name, uint32_t parameter)
    {
#ifdef MIP_USE_SERIAL
        // Todo: Detect USB connections (interface_name.find("ttyACM0") or similar)
        if(isSerialInterfaceName(interface_name))
            return std::make_unique<SerialConnection>(interface_name, parameter);
#endif

#ifdef MIP_USE_TCP
        if(isNetworkInterfaceName(interface_name))
            return std::make_unique<TcpConnection>(interface_name, parameter);
#endif

        return nullptr;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Determines if the name corresponds to a serial port device.
    ///
    ///@param interface_name
    ///
    ///@returns True if the interface is likely a serial port. False otherwise.
    ///
    bool isSerialInterfaceName(std::string_view interface_name)
    {
#ifdef WIN32
        return interface_name.find("COM", 0) != std::string::npos; // todo: make case insensitive
#else
        return interface_name.rfind("/dev/", 0) == 0;
#endif
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Determines if the name corresponds to a URL or IP address.
    ///
    ///@param interface_name
    ///
    ///@returns True if the interface could be a URL or IP address. False otherwise.
    ///
    bool isNetworkInterfaceName(std::string_view interface)
    {
        return interface == "localhost" || interface.find('.') != std::string::npos;
    }


} // namespace platform
} // namespace mip
