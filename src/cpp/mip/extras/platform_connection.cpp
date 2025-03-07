#include "mip/extras/platform_connection.hpp"

#if defined MIP_USE_SERIAL
#include <microstrain/connections/serial/serial_connection.hpp>
#endif // MIP_USE_SERIAL

#if defined MIP_USE_TCP
#include <microstrain/connections/tcp/tcp_connection.hpp>
#endif // MIP_USE_TCP

#include <microstrain/connections/connection.hpp>

#include <memory>
#include <regex>

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
        std::unique_ptr<microstrain::Connection> createConnectionFromInterfaceName(std::string interface_name, uint32_t parameter)
        {
#if defined MIP_USE_SERIAL
            // Todo: Detect USB connections (interface_name.find("ttyACM0") or similar)
            if(isSerialInterfaceName(interface_name))
                return std::make_unique<microstrain::connections::SerialConnection>(std::move(interface_name), parameter);
#endif // MIP_USE_SERIAL

#if defined MIP_USE_TCP
            if(isNetworkInterfaceName(interface_name))
                return std::make_unique<microstrain::connections::TcpConnection>(std::move(interface_name), parameter);
#endif // MIP_USE_TCP

#if !defined MIP_USE_TCP && !defined MIP_USE_SERIAL
            (void)interface_name;
            (void)parameter;
#endif // !MIP_USE_TCP && !MIP_USE_SERIAL

            return nullptr;
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Determines if the name corresponds to a serial port device.
        ///
        ///@param interface_name
        ///
        ///@returns True if the interface is likely a serial port. False otherwise.
        ///
        bool isSerialInterfaceName(const std::string_view interface_name)
        {
#if defined MICROSTRAIN_PLATFORM_WINDOWS
            return std::regex_search(interface_name.begin(), interface_name.end(), std::regex("COM", std::regex::icase));
#else // Unix
            return interface_name.rfind("/dev/", 0) == 0;
#endif // MICROSTRAIN_PLATFORM_WINDOWS
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Determines if the name corresponds to a URL or IP address.
        ///
        ///@param interface_name
        ///
        ///@returns True if the interface could be a URL or IP address. False otherwise.
        ///
        bool isNetworkInterfaceName(std::string_view interface_name)
        {
            return interface_name == "localhost" || interface_name.find('.') != std::string::npos;
        }
    } // namespace platform
} // namespace mip
