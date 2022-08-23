#include <vector>
#include <stdexcept>

#include "example_utils.hpp"


#ifdef WIN32
    #define PORT_KEY "COM"
#else
    #define PORT_KEY "/dev/"
#endif

mip::Timestamp getCurrentTimestamp()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>( steady_clock::now().time_since_epoch() ).count();
}


std::unique_ptr<mip::DeviceInterface> openDeviceFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port)
{
    std::unique_ptr<mip::DeviceInterface> device;

    if(port_or_hostname.find(PORT_KEY) == std::string::npos)  // Not a serial port
    {

#ifdef MSCL_USE_SOCKETS
        uint32_t port = std::strtoul(baud_or_port.c_str(), nullptr, 10);
        if( port < 1024 || port > 65535 )
            throw std::runtime_error("Invalid TCP port (must be between 1024 and 65535.");

        return std::make_unique<TcpMipDevice>(port_or_hostname, port);

#else  // MSCL_USE_SOCKETS
        throw std::runtime_error("This program was compiled without socket support. Recompile with -DMSCL_USE_SOCKETS=1");
#endif // MSCL_USE_SOCKETS

    }
    else  // Serial port
    {

#ifdef MSCL_USE_SERIAL
        uint32_t baud = std::strtoul(baud_or_port.c_str(), nullptr, 10);
        if( baud == 0 )
            throw std::runtime_error("Serial baud rate must be a decimal integer greater than 0.");

        return std::make_unique<mip::platform::SerialDeviceInterface>(port_or_hostname, baud);
#else  // MSCL_USE_SERIAL
        throw std::runtime_error("This program was compiled without serial support. Recompile with -DMSCL_USE_SERIAL=1.\n");
#endif // MSCL_USE_SERIAL

    }
}

std::unique_ptr<mip::DeviceInterface> handleCommonArgs(int argc, const char* argv[], int maxArgs)
{
    if( argc < 3 || argc > maxArgs )
    {
        throw std::underflow_error("Usage error");
    }

    return openDeviceFromArgs(argv[1], argv[2]);
}

int printCommonUsage(const char* argv[])
{
    fprintf(stderr, "Usage: %s <portname> <baudrate>\nUsage: %s <hostname> <port>\n", argv[0], argv[0]);
    return 1;
}

