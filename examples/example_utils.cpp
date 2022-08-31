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


std::unique_ptr<ExampleUtils> openFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port)
{
    auto example_utils = std::make_unique<ExampleUtils>();
    if(port_or_hostname.find(PORT_KEY) == std::string::npos)  // Not a serial port
    {

#ifdef MIP_USE_TCP
        uint32_t port = std::strtoul(baud_or_port.c_str(), nullptr, 10);
        if( port < 1024 || port > 65535 )
            throw std::runtime_error("Invalid TCP port (must be between 1024 and 65535.");

        example_utils->connection = std::make_shared<mip::platform::TcpConnection>(port_or_hostname, port);
        example_utils->device = std::make_shared<mip::DeviceInterface>(example_utils->connection.get(), example_utils->buffer, sizeof(example_utils->buffer), 1000, 2000);
#else  // MIP_USE_TCP
        throw std::runtime_error("This program was compiled without socket support. Recompile with -DMIP_USE_TCP=1");
#endif // MIP_USE_TCP

    }
    else  // Serial port
    {

#ifdef MIP_USE_SERIAL
        uint32_t baud = std::strtoul(baud_or_port.c_str(), nullptr, 10);
        if( baud == 0 )
            throw std::runtime_error("Serial baud rate must be a decimal integer greater than 0.");

        example_utils->connection = std::make_shared<mip::platform::SerialConnection>(port_or_hostname, baud);
        example_utils->device = std::make_shared<mip::DeviceInterface>(example_utils->connection.get(), example_utils->buffer, sizeof(example_utils->buffer), mip::C::mip_timeout_from_baudrate(baud), 500);
#else  // MIP_USE_SERIAL
        throw std::runtime_error("This program was compiled without serial support. Recompile with -DMIP_USE_SERIAL=1.\n");
#endif //MIP_USE_SERIAL
    }
    return example_utils;
}

std::unique_ptr<ExampleUtils> handleCommonArgs(int argc, const char* argv[], int maxArgs)
{
    if( argc < 3 || argc > maxArgs )
    {
        throw std::underflow_error("Usage error");
    }

    return openFromArgs(argv[1], argv[2]);
}

int printCommonUsage(const char* argv[])
{
    fprintf(stderr, "Usage: %s <portname> <baudrate>\nUsage: %s <hostname> <port>\n", argv[0], argv[0]);
    return 1;
}

