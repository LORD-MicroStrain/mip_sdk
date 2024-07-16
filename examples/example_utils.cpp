#include <vector>
#include <stdexcept>

#include <stdarg.h>

#include "microstrain/common/logging.h"

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

void customLog(void* user, microstrain_log_level level, const char* fmt, va_list args)
{
    // Convert the varargs into a string
    std::string log;
    va_list args_copy;
    va_copy(args_copy, args);
    const int required_len = vsnprintf(nullptr, 0, fmt, args_copy);
    if (required_len >= 0)
    {
        log.resize(required_len);
        vsnprintf(&log[0], required_len + 1, fmt, args);
    }
    va_end(args_copy);

    // Print to the proper stream
    switch (level)
    {
        case MICROSTRAIN_LOG_LEVEL_FATAL:
        case MICROSTRAIN_LOG_LEVEL_ERROR:
            std::cerr << log;
            break;
        default:
            std::cout << log;
            break;
    }
}

std::unique_ptr<ExampleUtils> openFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port, const std::string& binary_file_path)
{
    auto example_utils = std::make_unique<ExampleUtils>();

    if( !binary_file_path.empty() )
    {
#ifdef MIP_USE_EXTRAS
        example_utils->recordedFile = std::make_unique<std::ofstream>(binary_file_path);
        if( !example_utils->recordedFile->is_open() )
            throw std::runtime_error("Unable to open binary file");
#else  // MIP_USE_EXTRAS
        throw std::runtime_error("The program was compiled without binary file recording support. Recompile with -DMIP_USE_EXTRAS=ON");
#endif  // MIP_USE_EXTRAS
    }

    if(port_or_hostname.find(PORT_KEY) == std::string::npos)  // Not a serial port
    {

#ifdef MIP_USE_TCP
        uint32_t port = std::strtoul(baud_or_port.c_str(), nullptr, 10);
        if( port < 1024 || port > 65535 )
            throw std::runtime_error("Invalid TCP port (must be between 1024 and 65535.");

#ifdef MIP_USE_EXTRAS
        using RecordingTcpConnection = microstrain::connections::RecordingConnectionWrapper<microstrain::connections::TcpConnection>;
        example_utils->connection = std::make_unique<RecordingTcpConnection>(example_utils->recordedFile.get(), example_utils->recordedFile.get(), port_or_hostname, port);
#else  // MIP_USE_EXTRAS
        using TcpConnection = microstrain::connections::TcpConnection;
        example_utils->connection = std::make_unique<TcpConnection>(port_or_hostname, port);
#endif  // MIP_USE_EXTRAS

        example_utils->device = std::make_unique<mip::Interface>(example_utils->connection.get(), example_utils->buffer, sizeof(example_utils->buffer), 1000, 2000);
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

#ifdef MIP_USE_EXTRAS
        using RecordingSerialConnection = microstrain::connections::RecordingConnectionWrapper<microstrain::connections::SerialConnection>;
        example_utils->connection = std::make_unique<RecordingSerialConnection>(example_utils->recordedFile.get(), example_utils->recordedFile.get(), port_or_hostname, baud);
#else  // MIP_USE_EXTRAS
        using SerialConnection = microstrain::connections::SerialConnection;
        example_utils->connection = std::make_unique<SerialConnection>(port_or_hostname, baud);
#endif  // MIP_USE_EXTRAS

        example_utils->device = std::make_unique<mip::Interface>(example_utils->connection.get(), example_utils->buffer, sizeof(example_utils->buffer), mip::C::mip_timeout_from_baudrate(baud), 500);
#else  // MIP_USE_SERIAL
        throw std::runtime_error("This program was compiled without serial support. Recompile with -DMIP_USE_SERIAL=1.\n");
#endif //MIP_USE_SERIAL
    }

    if( !example_utils->connection->connect() )
        throw std::runtime_error("Failed to open the connection");

    return example_utils;
}

std::unique_ptr<ExampleUtils> handleCommonArgs(int argc, const char* argv[], int maxArgs)
{
    // Setup the logger for the MIP SDK
    MIP_LOG_INIT(&customLog, MICROSTRAIN_LOG_LEVEL_DEBUG, nullptr);

    if( argc < 3 || argc > maxArgs )
    {
        throw std::underflow_error("Usage error");
    }

    // If we were passed a file name, record the data in that file
    std::string binary_file_path = "";
    if (argc >= 4)
        binary_file_path = argv[3];

    return openFromArgs(argv[1], argv[2], binary_file_path);
}

int printCommonUsage(const char* argv[])
{
    fprintf(stderr, "Usage: %s <portname> <baudrate> <binaryfile>\nUsage: %s <hostname> <port> <binaryfile>\n", argv[0], argv[0]);
    return 1;
}

void displayFilterState(const mip::data_filter::FilterMode &filterState, std::string &currentState, bool isFiveSeries) {
    std::string read_state = "";
    switch (filterState) 
    {
        case mip::data_filter::FilterMode::INIT:
            read_state = (isFiveSeries ? "GX5_INIT" : "INIT") + std::string(" (1)");
            break;
        case mip::data_filter::FilterMode::VERT_GYRO:
            read_state = (isFiveSeries ? "GX5_RUN_SOLUTION_VALID" : "VERT_GYRO") + std::string(" (2)");
            break;
        case mip::data_filter::FilterMode::AHRS:
            read_state = (isFiveSeries ? "GX5_RUN_SOLUTION_ERROR" : "AHRS") + std::string(" (3)");
            break;
        case mip::data_filter::FilterMode::FULL_NAV:
            read_state = "FULL_NAV (4)";
            break;
        default:
            read_state = "STARTUP (0)";
            break;
    }

    if (read_state != currentState) 
    {
        printf("Filter state: %s\n", read_state.data());
        currentState = read_state;
    }
}
