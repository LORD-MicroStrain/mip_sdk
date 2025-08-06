////////////////////////////////////////////////////////////////////////////////
/// device_info_example.cpp
///
/// Example program to print device information from any MIP-enabled MicroStrain
/// device using C++
///
/// If this example does not meet your specific setup needs, please consult the
/// MIP SDK API documentation for the proper commands.
///
/// @section LICENSE
///
/// THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
/// WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
/// TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD LIABLE FOR ANY
/// DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
/// FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
/// CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
///

// Include the MicroStrain Serial connection header
#include <microstrain/connections/serial/serial_connection.hpp>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.hpp>

#include <mip/mip_interface.hpp>
#include <mip/extras/pretty_print/debug_print.hpp>
//#include <mip/extras/pretty_print/packet_printer.hpp>
//#include <mip/extras/pretty_print/basic_formatter.hpp>

#include <sstream>
#include <cstdio>
#include <csignal>

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
// Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static constexpr const char* PORT_NAME = "COM1";
#else // Unix
static constexpr const char* PORT_NAME = "/dev/ttyACM0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
static constexpr uint32_t BAUDRATE = 115200;
////////////////////////////////////////////////////////////////////////////////


#if MICROSTRAIN_LOGGING_MAX_LEVEL < MICROSTRAIN_LOG_LEVEL_INFO
// This example doesn't do anything without logging.
#error "MICROSTRAIN_LOGGING_MAX_LEVEL must be at least MICROSTRAIN_LOG_LEVEL_INFO"
#endif

void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);


void handlePacket(void*, const mip::PacketView& packet, mip::Timestamp)
{
    mip::debugPrint(packet, MICROSTRAIN_LOG_LEVEL_INFO);

    //// Create the formatter and printers.
    //// Note that this has some (relatively low) overhead, but you may wish
    //// to pass a reference to a persistent printer/formatter for other reasons.
    //std::ostringstream stream;
    //mip::printer::BasicFormatter formatter(stream);
    //mip::printer::PacketPrinter  printer(formatter);
    //
    //// Print the packet to the stream.
    //printer.formatPacket(packet);
    //
    //MICROSTRAIN_LOG_INFO("%s\n", stream.str().c_str());
}


int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&logCallback, MICROSTRAIN_LOG_LEVEL_INFO, nullptr);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the connection to the device
    if (!connection.connect())
    {
        MICROSTRAIN_LOG_FATAL("Could not open the connection!\n");
        return 1;
    }

    mip::Interface device(&connection, 100, 100);

    mip::DispatchHandler handler;
    device.registerPacketCallback<&handlePacket>(handler, mip::Dispatcher::ANY_DATA_SET, false);

    // Setup a signal handler to catch ctrl+C.
    static volatile std::sig_atomic_t stop = false;
    std::signal(SIGTERM, [](int){ stop=true; });

    while(!stop)
    {
        if(!device.update(100))
        {
            MICROSTRAIN_LOG_FATAL("Failed to read from device!\n");
            return 1;
        }
    }

    MICROSTRAIN_LOG_INFO("Received Ctrl+C, stopping.\n");

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Custom logging callback for MIP SDK message formatting and output
///
/// @details Processes and formats log messages from the MIP SDK based on
///          severity level. Routes messages to appropriate output streams -
///          errors and fatal messages go to stderr while other levels go to
///          stdout. Each message is prefixed with its severity level name.
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _level Log message severity level from microstrain_log_level enum
/// @param _format Printf-style format string for the message
/// @param _args Variable argument list containing message parameters
///
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
{
    // Unused parameter
    (void)_user;

    switch (_level)
    {
    case MICROSTRAIN_LOG_LEVEL_FATAL:
    case MICROSTRAIN_LOG_LEVEL_ERROR:
    {
        fprintf(stderr, "%s: ", microstrain_logging_level_name(_level));
        vfprintf(stderr, _format, _args);
        break;
    }
    default:
    {
        fprintf(stdout, "%s: ", microstrain_logging_level_name(_level));
        vfprintf(stdout, _format, _args);
        break;
    }
    }
}
