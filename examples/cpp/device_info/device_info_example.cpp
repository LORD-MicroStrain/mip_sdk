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

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.hpp>
#include <mip/mip_interface.hpp>
#include <mip/definitions/commands_base.hpp>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Common device initialization procedure
void initializeDevice(mip::Interface& _device);

// Utility functions the handle application closing and printing error messages
void terminate(microstrain::connections::Connection* _connection, const char* _message, const bool _successful = false);
void terminate(const mip::CmdResult _cmdResult, const char* _format, ...);

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&logCallback, MICROSTRAIN_LOG_LEVEL_INFO, nullptr);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    MICROSTRAIN_LOG_INFO("Connecting to the device.\n");
    // Open the connection to the device
    if (!connection.connect())
    {
        terminate(&connection, "Could not open the connection!\n");
    }

    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip::Interface device(
        connection,                         // Connection for the device
        mip::timeoutFromBaudrate(BAUDRATE), // Set the base timeout for commands (milliseconds)
        2000                                // Set the base timeout for command replies (milliseconds)
    );
    initializeDevice(device);

    // Note: The connection is cleaned up in the device destructor
    terminate(nullptr, "Example Completed Successfully.\n", true);

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
        case MICROSTRAIN_LOG_LEVEL_WARN:
        case MICROSTRAIN_LOG_LEVEL_INFO:
        case MICROSTRAIN_LOG_LEVEL_DEBUG:
        case MICROSTRAIN_LOG_LEVEL_TRACE:
        {
            fprintf(stdout, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stdout, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_OFF:
        default:
        {
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and configures a MIP device interface
///
/// @details Performs a complete device initialization sequence:
///          1. Verifies device communication with a ping command
///          2. Sets the device to idle mode to ensure reliable configuration
///          3. Queries and displays detailed device information
///
/// @param _device Reference to a MIP device interface to initialize
///
void initializeDevice(mip::Interface& _device)
{
    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    mip::CmdResult cmdResult = mip::commands_base::ping(_device);
    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmdResult = mip::commands_base::setIdle(_device);
    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
    mip::commands_base::BaseDeviceInfo deviceInfo;
    cmdResult = mip::commands_base::getDeviceInfo(_device, &deviceInfo);
    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = deviceInfo.firmware_version / 1000;
    const uint16_t minor = deviceInfo.firmware_version / 100 % 10;
    const uint16_t patch = deviceInfo.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16];
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d",
        major,
        minor,
        patch
    );

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name",             deviceInfo.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number",     deviceInfo.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number",    deviceInfo.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number",       deviceInfo.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options",          deviceInfo.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n",  "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and cleanup
///
/// @details Handles graceful shutdown when errors occur:
///          - Outputs provided error message
///          - Closes the connection if open
///          - Exits with appropriate status code
///
/// @param _connection Pointer to the connection to close
/// @param _message Error message to display
/// @param _successful Whether termination is due to success or failure
///
void terminate(microstrain::connections::Connection* _connection, const char* _message,
    const bool _successful /* = false */)
{
    if (strlen(_message) != 0)
    {
        if (_successful)
        {
            MICROSTRAIN_LOG_INFO("%s", _message);
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("%s", _message);
        }
    }

    if (_connection && _connection->isConnected())
    {
        MICROSTRAIN_LOG_INFO("Closing the connection.\n");

        if (!_connection->disconnect())
        {
            MICROSTRAIN_LOG_ERROR("Failed to close the connection!\n");
        }
    }

    MICROSTRAIN_LOG_INFO("Exiting the program.\n");

#ifdef _WIN32
    // Keep the console open on Windows
    system("pause");
#endif // _WIN32

    if (!_successful)
    {
        exit(1);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and command failure cleanup
///
/// @details Handles command failure scenarios:
///          - Formats and displays an error message with command result
///          - Exits with failure status
///
/// @param _cmdResult Result code from a failed command
/// @param _format Printf-style format string for error message
/// @param ... Variable arguments for format string
///
void terminate(const mip::CmdResult _cmdResult, const char* _format, ...)
{
    va_list args;
    va_start(args, _format);
    MICROSTRAIN_LOG_ERROR_V(_format, args);
    va_end(args);

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmdResult.value, _cmdResult.name());

    terminate(nullptr, "");
}
