////////////////////////////////////////////////////////////////////////////////
/// recording_example.cpp
///
/// Example program to record factory streaming from any MIP-enabled MicroStrain
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

// Include the MicroStrain timestamping header
#include <microstrain/embedded_time.hpp>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.hpp>
#include <mip/mip_interface.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>

#include <atomic>
#include <csignal>
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

// Switch between initializing the recording through the connection constructor or after construction
// Note: Set to 0 to initialize after construction instead
#define CONNECTION_RECORDING_WRAPPERS 1

// Create a user-managed recording stream instead of delegating it to the connection interface
// Note: Set to 0 to show user-managed stream usage
#define USER_RECORDING_STREAMS 0

// TODO: Update with desired recording file names
// Note: Streams may also be used in place of files
static const char* RECEIVED_BYTES_BINARY = "received_bytes.bin";
static const char* SENT_BYTES_BINARY     = "sent_bytes.bin";
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Utility function to get supported data descriptors for the device
void getSupportedDataDescriptors(const uint16_t* _supportedDescriptors, const uint8_t _supportedDescriptorCount,
    uint8_t* _supportedDataDescriptorsOut, const uint8_t _supportedDataDescriptorsSize);

// Common device initialization procedure
void initializeDevice(mip::Interface& _device);

// Stop the application on signal interrupt
void signalInterruptHandler(const int _signalValue);

// Utility functions the handle application closing and printing error messages
void terminate(microstrain::connections::Connection* _connection, const char* _message, const bool _successful = false);
void terminate(const mip::CmdResult _cmdResult, const char* _format, ...);

// Running state of the application
std::atomic<bool> gRunning(true);

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&logCallback, MICROSTRAIN_LOG_LEVEL_INFO, nullptr);

    // Configure the signal interrupt handler to terminate the application
    if (signal(SIGINT, signalInterruptHandler) == SIG_ERR)
    {
        terminate(nullptr, "Failed to set signal handler!\n", false);
    }

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the recording files for the connection
    MICROSTRAIN_LOG_INFO("Opening connection recording files. Receive: '%s'    Send: '%s'\n",
        RECEIVED_BYTES_BINARY,
        SENT_BYTES_BINARY
    );

#if USER_RECORDING_STREAMS
    // Manual stream creation and management
    FILE* receiveStream = fopen(RECEIVED_BYTES_BINARY, "wb");
    FILE* sendStream    = fopen(SENT_BYTES_BINARY, "wb");
#endif // USER_RECORDING_STREAMS

#if CONNECTION_RECORDING_WRAPPERS
#if USER_RECORDING_STREAMS
    // Create the connection with user-managed streams
    // Note: User-managed streams can also be closed through the connection interface
    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE, receiveStream, sendStream);
#else // Connection manages the streams
    // Create the connection, and it opens and manages the recording streams
    // Note: Connection managed streams are closed in the connection destructor
    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE, RECEIVED_BYTES_BINARY, SENT_BYTES_BINARY);
#endif // USER_RECORDING_STREAMS
#else // Manual initialization
    // Create the connection first
    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    // Initialize the recording after
#if USER_RECORDING_STREAMS
    // Initialize user-managed streams
    // Note: User-managed streams can also be closed through the connection interface
    connection.initializeRecordingStreams(receiveStream, sendStream);
#else // Connection manages the streams
    // Initialize connection managed streams
    // Note: Connection managed streams are closed in the connection destructor
    connection.initializeRecordingFiles(RECEIVED_BYTES_BINARY, SENT_BYTES_BINARY);
#endif // USER_RECORDING_STREAMS
#endif // CONNECTION_RECORDING_WRAPPERS

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

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    const mip::CmdResult cmdResult = mip::commands_base::resume(device);
    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... recording data.\n");
    MICROSTRAIN_LOG_INFO("Press Ctrl+C to stop recording and exit.\n");

    // Previous print time for the recording status message
    microstrain::EmbeddedTimestamp previousPrintTime = microstrain::getCurrentTimestamp();
    uint8_t ellipsesCount = 0;

    // Display a recording status
    MICROSTRAIN_LOG_INFO("Recording");

    // Device loop
    // Exit after a signal interrupt
    while (gRunning.load())
    {
        // Update the device state
        // Note: This will update the device callbacks
        device.update();

        // Redraw ellipses after the recording message
        if (microstrain::getCurrentTimestamp() - previousPrintTime >= 750)
        {
            previousPrintTime = microstrain::getCurrentTimestamp();

            if (++ellipsesCount == 4)
            {
                // Move the cursor before the ellipses and clear it
                printf("\033[3D\033[0K");
            }
            else
            {
                // Print the ellipses
                printf(".");
            }

            // Reset the ellipses
            ellipsesCount %= 4;
        }
    }

    // New line for terminating the application
    printf("\n");

#if USER_RECORDING_STREAMS
    // Note: Even if streams are user-managed, recording and connection interfaces can still be used to close the
    // streams using 'recording.closeStreams()' and 'connection.closeRecordingStreams()'
    fclose(receiveStream);
    fclose(sendStream);
#endif // USER_RECORDING_STREAMS

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
/// @brief Extracts supported data descriptor sets from a list of device
///        descriptors
///
/// @details Parses a list of device descriptors and extracts only the data
///          descriptor sets (i.e., excludes field descriptors). The function
///          filters for unique data descriptors and stores them in the provided
///          output array. Processing stops when a zero descriptor is
///          encountered or when the output array reaches capacity.
///
/// @param _supportedDescriptors Array of supported composite descriptors from
///                              the device
/// @param _supportedDescriptorCount Number of descriptors in the input array
/// @param _supportedDataDescriptorsOut Output array for unique data descriptors
/// @param _supportedDataDescriptorsSize Maximum capacity of the output array
///
void getSupportedDataDescriptors(const uint16_t* _supportedDescriptors, const uint8_t _supportedDescriptorCount,
    uint8_t* _supportedDataDescriptorsOut, const uint8_t _supportedDataDescriptorsSize)
{
    uint8_t lastIndex = 0;

    for (uint16_t descriptorIndex = 0; descriptorIndex < _supportedDescriptorCount; ++descriptorIndex)
    {
        // Extract the descriptor set from the composite descriptor
        const uint8_t supported_descriptor_set = (_supportedDescriptors[descriptorIndex] & 0xFF00) >> 8;

        // Initialized the array with 0 can break on a 0 descriptor
        if (supported_descriptor_set == 0)
        {
            break;
        }

        // Store supported data descriptors
        if (mip::isDataDescriptorSet(supported_descriptor_set))
        {
            // Only add unique descriptors
            if (lastIndex == 0 || _supportedDataDescriptorsOut[lastIndex - 1] != supported_descriptor_set)
            {
                _supportedDataDescriptorsOut[lastIndex] = supported_descriptor_set;

                ++lastIndex;

                assert(lastIndex <= _supportedDataDescriptorsSize);

                // Cannot add more descriptors (increase array size)
                if (lastIndex == _supportedDataDescriptorsSize)
                {
                    break;
                }
            }
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
///          4. Loads default device settings for a known state
///          5. Enables factory data streaming
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

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    MICROSTRAIN_LOG_INFO("Loading default settings.\n");
    cmdResult = mip::commands_3dm::defaultDeviceSettings(_device);
    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not load %s!\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    }

    // Get supported descriptors to check if certain data descriptors are supported for streaming
    MICROSTRAIN_LOG_INFO("Getting supported descriptors for the device.\n");
    uint8_t  descriptorsCount          = 0;
    uint16_t supportedDescriptors[256] = { 0 };
    cmdResult = mip::commands_base::getDeviceDescriptors(
        _device,
        supportedDescriptors,                                           // Descriptors array from the device
        sizeof(supportedDescriptors) / sizeof(supportedDescriptors[0]), // Max array size
        &descriptorsCount                                               // Descriptor count returned from the device
    );

    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not get supported descriptors!\n");
    }

    // Some devices have a large number of descriptors
    // The extended descriptors command can get the remaining descriptors
    MICROSTRAIN_LOG_INFO("Getting extended supported descriptors for the device.\n");
    uint8_t extendedDescriptorsCount = 0;
    cmdResult = mip::commands_base::getExtendedDescriptors(
        _device,
        &supportedDescriptors[descriptorsCount],                                           // Append to the existing array
        sizeof(supportedDescriptors) / sizeof(supportedDescriptors[0]) - descriptorsCount, // Remaining size of the array
        &extendedDescriptorsCount
    );

    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not get extended supported descriptors!\n");
    }

    uint8_t           supportedDataDescriptors[16] = { 0 };
    constexpr uint8_t supportedDataDescriptorSize  =
        sizeof(supportedDataDescriptors) / sizeof(supportedDataDescriptors[0]);

    getSupportedDataDescriptors(supportedDescriptors, descriptorsCount + extendedDescriptorsCount,
        supportedDataDescriptors, supportedDataDescriptorSize);

    MICROSTRAIN_LOG_INFO("Enabling factory support data streaming.\n");
    cmdResult = mip::commands_3dm::factoryStreaming(_device, mip::commands_3dm::FactoryStreaming::Action::OVERWRITE, 0);

    if (!cmdResult.isAck())
    {
        terminate(cmdResult, "Could not enable factory support data streaming!\n");
    }

    // Enable streaming for supported data descriptors
    for (uint8_t dataDescriptorIndex = 0; dataDescriptorIndex < supportedDataDescriptorSize; ++dataDescriptorIndex)
    {
        // Any 0 value was from initialization
        if (supportedDataDescriptors[dataDescriptorIndex] == 0)
        {
            break;
        }

        cmdResult = mip::commands_3dm::writeDatastreamControl(_device, supportedDataDescriptors[dataDescriptorIndex], true);

        if (!cmdResult.isAck())
        {
            terminate(
                cmdResult,
                "Could not enable streaming for data descriptor 0x%02X!\n",
                supportedDataDescriptors[dataDescriptorIndex]
            );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Signal interrupt callback handler
///
/// @details Handles the signal interrupt callback and sets the application
///          running flag to false (0)
///
/// @param _signalValue Value of the signal (unused in this implementation)
///
void signalInterruptHandler(const int _signalValue)
{
    // Unused parameter
    (void)_signalValue;

    // Stop the application
    gRunning.store(false);
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
