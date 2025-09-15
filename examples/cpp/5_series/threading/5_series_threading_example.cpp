////////////////////////////////////////////////////////////////////////////////
/// 5_series_threading_example.cpp
///
/// Example multithreading program for 5-series devices using C++
///
/// This example shows a basic setup for 5-series devices to demonstrate
/// multithreading for data collection and command updates using C++.
/// This is not an exhaustive example of all settings for those devices.
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

#ifndef MICROSTRAIN_ENABLE_LOGGING
#error This example requires logging to be enabled
#endif // !MICROSTRAIN_ENABLE_LOGGING

#if !defined MICROSTRAIN_LOGGING_MAX_LEVEL || MICROSTRAIN_LOGGING_MAX_LEVEL < MICROSTRAIN_LOG_LEVEL_INFO
#error This example requires a logging level of at least MICROSTRAIN_LOG_LEVEL_INFO
#endif // MICROSTRAIN_LOGGING_MAX_LEVEL < MICROSTRAIN_LOG_LEVEL_INFO

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.hpp>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.hpp>
#include <mip/mip_interface.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/data_sensor.hpp>

#include <chrono>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
// Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static constexpr const char* PORT_NAME = "COM1";
#else  // Unix
static constexpr const char* PORT_NAME = "/dev/ttyACM0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
// Note: For native serial connections this needs to be 115200 due to the device default settings command
// Use mip_3dm_*_uart_baudrate() to write and save the baudrate on the device
static constexpr uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
// Streaming rate in Hz
static constexpr uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
// Example run time
static constexpr uint32_t RUN_TIME_SECONDS = 30;

// TODO: Enable/disable data collection threading
// Use this to test the behaviors of threading
#define USE_THREADS true
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
mip::Timestamp getCurrentTimestamp();

// Common device initialization procedure
void initializeDevice(mip::Interface& _device);

// Message format configuration
void configureSensorMessageFormat(mip::Interface& _device);

// Packet callback handler
void packetCallback(void* _user, const mip::PacketView& _packetView, mip::Timestamp _timestamp);

#if USE_THREADS
// Threaded functions
bool updateDevice(mip::Interface& _device, mip::Timeout _waitTime, bool _fromCmd);
void dataCollectionThread(mip::Interface& _device, const volatile bool& _running);
#endif // USE_THREADS

// Utility functions the handle application closing and printing error messages
void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful = false);
void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...);

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
        terminate(&connection, "Could not open the connection!\n");
    }

    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip::Interface device(
        &connection,                                 // Connection for the device
        mip::C::mip_timeout_from_baudrate(BAUDRATE), // Set the base timeout for commands (milliseconds)
        2000                                         // Set the base timeout for command replies (milliseconds)
    );
    initializeDevice(device);

    // Configure the message format for sensor data
    configureSensorMessageFormat(device);

    // Sensor data packet callback
    MICROSTRAIN_LOG_INFO("Registering a sensor data packet callback.\n");

    mip::DispatchHandler packetHandler;

    // Register the callback for packets
    device.registerPacketCallback<&packetCallback>(
        packetHandler,
        mip::data_sensor::DESCRIPTOR_SET, // Data descriptor set
        false,                            // Process after field callback
        nullptr                           // User data
    );

#if USE_THREADS
    MICROSTRAIN_LOG_INFO("Initializing the device update function for threading.\n");
    // Note: This allows the update function to be split into command and data updates across multiple threads
    device.setUpdateFunctionFree<&updateDevice>();

    // Data collection thread run flag
    volatile bool running = true;

    MICROSTRAIN_LOG_INFO("Creating the data collection thread.\n");
    std::thread dataThread(dataCollectionThread, std::ref(device), std::ref(running));
#endif // USE_THREADS

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    const mip::CmdResult cmdResult = mip::commands_base::resume(device);

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("The device is configured... waiting for data.\n");

    // Get the start time of the device update loop to handle exiting the application
    const mip::Timestamp loopStartTime = getCurrentTimestamp();

    // Running loop
    // Exit after a predetermined time in seconds
    while (getCurrentTimestamp() - loopStartTime <= RUN_TIME_SECONDS * 1000)
    {
        // Stress testing the device with ping
        // This attempts to trigger race conditions across threads
        // Note: Only one thread at a time can safely send commands
        MICROSTRAIN_LOG_WARN("Running device stress test!\n");
        for (uint8_t counter = 0; counter < 100; ++counter)
        {
            // Note: Sending commands calls the device update function every time
            mip::commands_base::ping(device);
        }
    }

#if USE_THREADS
    // Signal the data collection thread to stop
    running = false;

    // Join the thread back before exiting the program
    MICROSTRAIN_LOG_INFO("Waiting for the thread to join.\n");
    dataThread.join();
#endif // USE_THREADS

    terminate(&connection, "Example Completed Successfully.\n", true);

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
            fflush(stderr);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_WARN:
        case MICROSTRAIN_LOG_LEVEL_INFO:
        case MICROSTRAIN_LOG_LEVEL_DEBUG:
        case MICROSTRAIN_LOG_LEVEL_TRACE:
        {
            fprintf(stdout, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stdout, _format, _args);
            fflush(stdout);
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
/// @brief Gets the current system timestamp in milliseconds
///
/// @details Provides system time measurement using std::chrono for milliseconds
///          since epoch. Uses system_clock to get wall-clock time that
///          corresponds to calendar time and can be synchronized with external
///          time sources.
///
/// @note Update this function to use a different time source if needed for
///       your specific application requirements
///
/// @return Current timestamp in milliseconds since epoch
///
mip::Timestamp getCurrentTimestamp()
{
    const std::chrono::nanoseconds timeSinceEpoch = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<mip::Timestamp>(std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceEpoch).count());
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and configures a MIP device interface
///
/// @details Performs a complete device initialization sequence:
///          1. Verifies device communication with a ping command
///          2. Sets the device to idle mode to ensure reliable configuration
///          3. Queries and displays detailed device information
///          4. Loads default device settings for a known state
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
        terminate(_device, cmdResult, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmdResult = mip::commands_base::setIdle(_device);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
    mip::commands_base::BaseDeviceInfo deviceInfo;
    cmdResult = mip::commands_base::getDeviceInfo(_device, &deviceInfo);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = deviceInfo.firmware_version / 1000;
    const uint16_t minor = deviceInfo.firmware_version / 100 % 10;
    const uint16_t patch = deviceInfo.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16];
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d", major, minor, patch);

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name", deviceInfo.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number", deviceInfo.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number", deviceInfo.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number", deviceInfo.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options", deviceInfo.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n", "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    MICROSTRAIN_LOG_INFO("Loading %s.\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    cmdResult = mip::commands_3dm::defaultDeviceSettings(_device);

    if (!cmdResult.isAck())
    {
        // Note: Default settings will reset the baudrate to 115200 and may cause connection issues
        if (cmdResult == mip::CmdResult::STATUS_TIMEDOUT && BAUDRATE != 115200)
        {
            MICROSTRAIN_LOG_WARN(
                "On a native serial connections the baudrate needs to be 115200 for this example to run.\n"
            );
        }

        terminate(_device, cmdResult, "Could not load %s!\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for sensor data streaming
///
/// @details Sets up sensor data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - Scaled accelerometer
///
/// @param _device Reference to the initialized MIP device interface
///
void configureSensorMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t       sensorBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::imuGetBaseRate(
        _device,
        &sensorBaseRate // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get the base rate for sensor data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > sensorBaseRate)
    {
        terminate(
            _device,
            mip::CmdResult::NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            sensorBaseRate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t sensorDecimation = sensorBaseRate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating sensor base rate %d by %d to stream data at %dHz.\n",
        sensorBaseRate,
        sensorDecimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip::DescriptorRate sensorDescriptors[1] = {
        { mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR, sensorDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for sensor data.\n", mip::commands_3dm::ImuMessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeImuMessageFormat(
        _device,
        sizeof(sensorDescriptors) / sizeof(sensorDescriptors[0]), // Number of descriptors to include
        sensorDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not configure %s for sensor data!\n",
            mip::commands_3dm::ImuMessageFormat::DOC_NAME
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Callback function that processes received MIP packets
///
/// @details This function is called whenever a MIP packet is received from the
///          device.
///          It processes the packet by:
///          1. Extracting all fields from the packet
///          2. Building a formatted string of field descriptors
///          3. Logging packet information including timestamp, descriptor set,
///             and field descriptors
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _packetView Reference to the received MIP packet
/// @param _timestamp Timestamp when the packet was received
///
void packetCallback(void* _user, const mip::PacketView& _packetView, mip::Timestamp _timestamp)
{
    // Unused parameter
    (void)_user;

    // Create a buffer for printing purposes
    char fieldDescriptorsBuffer[255] = { 0 };
    int  bufferOffset                = 0;

    // Iterate the packet and extract each field
    for (const mip::FieldView& fieldView : _packetView)
    {
        bufferOffset += snprintf(
            &fieldDescriptorsBuffer[bufferOffset],
            sizeof(fieldDescriptorsBuffer) / sizeof(fieldDescriptorsBuffer[0]) - bufferOffset,
            " 0x%02X,",
            fieldView.fieldDescriptor()
        );
    }

    // Trim off the last comma
    if (bufferOffset > 0)
    {
        fieldDescriptorsBuffer[bufferOffset - 1] = '\0';
    }

    MICROSTRAIN_LOG_INFO(
        "Received a packet at %" PRIu64 " with descriptor set 0x%02X:%s\n",
        _timestamp,
        _packetView.descriptorSet(),
        fieldDescriptorsBuffer
    );
}

#if USE_THREADS
////////////////////////////////////////////////////////////////////////////////
/// @brief Updates the device state based on command or data collection context
///
/// @details Handles device updates differently depending on whether called from
///          command handling or data collection:
///          - For data collection (_fromCmd = false): Performs normal device
///            updates
///          - For commands (_fromCmd = true): Sleeps briefly to save power
///            while avoiding command timeouts
///
/// @param _device Pointer to the MIP device interface
/// @param _waitTime Time to wait for updates (typically used only from
///                   commands)
/// @param _fromCmd True if called from command handling, false for data
///                  collection
///
/// @returns true if the update was successful, false on error.
///          Always returns true when called from commands to avoid race
///          conditions.
///
bool updateDevice(mip::Interface& _device, mip::Timeout _waitTime, bool _fromCmd)
{
    // Do normal updates only if not called from a command handler
    // Note: This is the separation between the main/other thread and the data collection thread
    if (!_fromCmd)
    {
        return _device.defaultUpdate(_waitTime, true);
    }

    // Sleep for a bit to save power
    // Note: Waiting too long in here will cause commands to timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Note: This needs to return true to avoid terminating the data collection thread
    // Note: Returning false may cause a race condition (see comments in mip_interface_wait_for_reply)
    return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles continuous data collection from the device in a separate
///        thread
///
/// @details Main function for the data collection thread that:
///          1. Continuously updates device state for receiving data
///          2. Clears command queue if the connection closes to avoid
///             deadlocks
///          3. Yields thread time when possible
///          4. Runs until the running state becomes false
///
/// @param _device Reference to the MIP device interface
/// @param _running Reference to volatile boolean controlling thread execution
///
void dataCollectionThread(mip::Interface& _device, const volatile bool& _running)
{
    MICROSTRAIN_LOG_INFO("Data collection thread created!\n");

    while (_running)
    {
        // Update the device for data collection
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        const bool updated = _device.update(
            10,   // Time to wait
            false // From command
        );

        // Clean up and exit the thread on failed device updates
        if (!updated)
        {
            // Avoid deadlocks if the connection is closed
            _device.cmdQueue().clear();

            break;
        }

        std::this_thread::yield();
    }
}
#endif // USE_THREADS

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and cleanup
///
/// @details Handles graceful shutdown when errors occur:
///          - Outputs provided error message
///          - Closes device connection if open
///          - Exits with appropriate status code
///
/// @param _connection Pointer to the device connection to close
/// @param _message Error message to display
/// @param _successful Whether termination is due to success or failure
///
void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful /* = false */)
{
    if (_message && strlen(_message) != 0)
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

    if (!_connection)
    {
        // Create the device interface with a connection or set it after creation
        MICROSTRAIN_LOG_ERROR("Connection not set for the device interface. Cannot close the connection.\n");
    }
    else
    {
        if (_connection->isConnected())
        {
            MICROSTRAIN_LOG_INFO("Closing the connection.\n");

            if (!_connection->disconnect())
            {
                MICROSTRAIN_LOG_ERROR("Failed to close the connection!\n");
            }
        }
    }

    MICROSTRAIN_LOG_INFO("Press 'Enter' to exit the program.\n");

    // Make sure the console remains open
    const int confirmExit = getc(stdin);
    (void)confirmExit; // Unused

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
///          - Closes device connection
///          - Exits with failure status
///
/// @param _device MIP device interface for the command that failed
/// @param _cmdResult Result code from a failed command
/// @param _format Printf-style format string for error message
/// @param ... Variable arguments for format string
///
void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...)
{
    if (_format && strlen(_format) != 0)
    {
        va_list args;
        va_start(args, _format);
        MICROSTRAIN_LOG_ERROR_V(_format, args);
        va_end(args);
    }

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmdResult.value, _cmdResult.name());

    // Get the connection pointer that was set during device initialization
    microstrain::Connection* connection = static_cast<microstrain::Connection*>(_device.userPointer());

    terminate(connection, "");
}
