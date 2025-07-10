////////////////////////////////////////////////////////////////////////////////
/// 7_series_threading_example.cpp
///
/// Example program to demonstrate multithreading for data collection and
/// command updates on 7-series devices using C++
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
#else // Unix
static constexpr const char* PORT_NAME = "/dev/ttyUSB0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
static constexpr uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
// Streaming rate in Hz
static constexpr uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
// Example run time
static constexpr uint32_t RUN_TIME_SECONDS = 30;

// TODO: Enable/disable data collection threading
// Use this to test the behaviors of threading
#define USE_THREADS 1
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
        mip::data_sensor::DESCRIPTOR_SET, // Data field descriptor set
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

    MICROSTRAIN_LOG_INFO("Sensor is configured... waiting for data.\n");

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
}

// Custom logging handler callback
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

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
mip::Timestamp getCurrentTimestamp()
{
    const std::chrono::nanoseconds timeSinceEpoch = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<mip::Timestamp>(std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceEpoch).count());
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize a MIP device and send some commands to prepare for configuration
///
/// @param _device Device to initialize
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
        terminate(_device, cmdResult, "Could not load %s!\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    }
}

// Configure Sensor data message format
void configureSensorMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t sensorBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(
        _device,
        mip::data_sensor::DESCRIPTOR_SET, // Data descriptor set
        &sensorBaseRate                   // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get sensor base rate!\n");
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
    const uint16_t sensorDecimation  = sensorBaseRate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO("Decimating sensor base rate %d by %d to stream data at %dHz.\n",
        sensorBaseRate,
        sensorDecimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip::DescriptorRate sensorDescriptors[1] = {
        { mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR, sensorDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for sensor data.\n", mip::commands_3dm::MessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,
        mip::data_sensor::DESCRIPTOR_SET,                         // Data descriptor set
        sizeof(sensorDescriptors) / sizeof(sensorDescriptors[0]), // Number of descriptors to include
        sensorDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s for sensor data!\n",
            mip::commands_3dm::MessageFormat::DOC_NAME
        );
    }
}

// Generic packet callback handler
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

    MICROSTRAIN_LOG_INFO("Received a packet at %" PRIu64 " with descriptor set 0x%02X:%s\n",
        _timestamp,
        _packetView.descriptorSet(),
        fieldDescriptorsBuffer
    );
}

#if USE_THREADS
// Device update callback
// Note: This handles separation of data collection updates and command updates
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

// Handle data collection from the device on a separate thread than commands
void dataCollectionThread(mip::Interface& _device, const volatile bool& _running)
{
    MICROSTRAIN_LOG_INFO("Data collection thread created!\n");

    while (_running)
    {
        // Update the device for data collection
        if (!_device.update(
            0,      // Wait time for the update (Typically only used from commands)
            false)) // From a command (Note: This update is for data handling, not from a command)
        {
            // Avoid deadlocks if the connection is closed
            _device.cmdQueue().clear();
            break;
        }

        std::this_thread::yield();
    }
}
#endif // USE_THREADS

// Print an error message and close the application
void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful /* = false */)
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

    MICROSTRAIN_LOG_INFO("Exiting the program.\n");

#ifdef _WIN32
    // Keep the console open on Windows
    system("pause");
#endif // _WIN32

    if (!_successful)
    {
        exit(1);
    }

    exit(0);
}

// Print an error message for a command and close the application
void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...)
{
    va_list args;
    va_start(args, _format);
    MICROSTRAIN_LOG_ERROR_V(_format, args);
    va_end(args);

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmdResult.value, _cmdResult.name());

    // Get the connection pointer that was set during device initialization
    microstrain::Connection* connection = static_cast<microstrain::Connection*>(_device.userPointer());

    terminate(connection, "");
}
