////////////////////////////////////////////////////////////////////////////////
/// @addtogroup examples_cpp
/// @{
///
/// @defgroup 5_series_stream_imu_example_cpp 5-Series Stream IMU Example [CPP]
///
/// @brief Example setup program for streaming IMU data on 5-series devices
///        using C++
///
/// @details This example shows a basic setup for streaming IMU data on 5-series
///          devices using C++. This is not an exhaustive example of all
///          streaming options for those devices. If this example does not meet
///          your specific setup needs, please consult the MIP SDK API
///          documentation for the proper commands.
///
/// @section LICENSE
///
/// @copyright Copyright (c) 2025 MicroStrain by HBK
///            Licensed under MIT License
///
/// @{
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
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
mip::Timestamp getCurrentTimestamp();

// Common device initialization procedure
void initializeDevice(mip::Interface& _device);

// Utility to help check if the device supports a descriptor
bool isDescriptorSupported(
    const mip::CompositeDescriptor& _compositeDescriptor, const uint16_t* _supportedDescriptors,
    const uint8_t _supportedDescriptorCount
);

// Message format configuration
void configureSensorMessageFormat(
    mip::Interface& _device, const uint16_t* _supportedDescriptors, const uint8_t _supportedDescriptorCount
);

// Callback handlers
void packetCallback(void* _user, const mip::PacketView& _packetView, mip::Timestamp _timestamp);
void accelFieldCallback(void* _user, const mip::FieldView& _fieldView, mip::Timestamp _timestamp);
void gyroFieldCallback(void* _user, const mip::FieldView& _fieldView, mip::Timestamp _timestamp);
void magFieldCallback(void* _user, const mip::FieldView& _fieldView, mip::Timestamp _timestamp);

// Utility functions the handle application closing and printing error messages
void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful = false);
void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...);

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

// Note: This is a compile-time way of checking that the proper logging level is enabled
// Note: The max available logging level may differ in pre-packaged installations of the MIP SDK
#ifndef MICROSTRAIN_LOGGING_ENABLED_INFO
#error This example requires a logging level of at least MICROSTRAIN_LOGGING_LEVEL_INFO_ to work properly
#endif // !MICROSTRAIN_LOGGING_ENABLED_INFO

    // Initialize the custom logger to print messages/errors as they occur
    // Note: The logging level parameter doesn't need to match the max logging level.
    // If the parameter is higher than the max level, higher-level logging functions will be ignored
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

    // Get supported descriptors to check if certain data descriptors are supported for streaming
    MICROSTRAIN_LOG_INFO("Getting supported descriptors for the device.\n");
    uint8_t  descriptorsCount          = 0;
    uint16_t supportedDescriptors[256] = { 0 };

    mip::CmdResult cmdResult = mip::commands_base::getDeviceDescriptors(
        device,
        supportedDescriptors,                                           // Descriptors array from the device
        sizeof(supportedDescriptors) / sizeof(supportedDescriptors[0]), // Max array size
        &descriptorsCount                                               // Descriptor count returned from the device
    );

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not get supported descriptors!\n");
    }

    // Some devices have a large number of descriptors
    // The extended descriptors command can get the remaining descriptors
    MICROSTRAIN_LOG_INFO("Getting extended supported descriptors for the device.\n");
    uint8_t extendedDescriptorsCount = 0;

    cmdResult = mip::commands_base::getExtendedDescriptors(
        device,
        &supportedDescriptors[descriptorsCount],                                           // Append to the array
        sizeof(supportedDescriptors) / sizeof(supportedDescriptors[0]) - descriptorsCount, // Remaining array size
        &extendedDescriptorsCount
    );

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not get extended supported descriptors!\n");
    }

    // Configure the message format for sensor data
    configureSensorMessageFormat(device, supportedDescriptors, descriptorsCount + extendedDescriptorsCount);

    // Register packet and data callbacks

    // Generic packet callback
    MICROSTRAIN_LOG_INFO("Registering a generic packet callback.\n");

    mip::DispatchHandler packetHandler;

    // Register the callback for packets
    device.registerPacketCallback<&packetCallback>(
        packetHandler,
        mip::Dispatcher::ANY_DATA_SET, // Data descriptor set
        false,                         // Process after field callback
        nullptr                        // User data
    );

    // Sensor data callbacks
    MICROSTRAIN_LOG_INFO("Registering sensor data callbacks.\n");

    mip::DispatchHandler sensorDataHandlers[3];

    // Register the callbacks for the sensor fields

    device.registerFieldCallback<&accelFieldCallback>(
        sensorDataHandlers[0],
        mip::data_sensor::ScaledAccel::DESCRIPTOR_SET,   // Data descriptor set
        mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR, // Data field descriptor set
        nullptr                                          // User data
    );

    device.registerFieldCallback<&gyroFieldCallback>(
        sensorDataHandlers[1],
        mip::data_sensor::ScaledGyro::DESCRIPTOR_SET,   // Data descriptor set
        mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR, // Data field descriptor set
        nullptr                                         // User data
    );

    // Note: Even if the device doesn't support mag data, registering this won't break anything
    // The callback will just never be called
    device.registerFieldCallback<&magFieldCallback>(
        sensorDataHandlers[2],
        mip::data_sensor::ScaledMag::DESCRIPTOR_SET,   // Data descriptor set
        mip::data_sensor::ScaledMag::FIELD_DESCRIPTOR, // Data field descriptor set
        nullptr                                        // User data
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmdResult = mip::commands_base::resume(device);

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
        // Update the device state
        // Note: This will update the device callbacks
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        device.update(
            10 // Time to wait
        );
    }

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
/// @brief Determines if the device supports a specific descriptor
///
/// @details Checks if a given composite descriptor is supported by comparing
///          against an array of supported descriptors. The device returns
///          descriptors as composite values (e.g., 0x8001 from 0x80,0x01).
///
/// @param _compositeDescriptor Reference to the composite descriptor to check.
/// @param _supportedDescriptors Array of supported descriptors from the device
/// @param _supportedDescriptorCount Number of entries in the array
///
/// @returns true if the descriptor is supported, false otherwise
///
bool isDescriptorSupported(
    const mip::CompositeDescriptor& _compositeDescriptor, const uint16_t* _supportedDescriptors,
    const uint8_t _supportedDescriptorCount
)
{
    for (uint8_t index = 0; index < _supportedDescriptorCount; ++index)
    {
        if (_supportedDescriptors[index] == _compositeDescriptor.as_u16())
        {
            return true;
        }
    }

    return false;
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
///             - Scaled gyroscope
///             - Scaled magnetometer
///
/// @param _device Reference to the initialized MIP device interface
/// @param _supportedDescriptors Array of descriptors supported by the device
/// @param _supportedDescriptorCount Number of descriptors in the array
///
void configureSensorMessageFormat(
    mip::Interface& _device, const uint16_t* _supportedDescriptors, const uint8_t _supportedDescriptorCount
)
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
    const mip::DescriptorRate sensorDescriptors[3] = {
        { mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR, sensorDecimation },
        { mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR,  sensorDecimation },
        { mip::data_sensor::ScaledMag::FIELD_DESCRIPTOR,   sensorDecimation }
    };

    // Assume all descriptors are supported until the check below
    uint8_t sensorDescriptorCount = sizeof(sensorDescriptors) / sizeof(sensorDescriptors[0]);

    MICROSTRAIN_LOG_INFO("Checking if the device supports magnetometer data.\n");

    // Not all devices have a magnetometer
    if (isDescriptorSupported(mip::data_sensor::ScaledMag::DESCRIPTOR, _supportedDescriptors, _supportedDescriptorCount))
    {
        MICROSTRAIN_LOG_INFO("The device supports magnetometer data.\n");
    }
    else
    {
        MICROSTRAIN_LOG_INFO("The device does not support magnetometer data.\n");

        // Don't include the magnetometer data descriptor in the array for message format configuration
        --sensorDescriptorCount;
    }

    MICROSTRAIN_LOG_INFO("Configuring %s for sensor data.\n", mip::commands_3dm::ImuMessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeImuMessageFormat(
        _device,
        sensorDescriptorCount, // Number of descriptors to include
        sensorDescriptors      // Descriptor array
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

////////////////////////////////////////////////////////////////////////////////
/// @brief Callback handler for accelerometer data fields
///
/// @details Processes scaled accelerometer data fields by:
///          1. Extracting the scaled acceleration vector from the field
///          2. Logging the X, Y, Z acceleration values with descriptors
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _fieldView Reference to the field containing accelerometer data
/// @param _timestamp Timestamp indicating when the field was received from the
///                   device (unused in this implementation)
///
void accelFieldCallback(void* _user, const mip::FieldView& _fieldView, mip::Timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip::data_sensor::ScaledAccel scaledAccelData;

    if (_fieldView.extract(scaledAccelData))
    {
        MICROSTRAIN_LOG_INFO(
            "%-17s (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Accel Data",
            mip::data_sensor::ScaledAccel::DESCRIPTOR_SET,   // Data descriptor set
            mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR, // Data field descriptor set
            scaledAccelData.scaled_accel[0],                 // X
            scaledAccelData.scaled_accel[1],                 // Y
            scaledAccelData.scaled_accel[2]                  // Z
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Callback handler for gyroscope data fields
///
/// @details Processes scaled gyroscope data fields by:
///          1. Extracting the scaled angular rates from the field
///          2. Logging the X, Y, Z angular rate values with descriptors
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _fieldView Reference to the field containing gyroscope data
/// @param _timestamp Timestamp indicating when the field was received from the
///                   device (unused in this implementation)
///
void gyroFieldCallback(void* _user, const mip::FieldView& _fieldView, mip::Timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip::data_sensor::ScaledGyro scaledGyroData;

    if (_fieldView.extract(scaledGyroData))
    {
        MICROSTRAIN_LOG_INFO(
            "%-17s (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Gyro Data",
            mip::data_sensor::ScaledGyro::DESCRIPTOR_SET,   // Data descriptor set
            mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR, // Data field descriptor set
            scaledGyroData.scaled_gyro[0],                  // X
            scaledGyroData.scaled_gyro[1],                  // Y
            scaledGyroData.scaled_gyro[2]                   // Z
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Callback handler for magnetometer data fields
///
/// @details Processes scaled magnetometer data fields by:
///          1. Extracting the scaled magnetic field vector from the field
///          2. Logging the X, Y, Z magnetic field values with descriptors
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _fieldView Reference to the field containing magnetometer data
/// @param _timestamp Timestamp indicating when the field was received from the
///                   device (unused in this implementation)
///
void magFieldCallback(void* _user, const mip::FieldView& _fieldView, mip::Timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip::data_sensor::ScaledMag scaledMagData;

    if (_fieldView.extract(scaledMagData))
    {
        MICROSTRAIN_LOG_INFO(
            "%-17s (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Mag Data",
            mip::data_sensor::ScaledMag::DESCRIPTOR_SET,   // Data descriptor set
            mip::data_sensor::ScaledMag::FIELD_DESCRIPTOR, // Data field descriptor set
            scaledMagData.scaled_mag[0],                   // X
            scaledMagData.scaled_mag[1],                   // Y
            scaledMagData.scaled_mag[2]                    // Z
        );
    }
}

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

/// @} group 5_series_stream_imu_example_cpp
/// @} group examples_cpp
////////////////////////////////////////////////////////////////////////////////
