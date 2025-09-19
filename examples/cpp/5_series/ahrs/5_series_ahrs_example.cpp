////////////////////////////////////////////////////////////////////////////////
/// 5_series_ahrs_example.cpp
///
/// Example setup program for the 3DM-CX5-AHRS, 3DM-CV5-AHRS, and 3DM-GX5-AHRS
/// using C++
///
/// This example shows a basic setup for the 3DM-CX5-AHRS, 3DM-CV5-AHRS, and
/// 3DM-GX5-AHRS using C++.
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

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.hpp>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.hpp>
#include <mip/mip_interface.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>
#include <mip/definitions/data_filter.hpp>

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

// Capture gyro bias
void captureGyroBias(mip::Interface& _device);

// Filter message format configuration
void configureFilterMessageFormat(mip::Interface& _device);

// Filter initialization
void initializeFilter(mip::Interface& _device);

// Utility to display filter state changes
void displayFilterState(const mip::data_filter::FilterMode _filterState);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
mip::Timestamp getCurrentTimestamp();

// Common device initialization procedure
void initializeDevice(mip::Interface& _device);

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

    // Capture gyro bias
    captureGyroBias(device);

    // Configure the message format for filter data
    configureFilterMessageFormat(device);

    // Configure Sensor-to-Vehicle Rotation
    MICROSTRAIN_LOG_INFO("Configuring %s.\n", mip::commands_filter::SensorToVehicleRotationEuler::DOC_NAME);
    mip::CmdResult cmdResult = mip::commands_filter::writeSensorToVehicleRotationEuler(
        device,
        0.0f, // Roll
        0.0f, // Pitch
        0.0f  // Yaw
    );

    if (!cmdResult.isAck())
    {
        terminate(
            device,
            cmdResult,
            "Could not configure %s!\n",
            mip::commands_filter::SensorToVehicleRotationEuler::DOC_NAME
        );
    }

    // Initialize the navigation filter
    initializeFilter(device);

    // Register filter data callbacks
    MICROSTRAIN_LOG_INFO("Registering filter data callbacks.\n");

    mip::DispatchHandler filterDataHandlers[3];

    // Data stores for filter data
    mip::data_filter::Timestamp   filterTimestamp;
    mip::data_filter::Status      filterStatus;
    mip::data_filter::EulerAngles filterEulerAngles;

    // Register the callbacks for the filter fields

    device.registerExtractor(
        filterDataHandlers[0],
        &filterTimestamp // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[1],
        &filterStatus // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[2],
        &filterEulerAngles // Data field out
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmdResult = mip::commands_base::resume(device);

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("The device is configured... waiting for the filter to initialize.\n");

    mip::data_filter::FilterMode currentState = filterStatus.filter_state;

    // Wait for the device to initialize
    while (filterStatus.filter_state < mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        device.update(
            10 // Time to wait
        );

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }
    }

    // Get the start time of the device update loop to handle exiting the application
    const mip::Timestamp loopStartTime = getCurrentTimestamp();

    mip::Timestamp previousPrintTimestamp = 0;

    // Running loop
    // Exit after a predetermined time in seconds
    while (getCurrentTimestamp() - loopStartTime <= RUN_TIME_SECONDS * 1000)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        device.update(
            10 // Time to wait
        );

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }

        const mip::Timestamp currentTimestamp = getCurrentTimestamp();

        // Print out data based on the sample rate (1000 ms / SAMPLE_RATE_HZ)
        if (currentTimestamp - previousPrintTimestamp >= 1000 / SAMPLE_RATE_HZ)
        {
            if (filterStatus.filter_state >= mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID)
            {
                MICROSTRAIN_LOG_INFO(
                    "%s = %10.3f%16s = [%9.6f, %9.6f, %9.6f]\n",
                    "TOW",
                    filterTimestamp.tow,
                    mip::data_filter::EulerAngles::DOC_NAME, // Built-in metadata for easy printing
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch,
                    filterEulerAngles.yaw
                );
            }

            previousPrintTimestamp = currentTimestamp;
        }
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
/// @brief Captures and configures device gyro bias
///
/// @param _device Reference to the initialized MIP device interface
///
void captureGyroBias(mip::Interface& _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip::CmdQueue&     cmdQueue        = _device.cmdQueue();
    const mip::Timeout previousTimeout = cmdQueue.baseReplyTimeout();
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previousTimeout);

    // Note: The default is 15 s (15,000 ms)
    constexpr uint16_t captureDuration          = 15000;
    constexpr uint16_t increasedCmdReplyTimeout = captureDuration + 1000;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n", increasedCmdReplyTimeout);
    cmdQueue.setBaseReplyTimeout(increasedCmdReplyTimeout);

    mip::Vector3f gyroBias = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: When capturing gyro bias, the device needs to remain still on a flat surface
    MICROSTRAIN_LOG_WARN(
        "About to capture gyro bias for %.2g seconds!\n",
        static_cast<float>(captureDuration) / 1000.0f
    );
    MICROSTRAIN_LOG_WARN("Please do not move the device during this time!\n");
    MICROSTRAIN_LOG_WARN("Press 'Enter' when ready...");

    // Wait for anything to be entered
    const int confirmCapture = getc(stdin);
    (void)confirmCapture; // Unused

    MICROSTRAIN_LOG_WARN("Capturing gyro bias...\n");
    const mip::CmdResult cmdResult = mip::commands_3dm::captureGyroBias(
        _device,
        captureDuration, // Capture duration (ms)
        gyroBias         // Gyro bias out (result of the capture)
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO(
        "Capture gyro bias completed with result: [%f, %f, %f]\n",
        gyroBias[0],
        gyroBias[1],
        gyroBias[2]
    );

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previousTimeout);
    cmdQueue.setBaseReplyTimeout(previousTimeout);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for filter data streaming
///
/// @details Sets up filter data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - Filter timestamp
///             - Filter status
///             - Euler angles
///
/// @param _device Reference to the initialized MIP device interface
///
void configureFilterMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t       filterBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::filterGetBaseRate(
        _device,
        &filterBaseRate // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get the base rate for filter data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > filterBaseRate)
    {
        terminate(
            _device,
            mip::CmdResult::NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            filterBaseRate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t filterDecimation = filterBaseRate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating filter base rate %d by %d to stream data at %dHz.\n",
        filterBaseRate,
        filterDecimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip::DescriptorRate filterDescriptors[3] = {
        { mip::data_filter::Timestamp::FIELD_DESCRIPTOR,   filterDecimation },
        { mip::data_filter::Status::FIELD_DESCRIPTOR,      filterDecimation },
        { mip::data_filter::EulerAngles::FIELD_DESCRIPTOR, filterDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for filter data.\n", mip::commands_3dm::FilterMessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeFilterMessageFormat(
        _device,
        sizeof(filterDescriptors) / sizeof(filterDescriptors[0]), // Number of descriptors to include
        filterDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not configure %s for filter data!\n",
            mip::commands_3dm::FilterMessageFormat::DOC_NAME
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and resets the navigation filter
///
/// @details Configures the navigation filter by:
///          1. Setting the magnetometer as the heading source
///          2. Enabling filter auto-initialization
///          3. Resetting the filter to apply new settings
///
/// @param _device Reference to the initialized MIP device interface
///
void initializeFilter(mip::Interface& _device)
{
    // Configure filter heading source
    MICROSTRAIN_LOG_INFO("Configuring filter %s to magnetometer.\n", mip::commands_filter::HeadingSource::DOC_NAME);
    mip::CmdResult cmdResult = mip::commands_filter::writeHeadingSource(
        _device,
        mip::commands_filter::HeadingSource::Source::MAG // Aiding Source type
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not configure filter %s to magnetometer!\n",
            mip::commands_filter::HeadingSource::DOC_NAME
        );
    }

    // Enable filter auto initialization control
    MICROSTRAIN_LOG_INFO("Enabling filter %s.\n", mip::commands_filter::AutoInitControl::DOC_NAME);
    cmdResult = mip::commands_filter::writeAutoInitControl(
        _device,
        1 // Enabled
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not enable filter %s!\n", mip::commands_filter::AutoInitControl::DOC_NAME);
    }

    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to %s.\n", mip::commands_filter::Reset::DOC_NAME);
    cmdResult = mip::commands_filter::reset(_device);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not %s!\n", mip::commands_filter::Reset::DOC_NAME);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Displays the current filter state when changes occur
///
/// @details Outputs readable messages for filter state transitions:
///          - Initialization mode
///          - Run solution valid mode
///          - Run solution error mode
///
/// @param _filterState Current filter mode from the MIP device interface
///
void displayFilterState(const mip::data_filter::FilterMode _filterState)
{
    const char* modeDescription = "startup";
    const char* modeType        = "GX5_STARTUP";

    switch (_filterState)
    {
        case mip::data_filter::FilterMode::GX5_INIT:
        {
            modeDescription = "initialization";
            modeType        = "GX5_INIT";
            break;
        }
        case mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID:
        {
            modeDescription = "run solution valid";
            modeType        = "GX5_RUN_SOLUTION_VALID";
            break;
        }
        case mip::data_filter::FilterMode::GX5_RUN_SOLUTION_ERROR:
        {
            modeDescription = "run solution error";
            modeType        = "GX5_RUN_SOLUTION_ERROR";
            break;
        }
        default:
        {
            break;
        }
    }

    MICROSTRAIN_LOG_INFO(
        "The filter has entered %s mode. (%d) %s\n",
        modeDescription,
        static_cast<uint8_t>(_filterState),
        modeType
    );
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
