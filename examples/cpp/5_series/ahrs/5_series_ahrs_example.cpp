/////////////////////////////////////////////////////////////////////////////
///
/// 5_series_ahrs_example.cpp
///
/// Example setup program for the 3DM-CX5-AHRS, 3DM-CV5-AHRS, and
/// 3DM-GX5-AHRS using C++
///
/// This example shows a typical setup for the 3DM-CX5-AHRS, 3DM-CV5-AHRS,
/// and 3DM-GX5-AHRS in a wheeled-vehicle application using C++.
/// It is not an exhaustive example of all settings for those devices.
/// If this example does not meet your specific setup needs, please consult
/// the MIP SDK API documentation for the proper commands.
///
/// @section LICENSE
///
/// THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
/// CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR
/// THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD
/// LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO
/// ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE
/// BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION
/// WITH THEIR PRODUCTS.
///

// Include the MicroStrain Serial connection header
#include <microstrain/connections/serial/serial_connection.hpp>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.hpp>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>
#include <mip/definitions/data_filter.hpp>
#include <mip/definitions/data_sensor.hpp>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

// TODO: Update to the correct port name and baudrate
////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static const char* PORT_NAME = "COM1";
#else // Unix
static const char* PORT_NAME = "/dev/ttyUSB0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
static const uint32_t BAUDRATE = 115200;
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
void captureGyroBias(mip::Interface& _device);

// Message format configuration
void configureSensorMessageFormat(mip::Interface& _device);
void configureFilterMessageFormat(mip::Interface& _device);

// Filter initialization
void initializeFilter(mip::Interface& _device);

// Utility to display filter state changes
void displayFilterState(const mip::data_filter::FilterMode _filterState);

// Get the current system time (in milliseconds)
mip::Timestamp getCurrentTimestamp();

// Common device initialization procedure
void initializeDevice(mip::Interface& _device);

// Utility functions the handle application closing and printing error messages
void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful = false);
void terminate(mip::Interface& _device, mip::CmdResult _cmdResult, const char* _format, ...);

int main(int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&logCallback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    microstrain::connections::UsbSerialConnection connection(PORT_NAME, BAUDRATE);

    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the connection to the device
    if (!connection.connect())
    {
        terminate(&connection, "Could not open the connection!\n");
    }

    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip::Interface device = mip::Interface(
        &connection,                                 // Connection for the device
        mip::C::mip_timeout_from_baudrate(BAUDRATE), // Set the base timeout for commands (milliseconds)
        2000                                         // Set the base timeout for command replies (milliseconds)
    );
    initializeDevice(device);

    // Capture gyro bias
    captureGyroBias(device);

    // Configure the message format for sensor data
    configureSensorMessageFormat(device);

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
        terminate(device, cmdResult, "Could not set %s!\n",
            mip::commands_filter::SensorToVehicleRotationEuler::DOC_NAME
        );
    }

    // Initialize the navigation filter
    initializeFilter(device);

    // Register data callbacks

    // Sensor data callbacks
    MICROSTRAIN_LOG_INFO("Registering sensor data callbacks.\n");

    mip::DispatchHandler sensorDataHandlers[3];

    // Data stores for sensor data
    mip::data_sensor::GpsTimestamp sensorGpsTimestamp;
    mip::data_sensor::ScaledAccel  sensorScaledAccel;
    mip::data_sensor::ScaledGyro   sensorScaledGyro;

    // Register the callbacks for the sensor fields
    device.registerExtractor(
        sensorDataHandlers[0], // Data handler
        &sensorGpsTimestamp    // Data field out
    );

    device.registerExtractor(
        sensorDataHandlers[1], // Data handler
        &sensorScaledAccel     // Data field out
    );

    device.registerExtractor(
        sensorDataHandlers[2], // Data handler
        &sensorScaledGyro      // Data field out
    );

    // Filter data callbacks
    MICROSTRAIN_LOG_INFO("Registering filter data callbacks.\n");

    mip::DispatchHandler filterDataHandlers[3];

    // Data stores for filter data
    mip::data_filter::Timestamp   filterTimestamp;
    mip::data_filter::Status      filterStatus;
    mip::data_filter::EulerAngles filterEulerAngles;

    // Register the callbacks for the filter fields
    device.registerExtractor(
        filterDataHandlers[0], // Data handler
        &filterTimestamp       // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[1], // Data handler
        &filterStatus          // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[2], // Data handler
        &filterEulerAngles     // Data field out
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmdResult = mip::commands_base::resume(device);
    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... waiting for filter to initialize.\n");

    mip::data_filter::FilterMode currentState = filterStatus.filter_state;

    // Wait for the device to initialize
    while (filterStatus.filter_state != mip::data_filter::FilterMode::GX5_INIT)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        device.update();

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }
    }

    mip::Timestamp previousPrintTimestamp = 0;

    // Device loop
    // TODO: Update loop condition to allow for exiting
    while (true)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        device.update();

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }

        const mip::Timestamp currentTime = getCurrentTimestamp();

        // Print out data at 10 Hz (1000ms / 100ms)
        if (currentTime - previousPrintTimestamp >= 100)
        {
            if (filterStatus.filter_state == mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID)
            {
                MICROSTRAIN_LOG_INFO("TOW = %.3f: %s = [%f %f %f]\n",
                    filterTimestamp.tow,
                    mip::data_filter::EulerAngles::DOC_NAME, // Built-in metadata for easy printing
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch,
                    filterEulerAngles.yaw
                );
            }

            previousPrintTimestamp = currentTime;
        }
    }

    terminate(&connection, "Example Completed Successfully.\n", true);

    return 0;
}

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
{
    // Unused parameter
    (void)_user;

    switch (_level)
    {
        case MICROSTRAIN_LOG_LEVEL_FATAL:
        {
            fprintf(stderr, "%-9s", "FATAL: ");
            vfprintf(stderr, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_ERROR:
        {
            fprintf(stderr, "%-9s", "ERROR: ");
            vfprintf(stderr, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_WARN:
        {
            fprintf(stdout, "%-9s", "WARNING: ");
            vfprintf(stdout, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_INFO:
        {
            fprintf(stdout, "%-9s", "INFO: ");
            vfprintf(stdout, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_DEBUG:
        {
            fprintf(stdout, "%-9s", "DEBUG: ");
            vfprintf(stdout, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_TRACE:
        {
            fprintf(stdout, "%-9s", "TRACE: ");
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

// Capture gyro bias
void captureGyroBias(mip::Interface& _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip::CmdQueue&     cmdQueue        = _device.cmdQueue();
    const mip::Timeout previousTimeout = cmdQueue.baseReplyTimeout();
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previousTimeout);

    // Note: The default is 15 s (15,000 ms)
    // Longer sample times are recommended but shortened here for convenience
    const uint16_t captureDuration = 2000;

    const uint16_t increasedCmdReplyTimeout = captureDuration * 2;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n", increasedCmdReplyTimeout);
    cmdQueue.setBaseReplyTimeout(increasedCmdReplyTimeout);

    mip::Vector3f gyroBias = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    MICROSTRAIN_LOG_INFO("Capturing gyro bias. This will take %.2g seconds.\n",
        static_cast<float>(captureDuration) / 1000.0f
    );
    mip::CmdResult cmdResult = mip::commands_3dm::captureGyroBias(
        _device,
        captureDuration, // Capture duration (ms)
        gyroBias         // Gyro bias out (result of the capture)
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f %f %f]\n",
        gyroBias[0],
        gyroBias[1],
        gyroBias[2]
    );

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previousTimeout);
    cmdQueue.setBaseReplyTimeout(previousTimeout);
}

// Configure Sensor data message format
void configureSensorMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t sensorBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::imuGetBaseRate(
        _device,
        &sensorBaseRate // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get sensor base rate!\n");
    }

    const uint16_t sensorSampleRate = 100; // Hz
    const uint16_t sensorDecimation = sensorBaseRate / sensorSampleRate;

    // Descriptor rate is a pair of data descriptor set and decimation
    mip::DescriptorRate sensorDescriptors[4] = {
        { mip::data_sensor::GpsTimestamp::FIELD_DESCRIPTOR, sensorDecimation },
        { mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR,  sensorDecimation },
        { mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR,   sensorDecimation },
        { mip::data_sensor::ScaledMag::FIELD_DESCRIPTOR,    sensorDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for sensor data.\n", mip::commands_3dm::ImuMessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeImuMessageFormat(
        _device,
        sizeof(sensorDescriptors) / sizeof(sensorDescriptors[0]), // Size of the array
        sensorDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s for sensor data!\n",
            mip::commands_3dm::ImuMessageFormat::DOC_NAME);
    }
}

// Configure Filter data message format
void configureFilterMessageFormat(mip::Interface& _device)
{
    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t filterBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::filterGetBaseRate(
        _device,
        &filterBaseRate // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get filter base rate!\n");
    }

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation  = filterBaseRate / filter_sample_rate;

    // Descriptor rate is a pair of data descriptor set and decimation
    mip::DescriptorRate filterDescriptors[3] = {
        { mip::data_filter::Timestamp::FIELD_DESCRIPTOR,   filter_decimation },
        { mip::data_filter::Status::FIELD_DESCRIPTOR,      filter_decimation },
        { mip::data_filter::EulerAngles::FIELD_DESCRIPTOR, filter_decimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for filter data.\n", mip::commands_3dm::FilterMessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeFilterMessageFormat(
        _device,
        sizeof(filterDescriptors) / sizeof(filterDescriptors[0]), // Size of the array
        filterDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s for filter data!\n",
            mip::commands_3dm::FilterMessageFormat::DOC_NAME
        );
    }
}

// Initialize and reset the filter
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
        terminate(_device, cmdResult, "Could not configure filter %s!\n",
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
        terminate(_device, cmdResult, "Could not enable filter %s!\n",
            mip::commands_filter::AutoInitControl::DOC_NAME
        );
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

// Display the filter change status
void displayFilterState(const mip::data_filter::FilterMode _filterState)
{
    const char*   headerMessage    = "The filter has entered";
    const uint8_t filterStateValue = static_cast<uint8_t>(_filterState);

    switch (_filterState)
    {
        case mip::data_filter::FilterMode::GX5_INIT:
        {
            MICROSTRAIN_LOG_INFO("%s initialization mode. (%d) GX5_INIT\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        case mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID:
        {
            MICROSTRAIN_LOG_INFO("%s run solution valid mode. (%d) GX5_RUN_SOLUTION_VALID\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        case mip::data_filter::FilterMode::GX5_RUN_SOLUTION_ERROR:
        {
            MICROSTRAIN_LOG_INFO("%s run solution error mode. (%d) GX5_RUN_SOLUTION_ERROR\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        default:
        {
            MICROSTRAIN_LOG_INFO("%s startup mode. (%d) GX5_STARTUP\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
    }
}

// Get the current system time (in milliseconds)
mip::Timestamp getCurrentTimestamp()
{
    std::chrono::nanoseconds timeSinceEpoch = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<mip::Timestamp>(std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceEpoch).count());
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize a MIP device and send some commands to prepare for configuration
///
/// @param _device Device to initialize
///
void initializeDevice(mip::Interface& _device)
{
    // Create a command result to check/print results when running commands
    mip::CmdResult cmdResult;

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    cmdResult = mip::commands_base::ping(_device);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting device to idle.\n");
    cmdResult = mip::commands_base::setIdle(_device);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting device information.\n");
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

    if (_connection == nullptr)
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
}

// Print an error message for a command and close the application
void terminate(mip::Interface& _device, mip::CmdResult _cmdResult, const char* _format, ...)
{
    va_list args;
    va_start(args, _format);
    MICROSTRAIN_LOG_ERROR_V(_format, args);
    va_end(args);

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s\n",  _cmdResult.value, _cmdResult.name());

    // Get the connection pointer that was set during device initialization
    microstrain::Connection* connection = static_cast<microstrain::Connection*>(_device.userPointer());

    terminate(connection, "");
}
