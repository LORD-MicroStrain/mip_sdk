////////////////////////////////////////////////////////////////////////////////
/// cv7_ahrs_example.cpp
///
/// Example setup program for the 3DM-CV7-AHRS using C++
///
/// This example shows a typical setup for the 3DM-CV7-AHRS sensor using C++.
/// It is not an exhaustive example of all 3DM-CV7-AHRS settings.
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
#include <mip/definitions/data_shared.hpp>

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

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
// Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static constexpr const char* PORT_NAME = "COM1";
#else // Unix
static const char* PORT_NAME = "/dev/ttyUSB0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
static constexpr uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
// Streaming rate in Hz
static constexpr uint16_t SAMPLE_RATE_HZ = 1;
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
void captureGyroBias(mip::Interface& _device);

// Filter message format configuration
void configureFilterMessageFormat(mip::Interface& _device);

// Event configuration
void configureEventTriggers(mip::Interface& _device);
void configureEventActions(mip::Interface& _device);
void enableEvents(mip::Interface& _device);
void handleEventTriggers(void* _user, const mip::FieldView& _field, mip::Timestamp _timestamp);

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

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&logCallback, MICROSTRAIN_LOG_LEVEL_INFO, nullptr);

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

    // Configure the message format for filter data
    configureFilterMessageFormat(device);

    // Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
    configureEventTriggers(device);
    configureEventActions(device);
    enableEvents(device);

    // Configure Sensor-to-Vehicle Transformation
    MICROSTRAIN_LOG_INFO("Configuring %s.\n", mip::commands_3dm::Sensor2VehicleTransformEuler::DOC_NAME);
    mip::CmdResult cmdResult = mip::commands_3dm::writeSensor2VehicleTransformEuler(
        device,
        0.0f, // Roll
        0.0f, // Pitch
        0.0f  // Yaw
    );

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not set %s!\n", mip::commands_3dm::Sensor2VehicleTransformEuler::DOC_NAME);
    }

    // Initialize the navigation filter
    initializeFilter(device);

    // Register filter data callbacks
    MICROSTRAIN_LOG_INFO("Registering filter data callbacks.\n");

    mip::DispatchHandler filterDataHandlers[4];

    // Data stores for filter data
    mip::data_shared::GpsTimestamp filterGpsTimestamp;
    mip::data_filter::Status       filterStatus;
    mip::data_filter::EulerAngles  filterEulerAngles;

    // Register the callbacks for the filter fields

    device.registerExtractor(
        filterDataHandlers[0],
        &filterGpsTimestamp // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[1],
        &filterStatus // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[2],
        &filterEulerAngles // Data field out
    );

    // Register a custom callback for the event field
    device.registerFieldCallback<&handleEventTriggers>(
        filterDataHandlers[3],
        mip::data_filter::DESCRIPTOR_SET,               // Data descriptor set
        mip::data_shared::EventSource::FIELD_DESCRIPTOR // Data field descriptor set
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
    while (filterStatus.filter_state != mip::data_filter::FilterMode::INIT)
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

        const mip::Timestamp currentTimestamp = getCurrentTimestamp();

        // Print out data based on the sample rate (1000 ms / SAMPLE_RATE_HZ)
        if (currentTimestamp - previousPrintTimestamp >= 1000 / SAMPLE_RATE_HZ)
        {
            if (filterStatus.filter_state >= mip::data_filter::FilterMode::AHRS)
            {
                MICROSTRAIN_LOG_INFO("TOW = %.3f: %s = [%9.6f %9.6f %9.6f]\n",
                    filterGpsTimestamp.tow,
                    mip::data_filter::EulerAngles::DOC_NAME, // Built-in metadata for easy printing
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch,
                    filterEulerAngles.yaw
                );
            }
            else if (filterStatus.filter_state >= mip::data_filter::FilterMode::VERT_GYRO)
            {
                MICROSTRAIN_LOG_INFO("TOW = %.3f: %s = [%9.6f %9.6f]\n",
                    filterGpsTimestamp.tow,
                    mip::data_filter::EulerAngles::DOC_NAME, // Built-in metadata for easy printing
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch
                );
            }

            previousPrintTimestamp = currentTimestamp;
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
    constexpr uint16_t captureDuration = 2000;

    constexpr uint16_t increasedCmdReplyTimeout = captureDuration * 2;

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
    const mip::CmdResult cmdResult = mip::commands_3dm::captureGyroBias(
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

// Configure Filter data message format
void configureFilterMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t filterBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(
        _device,
        mip::data_filter::DESCRIPTOR_SET, // Data descriptor set
        &filterBaseRate                   // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get filter base rate!\n");
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t filterDecimation  = filterBaseRate / SAMPLE_RATE_HZ;

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip::DescriptorRate filterDescriptors[3] = {
        { mip::data_filter::Timestamp::FIELD_DESCRIPTOR,   filterDecimation },
        { mip::data_filter::Status::FIELD_DESCRIPTOR,      filterDecimation },
        { mip::data_filter::EulerAngles::FIELD_DESCRIPTOR, filterDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for filter data at %dHz.\n", mip::commands_3dm::MessageFormat::DOC_NAME,
        SAMPLE_RATE_HZ
    );
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,
        mip::data_filter::DESCRIPTOR_SET,                         // Data descriptor set
        sizeof(filterDescriptors) / sizeof(filterDescriptors[0]), // Number of descriptors to include
        filterDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s for filter data!\n",
            mip::commands_3dm::MessageFormat::DOC_NAME
        );
    }
}

// Set up a trigger for filter euler angles
void configureEventTriggers(mip::Interface& _device)
{
    // Configure a threshold trigger
    mip::commands_3dm::EventTrigger::Parameters eventTriggerParameters;
    eventTriggerParameters.threshold.desc_set   = mip::data_filter::EulerAngles::DESCRIPTOR_SET;
    eventTriggerParameters.threshold.field_desc = mip::data_filter::EulerAngles::FIELD_DESCRIPTOR;

    // X-axis (roll)
    eventTriggerParameters.threshold.param_id = 1;

    // Configure the high and low thresholds for the trigger window
    eventTriggerParameters.threshold.type       = mip::commands_3dm::EventTrigger::ThresholdParams::Type::WINDOW;
    eventTriggerParameters.threshold.low_thres  = 45.0 * M_PI / 180.0;                         // Note: Command expects radians. Converting 45 degrees into radians
    eventTriggerParameters.threshold.high_thres = -eventTriggerParameters.threshold.low_thres; // -45 degrees

    // Note: This is independent of the param_id
    uint8_t triggerInstanceId = 1;

    MICROSTRAIN_LOG_INFO("Configuring threshold event trigger for roll on trigger instance ID %d.\n",
        triggerInstanceId
    );
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventTrigger(
        _device,
        triggerInstanceId,
        mip::commands_3dm::EventTrigger::Type::THRESHOLD, // Trigger type
        eventTriggerParameters                            // Trigger parameters to set
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set pitch event parameters!\n");
    }

    // Use the same trigger configuration, but set it to the y-axis (pitch)
    eventTriggerParameters.threshold.param_id = 2;

    // Note: This is independent of the param_id
    triggerInstanceId = 2;

    MICROSTRAIN_LOG_INFO("Configuring threshold event trigger for pitch on trigger instance ID %d.\n",
        triggerInstanceId
    );
    cmdResult = mip::commands_3dm::writeEventTrigger(
        _device,
        triggerInstanceId,
        mip::commands_3dm::EventTrigger::Type::THRESHOLD, // Trigger type
        eventTriggerParameters                            // Trigger parameters to set
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set roll event parameters!\n");
    }
}

// Note: Event trigger instance IDs do not need to match the Action instance IDs
void configureEventActions(mip::Interface& _device)
{
    mip::commands_3dm::EventAction::Parameters eventActionParameters;
    eventActionParameters.message.desc_set       = mip::data_filter::DESCRIPTOR_SET;
    eventActionParameters.message.decimation     = 0;
    eventActionParameters.message.num_fields     = 1;
    eventActionParameters.message.descriptors[0] = mip::data_shared::EventSource::FIELD_DESCRIPTOR;

    // Note: These are independent of each other and do not need to be the same
    // The tigger instance ID should match the configured trigger instance ID the action should be tied to
    uint8_t action_instance_id  = 1;
    uint8_t trigger_instance_id = 1;

    MICROSTRAIN_LOG_INFO("Configuring message action instance ID %d for trigger instance ID %d (roll).\n",
        action_instance_id,
        trigger_instance_id
    );
    // Configure an action for event trigger 1 (roll)
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventAction(
        _device,
        action_instance_id,
        trigger_instance_id,                           // Trigger instance ID to link to
        mip::commands_3dm::EventAction::Type::MESSAGE, // Action type
        eventActionParameters                          // Action parameters to set
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set roll action parameters!\n");
    }

    // Note: These are independent of each other and do not need to be the same
    // The tigger instance ID should match the configured trigger instance ID the action should be tied to
    action_instance_id  = 2;
    trigger_instance_id = 2;

    MICROSTRAIN_LOG_INFO("Configuring message action instance ID %d for trigger instance ID %d (pitch).\n",
        action_instance_id,
        trigger_instance_id
    );
    // Configure an action for event trigger 2 (pitch)
    cmdResult = mip::commands_3dm::writeEventAction(
        _device,
        action_instance_id,
        trigger_instance_id,                           // Trigger instance ID to link to
        mip::commands_3dm::EventAction::Type::MESSAGE, // Action type
        eventActionParameters                          // Action parameters to set
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set pitch action parameters!\n");
    }
}

// Enable the events
void enableEvents(mip::Interface& _device)
{
    uint8_t eventTriggerInstanceId = 1;

    // Enable the roll event trigger
    MICROSTRAIN_LOG_INFO("Enabling event trigger instance ID %d (roll).\n", eventTriggerInstanceId);
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventControl(
        _device,
        eventTriggerInstanceId,                        // Event trigger instance ID to enable
        mip::commands_3dm::EventControl::Mode::ENABLED // Event control mode
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not enable roll event!\n");
    }

    eventTriggerInstanceId = 2;

    // Enable the pitch event trigger
    MICROSTRAIN_LOG_INFO("Enabling event trigger instance ID %d (pitch).\n", eventTriggerInstanceId);
    cmdResult = mip::commands_3dm::writeEventControl(
        _device,
        eventTriggerInstanceId,                        // Event trigger instance ID to enable
        mip::commands_3dm::EventControl::Mode::ENABLED // Event control mode
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not enable pitch event!\n");
    }
}

// Handler for filter event source field
void handleEventTriggers(void* _user, const mip::FieldView& _field, mip::Timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip::data_shared::EventSource eventSource;

    if (!_field.extract(eventSource))
    {
        return;
    }

    // Event trigger instance ID 1 (roll)
    if (eventSource.trigger_id == 1)
    {
        MICROSTRAIN_LOG_WARN("Roll event triggered! Trigger ID: %d.\n", eventSource.trigger_id);
    }
    // Event trigger instance ID 2 (pitch)
    else if (eventSource.trigger_id == 2)
    {
        MICROSTRAIN_LOG_WARN("Pitch event triggered! Trigger ID: %d.\n", eventSource.trigger_id);
    }
}

// Initialize and reset the filter
void initializeFilter(mip::Interface& _device)
{
    // Configure Filter Aiding Measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    MICROSTRAIN_LOG_INFO("Configuring %s.\n", mip::commands_filter::AidingMeasurementEnable::DOC_NAME);
    mip::CmdResult cmdResult = mip::commands_filter::writeAidingMeasurementEnable(
        _device,
        mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER, // Aiding Source type
        true                                                                       // Enabled
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s!\n", mip::commands_filter::AidingMeasurementEnable::DOC_NAME);
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
        case mip::data_filter::FilterMode::INIT:
        {
            MICROSTRAIN_LOG_INFO("%s initialization mode. (%d) INIT\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        case mip::data_filter::FilterMode::VERT_GYRO:
        {
            MICROSTRAIN_LOG_INFO("%s vertical gyro mode. (%d) VERT_GYRO\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        case mip::data_filter::FilterMode::AHRS:
        {
            MICROSTRAIN_LOG_INFO("%s AHRS mode. (%d) AHRS\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        case mip::data_filter::FilterMode::FULL_NAV:
        {
            MICROSTRAIN_LOG_INFO("%s full navigation mode. (%d) FULL_NAV\n",
                headerMessage,
                filterStateValue
            );

            break;
        }
        default:
        {
            MICROSTRAIN_LOG_INFO("%s startup mode. (%d) STARTUP\n",
                headerMessage,
                filterStateValue
            );

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
