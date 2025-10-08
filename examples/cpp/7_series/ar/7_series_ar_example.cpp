////////////////////////////////////////////////////////////////////////////////
/// @file 7_series_ar_example.cpp
///
/// @defgroup _7_series_ar_example_cpp 7-Series AR Example [CPP]
///
/// @ingroup examples_cpp
///
/// @brief Example setup program for the 3DM-CV7-AR, and 3DM-GV7-AR using C++
///
/// @details This example shows a basic setup to configure the attitude filter
///          to stream filter data for the 3DM-CV7-AR, and 3DM-GV7-AR using C++.
///          This is not an exhaustive example of all settings for those
///          devices. If this example does not meet your specific setup needs,
///          please consult the MIP SDK API documentation for the proper
///          commands.
///
/// @section _7_series_ar_example_cpp_license License
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
#include <mip/definitions/commands_filter.hpp>
#include <mip/definitions/data_filter.hpp>
#include <mip/definitions/data_shared.hpp>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <chrono>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
/// @brief Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static constexpr const char* PORT_NAME = "COM1";
#else  // Unix
static constexpr const char* PORT_NAME = "/dev/ttyACM0";
#endif // _WIN32

/// @brief Set the baudrate for the connection (Serial/USB)
/// @note For native serial connections this needs to be 115200 due to the device default settings command
/// Use mip::commands_base::*CommSpeed() to write and save the baudrate on the device
static constexpr uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
/// @brief Streaming rate in Hz
static constexpr uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
/// @brief Example run time
static constexpr uint32_t RUN_TIME_SECONDS = 30;
////////////////////////////////////////////////////////////////////////////////

///
/// @} group _7_series_ar_example_cpp
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
static void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
static void captureGyroBias(mip::Interface& _device);

// Filter message format configuration
static void configureFilterMessageFormat(mip::Interface& _device);

// Event configuration
static void configureEventTriggers(mip::Interface& _device);
static void configureEventActions(mip::Interface& _device);
static void enableEvents(mip::Interface& _device);
static void handleEventTriggers(void* _user, const mip::FieldView& _field, mip::Timestamp _timestamp);

// Filter initialization
static void initializeFilter(mip::Interface& _device);

// Utility to display filter state changes
static void displayFilterState(const mip::data_filter::FilterMode _filterState);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
static mip::Timestamp getCurrentTimestamp();

// Common device initialization procedure
static void initializeDevice(mip::Interface& _device);

// Utility functions the handle application closing and printing error messages
static void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful = false);
static void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...);

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
        terminate(device, cmdResult, "Could not configure %s!\n", mip::commands_3dm::Sensor2VehicleTransformEuler::DOC_NAME);
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
        &filterGpsTimestamp,             // Data field out
        mip::data_filter::DESCRIPTOR_SET // Data descriptor set
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

    MICROSTRAIN_LOG_INFO("The device is configured... waiting for the filter to initialize.\n");

    mip::data_filter::FilterMode currentState = filterStatus.filter_state;

    // Wait for the device to initialize
    while (filterStatus.filter_state < mip::data_filter::FilterMode::VERT_GYRO)
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

    MICROSTRAIN_LOG_INFO("This example will now output data for %ds.\n", RUN_TIME_SECONDS);

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
            if (filterStatus.filter_state >= mip::data_filter::FilterMode::VERT_GYRO)
            {
                MICROSTRAIN_LOG_INFO(
                    "%s = %10.3f%16s = [%9.6f, %9.6f, %9.6f]\n",
                    "TOW",
                    filterGpsTimestamp.tow,
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
/// @addtogroup _7_series_ar_example_cpp
/// @{
///

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
static void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
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
static void captureGyroBias(mip::Interface& _device)
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
    MICROSTRAIN_LOG_WARN("About to capture gyro bias for %.2g seconds!\n", static_cast<float>(captureDuration) / 1000.0f);
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

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f, %f, %f]\n", gyroBias[0], gyroBias[1], gyroBias[2]);

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
///             - GPS timestamp
///             - Filter status
///             - Euler angles
///
/// @param _device Reference to the initialized MIP device interface
///
static void configureFilterMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t       filterBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(
        _device,
        mip::data_filter::DESCRIPTOR_SET, // Data descriptor set
        &filterBaseRate                   // Base rate out
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
        {mip::data_shared::GpsTimestamp::FIELD_DESCRIPTOR, filterDecimation},
        {mip::data_filter::Status::FIELD_DESCRIPTOR,       filterDecimation},
        {mip::data_filter::EulerAngles::FIELD_DESCRIPTOR,  filterDecimation}
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for filter data.\n", mip::commands_3dm::MessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,
        mip::data_filter::DESCRIPTOR_SET,                         // Data descriptor set
        sizeof(filterDescriptors) / sizeof(filterDescriptors[0]), // Number of descriptors to include
        filterDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not configure %s for filter data!\n",
            mip::commands_3dm::MessageFormat::DOC_NAME
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures threshold event triggers for roll and pitch angles
///
/// @details Sets up two event triggers for monitoring Euler angles:
///          1. Roll angle threshold (Trigger ID 1)
///             - Monitors X-axis rotation
///             - Triggers when the angle exceeds +/-45 degrees
///          2. Pitch angle threshold (Trigger ID 2)
///             - Monitors Y-axis rotation
///             - Triggers when the angle exceeds +/-45 degrees
///
/// @param _device Reference to the initialized MIP device interface
///
static void configureEventTriggers(mip::Interface& _device)
{
    // Configure a threshold trigger
    mip::commands_3dm::EventTrigger::Parameters eventTriggerParameters;
    eventTriggerParameters.threshold.desc_set   = mip::data_filter::EulerAngles::DESCRIPTOR_SET;
    eventTriggerParameters.threshold.field_desc = mip::data_filter::EulerAngles::FIELD_DESCRIPTOR;

    // X-axis (roll)
    eventTriggerParameters.threshold.param_id = 1;

    // Configure the threshold as a trigger window
    eventTriggerParameters.threshold.type = mip::commands_3dm::EventTrigger::ThresholdParams::Type::WINDOW;

    // Configure the high and low thresholds for the trigger window
    // Note: The command expects radians for these values
    eventTriggerParameters.threshold.low_thres  = 45.0 * M_PI / 180.0;                         // 45 degrees
    eventTriggerParameters.threshold.high_thres = -eventTriggerParameters.threshold.low_thres; // -45 degrees

    // Note: This is independent of the param_id
    uint8_t triggerInstanceId = 1;

    MICROSTRAIN_LOG_INFO("Configuring a threshold event trigger for roll on trigger instance ID %d.\n", triggerInstanceId);
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventTrigger(
        _device,
        triggerInstanceId,
        mip::commands_3dm::EventTrigger::Type::THRESHOLD, // Trigger type
        eventTriggerParameters                            // Trigger parameters to set
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not configure a threshold event trigger for roll!\n");
    }

    // Use the same trigger configuration, but set it to the y-axis (pitch)
    eventTriggerParameters.threshold.param_id = 2;

    // Note: This is independent of the param_id
    triggerInstanceId = 2;

    MICROSTRAIN_LOG_INFO("Configuring a threshold event trigger for pitch on trigger instance ID %d.\n", triggerInstanceId);
    cmdResult = mip::commands_3dm::writeEventTrigger(
        _device,
        triggerInstanceId,
        mip::commands_3dm::EventTrigger::Type::THRESHOLD, // Trigger type
        eventTriggerParameters                            // Trigger parameters to set
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not configure a threshold event trigger for pitch!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures event actions to occur when triggers are activated
///
/// @details Sets up message actions for each event trigger:
///          - Links action instance 1 to trigger instance 1 (roll)
///          - Links action instance 2 to trigger instance 2 (pitch)
///          - Configures both to output event source data when triggered
///
/// @param _device Reference to the initialized MIP device interface
///
static void configureEventActions(mip::Interface& _device)
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

    MICROSTRAIN_LOG_INFO(
        "Configuring message action instance ID %d for trigger instance ID %d (roll).\n",
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
        terminate(_device, cmdResult, "Could not configure a message action for the roll event trigger!\n");
    }

    // Note: These are independent of each other and do not need to be the same
    // The tigger instance ID should match the configured trigger instance ID the action should be tied to
    action_instance_id  = 2;
    trigger_instance_id = 2;

    MICROSTRAIN_LOG_INFO(
        "Configuring message action instance ID %d for trigger instance ID %d (pitch).\n",
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
        terminate(_device, cmdResult, "Could not configure a message action for the pitch event trigger!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Enables the configured event triggers
///
/// @details Activates both event triggers:
///          1. Enables roll threshold monitoring (Trigger ID 1)
///          2. Enables pitch threshold monitoring (Trigger ID 2)
///
/// @param _device Reference to the initialized MIP device interface
///
static void enableEvents(mip::Interface& _device)
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
        terminate(_device, cmdResult, "Could not enable event trigger for roll!\n");
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
        terminate(_device, cmdResult, "Could not enable event trigger for pitch!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Event handler for filter data source triggers
///
/// @details Processes event trigger notifications for:
///          - Roll threshold events (Trigger ID 1)
///          - Pitch threshold events (Trigger ID 2)
///          Outputs appropriate warning messages when thresholds are exceeded.
///
/// @param _user User data pointer (unused)
/// @param _field Reference to the MIP field containing event data
/// @param _timestamp Timestamp of when the event occurred (unused)
///
static void handleEventTriggers(void* _user, const mip::FieldView& _field, mip::Timestamp _timestamp)
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

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and resets the navigation filter
///
/// @details Configures the filter by resetting it to apply new settings
///
/// @param _device Reference to the initialized MIP device interface
///
static void initializeFilter(mip::Interface& _device)
{
    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to %s.\n", mip::commands_filter::Reset::DOC_NAME);
    const mip::CmdResult cmdResult = mip::commands_filter::reset(_device);

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
///          - Vertical gyro mode
///          - AHRS mode
///          - Full navigation mode
///
/// @param _filterState Current filter mode from the MIP device interface
///
static void displayFilterState(const mip::data_filter::FilterMode _filterState)
{
    const char* modeDescription = "startup";
    const char* modeType        = "STARTUP";

    switch (_filterState)
    {
        case mip::data_filter::FilterMode::INIT:
        {
            modeDescription = "initialization";
            modeType        = "INIT";
            break;
        }
        case mip::data_filter::FilterMode::VERT_GYRO:
        {
            modeDescription = "vertical gyro";
            modeType        = "VERT_GYRO";
            break;
        }
        case mip::data_filter::FilterMode::AHRS:
        {
            modeDescription = "AHRS";
            modeType        = "AHRS";
            break;
        }
        case mip::data_filter::FilterMode::FULL_NAV:
        {
            modeDescription = "full navigation";
            modeType        = "FULL_NAV";
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
static mip::Timestamp getCurrentTimestamp()
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
static void initializeDevice(mip::Interface& _device)
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
    char firmwareVersion[16] = {0};
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
            MICROSTRAIN_LOG_WARN("On a native serial connections the baudrate needs to be 115200 for this example to run.\n");
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
static void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful /* = false */)
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
static void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...)
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

///
/// @} group _7_series_ar_example_cpp
////////////////////////////////////////////////////////////////////////////////
