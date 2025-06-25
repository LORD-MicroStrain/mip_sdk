/////////////////////////////////////////////////////////////////////////////
/// cv7_example.cpp
///
/// Example set-up program for the CV7 using C++
///
/// This example shows a typical setup for the CV7 sensor using C++.
/// It is not an exhaustive example of all CV7 settings.
/// If this example does not meet your specific setup needs, please consult
/// the MIP SDK API documentation for the proper commands.
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

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>
#include <mip/definitions/data_filter.hpp>
#include <mip/definitions/data_sensor.hpp>
#include <mip/definitions/data_shared.hpp>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <memory>

// Message format configuration
void configureSensorMessageFormat(mip::Interface& _device);
void configureFilterMessageFormat(mip::Interface& _device);

// Event configuration
void configureEventTriggers(mip::Interface& _device);
void configureEventActions(mip::Interface& _device);
void enableEvents(mip::Interface& _device);
void handleEventTriggers(void* _user, const mip::FieldView& _field, mip::Timestamp _timestamp);

void displayFilterState(const mip::data_filter::FilterMode _filterState);

mip::Timestamp getCurrentTimestamp();

void initializeDevice(mip::Interface& _device);

void terminate(mip::Interface& _device, mip::CmdResult _cmdResult, const char* _format, ...);

int main(int argc, const char* argv[])
{
    // Unused variables
    (void)argc;
    (void)argv;

    // TODO: Update to the correct port name
    // Set the port name for the connection (Serial)
#ifdef _WIN32
    const char* portName = "COM8";
#else // Unix
    const char* portName = "/dev/ttyUSB0";
#endif // _WIN32

    // TODO: Update to the correct baudrate
    // Set the baudrate for the connection (Serial)
    const uint32_t baudrate = 115200;

    // Initialize the connection
    printf("Initializing serial port.\n");
    std::unique_ptr<microstrain::connections::UsbSerialConnection> connection =
        std::make_unique<microstrain::connections::UsbSerialConnection>(portName, baudrate);

    printf("Connecting to the device on port %s with %d baudrate.\n", portName, baudrate);

    // Open the connection to the device
    if (!connection->connect())
    {
        printf("ERROR: Could not open the connection!\n");
        return 1;
    }

    printf("Initializing the device interface.\n");
    mip::Interface device = mip::Interface(
        connection.get(),                            // Connection for the device
        mip::C::mip_timeout_from_baudrate(baudrate), // Set the base timeout for commands (milliseconds)
        500                                          // Set the base timeout for command replies (milliseconds)
    );
    initializeDevice(device);

    // Configure the message format for sensor data
    configureSensorMessageFormat(device);

    // Configure the message format for filter data
    configureFilterMessageFormat(device);

    // Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
    configureEventTriggers(device);
    configureEventActions(device);
    enableEvents(device);

    // Configure Sensor-to-Vehicle Transformation
    printf("Configuring %s.\n", mip::commands_3dm::Sensor2VehicleTransformEuler::DOC_NAME);
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

    // Configure Filter Aiding Measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    printf("Configuring %s.\n", mip::commands_filter::AidingMeasurementEnable::DOC_NAME);
    cmdResult = mip::commands_filter::writeAidingMeasurementEnable(
        device,                                                                    // Device
        mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER, // Aiding Source
        true                                                                       // Enabled
    );

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not set %s!\n", mip::commands_filter::AidingMeasurementEnable::DOC_NAME);
    }

    // Reset the filter
    // Note: This is good to do after filter setup is complete
    printf("Attempting to %s.\n", mip::commands_filter::Reset::DOC_NAME);
    cmdResult = mip::commands_filter::reset(device);
    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not %s!\n", mip::commands_filter::Reset::DOC_NAME);
    }

    // Register data callbacks

    // Sensor data callbacks
    printf("Registering sensor data callbacks.\n");

    mip::DispatchHandler sensorDataHandlers[4];

    // Data stores for sensor data
    mip::data_shared::GpsTimestamp sensorGpsTimestamp;
    mip::data_sensor::ScaledAccel  sensorScaledAccel;
    mip::data_sensor::ScaledGyro   sensorScaledGyro;
    mip::data_sensor::ScaledMag    sensorScaledMag;

    device.registerExtractor(sensorDataHandlers[0], &sensorGpsTimestamp);
    device.registerExtractor(sensorDataHandlers[1], &sensorScaledAccel);
    device.registerExtractor(sensorDataHandlers[2], &sensorScaledGyro);
    device.registerExtractor(sensorDataHandlers[3], &sensorScaledMag);

    // Filter data callbacks
    printf("Registering filter data callbacks.\n");

    mip::DispatchHandler filterDataHandlers[5];

    // Data stores for filter data
    mip::data_shared::GpsTimestamp filterGpsTimestamp;
    mip::data_filter::Status       filterStatus;
    mip::data_filter::EulerAngles  filterEulerAngles;
    mip::data_filter::PositionLlh  filterPositionLlh;

    device.registerExtractor(filterDataHandlers[0], &filterGpsTimestamp);
    device.registerExtractor(filterDataHandlers[1], &filterStatus);
    device.registerExtractor(filterDataHandlers[2], &filterEulerAngles);
    device.registerExtractor(filterDataHandlers[3], &filterPositionLlh);
    device.registerFieldCallback<&handleEventTriggers>(
        filterDataHandlers[4],
        mip::data_filter::DESCRIPTOR_SET,
        mip::data_shared::EventSource::FIELD_DESCRIPTOR
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    printf("Resuming the device.\n");
    cmdResult = mip::commands_base::resume(device);
    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not resume the device!\n");
    }

    printf("Sensor is configured... waiting for filter to initialize.\n");

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
        device.update();

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }

        // Print out data at 10 Hz
        const mip::Timestamp currentTime = getCurrentTimestamp();

        if (currentTime - previousPrintTimestamp >= 100)
        {
            if (filterStatus.filter_state >= mip::data_filter::FilterMode::AHRS)
            {
                printf("TOW = %f: ATT_EULER = [%f %f %f]\n",
                    filterGpsTimestamp.tow,
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch,
                    filterEulerAngles.yaw
                );
            }
            else if (filterStatus.filter_state >= mip::data_filter::FilterMode::VERT_GYRO)
            {
                printf("TOW = %f: ATT_EULER = [%f %f]\n",
                    filterGpsTimestamp.tow,
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch
                );
            }

            if (filterStatus.filter_state >= mip::data_filter::FilterMode::AHRS)
            {
                printf("TOW = %f: POS_LLH = [%f %f %f]\n",
                    filterGpsTimestamp.tow,
                    filterPositionLlh.latitude,
                    filterPositionLlh.longitude,
                    filterPositionLlh.ellipsoid_height
                );
            }

            previousPrintTimestamp = currentTime;
        }
    }

    printf("Example Completed Successfully.\n");
    printf("Closing the connection and exiting.\n");
    connection->disconnect();

    return 0;
}

// Configure Sensor data message format
void configureSensorMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet (shown in GNSS setup)

    printf("Getting the base rate for sensor data.\n");
    uint16_t sensorBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(_device, mip::data_sensor::DESCRIPTOR_SET, &sensorBaseRate);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get sensor base rate format!\n");
    }

    const uint16_t sensorSampleRate = 100; // Hz
    const uint16_t sensorDecimation = sensorBaseRate / sensorSampleRate;

    mip::DescriptorRate sensorDescriptors[4] = {
        { mip::data_shared::GpsTimestamp::FIELD_DESCRIPTOR, sensorDecimation },
        { mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR,  sensorDecimation },
        { mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR,   sensorDecimation },
        { mip::data_sensor::ScaledMag::FIELD_DESCRIPTOR,    sensorDecimation }
    };

    printf("Configuring %s for sensor data.\n", mip::commands_3dm::MessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,                                                  // Device
        mip::data_sensor::DESCRIPTOR_SET,                         // Data Descriptor
        sizeof(sensorDescriptors) / sizeof(sensorDescriptors[0]), // Size of the array
        sensorDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s for sensor data!\n", mip::commands_3dm::MessageFormat::DOC_NAME);
    }
}

// Configure Filter data message format
void configureFilterMessageFormat(mip::Interface& _device)
{
    printf("Getting the base rate for filter data.\n");
    uint16_t filterBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(_device, mip::data_filter::DESCRIPTOR_SET, &filterBaseRate);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get filter base rate format!\n");
    }

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation  = filterBaseRate / filter_sample_rate;

    mip::DescriptorRate filterDescriptors[3] = {
        { mip::data_shared::GpsTimestamp::FIELD_DESCRIPTOR, filter_decimation },
        { mip::data_filter::Status::FIELD_DESCRIPTOR,       filter_decimation },
        { mip::data_filter::EulerAngles::FIELD_DESCRIPTOR,  filter_decimation }
    };

    printf("Configuring %s for filter data.\n", mip::commands_3dm::MessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,                                                  // Device
        mip::data_filter::DESCRIPTOR_SET,                         // Data Descriptor
        sizeof(filterDescriptors) / sizeof(filterDescriptors[0]), // Size of the array
        filterDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set %s for filter data!\n", mip::commands_3dm::MessageFormat::DOC_NAME);
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

    printf("Configuring threshold event trigger for roll on trigger instance ID %d.\n", triggerInstanceId);
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventTrigger(
        _device,                                          // Device
        triggerInstanceId,                                // Trigger instance ID
        mip::commands_3dm::EventTrigger::Type::THRESHOLD, // Trigger type
        eventTriggerParameters                            // Trigger parameters
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set pitch event parameters!\n");
    }

    // Use the same trigger configuration, but set it to the y-axis (pitch)
    eventTriggerParameters.threshold.param_id = 2;

    // Note: This is independent of the param_id
    triggerInstanceId = 2;

    printf("Configuring threshold event trigger for pitch on trigger instance ID %d.\n", triggerInstanceId);
    cmdResult = mip::commands_3dm::writeEventTrigger(
        _device,                                          // Device
        triggerInstanceId,                                // Trigger instance ID
        mip::commands_3dm::EventTrigger::Type::THRESHOLD, // Trigger type
        eventTriggerParameters                            // Trigger parameters
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

    printf("Configuring message action instance ID %d for trigger instance ID %d (roll).\n",
        action_instance_id,
        trigger_instance_id
    );
    // Configure an action for event trigger 1 (roll)
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventAction(
        _device,                                       // Device
        action_instance_id,                            // Action instance ID
        trigger_instance_id,                           // Trigger instance ID to link to
        mip::commands_3dm::EventAction::Type::MESSAGE, // Action type
        eventActionParameters                          // Action parameters
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set roll action parameters!\n");
    }

    // Note: These are independent of each other and do not need to be the same
    // The tigger instance ID should match the configured trigger instance ID the action should be tied to
    action_instance_id  = 2;
    trigger_instance_id = 2;

    printf("Configuring message action instance ID %d for trigger instance ID %d (pitch).\n",
        action_instance_id,
        trigger_instance_id
    );
    // Configure an action for event trigger 2 (pitch)
    cmdResult = mip::commands_3dm::writeEventAction(
        _device,                                       // Device
        action_instance_id,                            // Action instance ID
        trigger_instance_id,                           // Trigger instance ID to link to
        mip::commands_3dm::EventAction::Type::MESSAGE, // Action type
        eventActionParameters                          // Action parameters
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
    printf("Enabling event trigger instance ID %d (roll).\n", eventTriggerInstanceId);
    mip::CmdResult cmdResult = mip::commands_3dm::writeEventControl(
        _device,                                       // Device
        eventTriggerInstanceId,                        // Event trigger instance ID to enable
        mip::commands_3dm::EventControl::Mode::ENABLED // Event control mode
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not enable roll event!\n");
    }

    eventTriggerInstanceId = 2;

    // Enable the pitch event trigger
    printf("Enabling event trigger instance ID %d (pitch).\n", eventTriggerInstanceId);
    cmdResult = mip::commands_3dm::writeEventControl(
        _device,                                       // Device
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
    // Unused variables
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
        printf("EVENT: Roll event triggered! Trigger ID: %d\n", eventSource.trigger_id);
    }
    // Event trigger instance ID 2 (pitch)
    else if (eventSource.trigger_id == 2)
    {
        printf("EVENT: Pitch event triggered! Trigger ID: %d\n", eventSource.trigger_id);
    }
}

void displayFilterState(const mip::data_filter::FilterMode _filterState)
{
    printf("The filter has entered ");

    switch (_filterState)
    {
        case mip::data_filter::FilterMode::INIT:
        {
            printf("initialization mode. INIT (%d)", static_cast<uint8_t>(_filterState));
            break;
        }
        case mip::data_filter::FilterMode::VERT_GYRO:
        {
            printf("vertical gyro mode. VERT_GYRO (%d)", static_cast<uint8_t>(_filterState));
            break;
        }
        case mip::data_filter::FilterMode::AHRS:
        {
            printf("AHRS mode. AHRS (%d)", static_cast<uint8_t>(_filterState));
            break;
        }
        case mip::data_filter::FilterMode::FULL_NAV:
        {
            printf("full navigation mode. FULL_NAV (%d)", static_cast<uint8_t>(_filterState));
            break;
        }
        default:
        {
            printf("startup mode. STARTUP (%d)", static_cast<uint8_t>(_filterState));
            break;
        }
    }

    printf("\n");
}

mip::Timestamp getCurrentTimestamp()
{
    std::chrono::nanoseconds timeSinceEpoch = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<mip::Timestamp>(std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceEpoch).count());
}

void initializeDevice(mip::Interface& _device)
{
    // Create a command result to check/print results when running commands
    mip::CmdResult cmdResult;

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    printf("Pinging the device.\n");
    cmdResult = mip::commands_base::ping(_device);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not ping the device!");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    printf("Setting device to idle.\n");
    cmdResult = mip::commands_base::setIdle(_device);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set the device to idle!\n");
    }

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    printf("Loading default settings.\n");
    cmdResult = mip::commands_3dm::defaultDeviceSettings(_device);
    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not load %s!\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    }
}

// void terminate(mip::Interface& _device, const char* _message, mip::CmdResult _cmdResult)
void terminate(mip::Interface& _device, mip::CmdResult _cmdResult, const char* _format, ...)
{
    va_list args;
    va_start(args, _format);
    vprintf(_format, args);
    va_end(args);

    printf("ERROR: %s Command Result: %d %s\n", _format, _cmdResult.value, _cmdResult.name());

    // Get the connection pointer that was set during device initialization
    microstrain::Connection* connection = static_cast<microstrain::Connection *>(_device.userPointer());

    if (connection == nullptr)
    {
        // Create the device interface with a connection or set it after creation
        printf("ERROR: Connection not set for the device interface. Cannot close the connection.\n");
    }
    else
    {
        printf("Closing the connection.\n");
        connection->disconnect();
    }

    printf("Exiting the program.\n");

    exit(1);
}
