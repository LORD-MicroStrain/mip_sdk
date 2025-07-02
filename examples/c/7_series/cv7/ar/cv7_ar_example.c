////////////////////////////////////////////////////////////////////////////////
/// cv7_ar_example.c
///
/// Example setup program for the 3DM-CV7-AR using C
///
/// This example shows a typical setup for the 3DM-CV7-AR sensor using C.
/// It is not an exhaustive example of all 3DM-CV7-AR settings.
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
#include <microstrain/connections/serial/serial_port.h>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_filter.h>
#include <mip/definitions/data_filter.h>
#include <mip/definitions/data_sensor.h>
#include <mip/definitions/data_shared.h>
#include <mip/mip_interface.h>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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
void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Filter message format configuration
void configure_filter_message_format(mip_interface* _device);

// Event configuration
void configure_event_triggers(mip_interface* _device);
void configure_event_actions(mip_interface* _device);
void enable_events(mip_interface* _device);
void handle_event_triggers(void* _user, const mip_field_view* _field, mip_timestamp _timestamp);

// Filter initialization
void initialize_filter(mip_interface* _device);

// Utility to display filter state changes
void display_filter_state(const mip_filter_mode _filter_state);

// Utility to get the time delta since the application started (in milliseconds)
// Used for basic timestamping
mip_timestamp get_delta_time();

// Device callbacks used for reading and writing packets
bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length);
bool mip_interface_user_recv_from_device(mip_interface* _device, uint8_t* _buffer, size_t _max_length,
    mip_timeout _wait_time, bool _from_cmd, size_t* _length_out, mip_timestamp* _timestamp_out);

// Common device initialization procedure
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Utility functions the handle application closing and printing error messages
void terminate(serial_port* _device_port, const char* _message, const bool _successful);
void command_failure_terminate(mip_interface* _device, mip_cmd_result _cmd_result, const char* _format, ...);

// Global time variable for get_delta_time()
time_t g_start_time;

int main(int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the serial port.\n");
    serial_port device_port;
    serial_port_init(&device_port);

    // Record the application start time for use with get_delta_time()
    time(&g_start_time);

    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the connection to the device
    if (!serial_port_open(&device_port, PORT_NAME, BAUDRATE))
    {
        terminate(&device_port, "Could not open device port!\n", false);
    }

    mip_interface device;
    initialize_device(&device, &device_port, BAUDRATE);

    // Configure the message format for filter data
    configure_filter_message_format(&device);

    // Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
    configure_event_triggers(&device);
    configure_event_actions(&device);
    enable_events(&device);

    // Configure Sensor-to-Vehicle Transformation
    MICROSTRAIN_LOG_INFO("Configuring sensor-to-vehicle transformation.\n");
    mip_cmd_result cmd_result = mip_3dm_write_sensor_2_vehicle_transform_euler(
        &device,
        0.0f, // Roll
        0.0f, // Pitch
        0.0f  // Yaw
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not set sensor-to-vehicle transformation!\n");
    }

    // Initialize the navigation filter
    initialize_filter(&device);

    // Register filter data callbacks
    MICROSTRAIN_LOG_INFO("Registering filter data callbacks.\n");

    mip_dispatch_handler filter_data_handlers[4];

    // Data stores for filter data
    mip_shared_gps_timestamp_data filter_gps_timestamp;
    mip_filter_status_data        filter_status;
    mip_filter_euler_angles_data  filter_euler_angles;

    // Register the callbacks for the filter fields

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[0],                         // Data handler
        MIP_FILTER_DATA_DESC_SET,                         // Data descriptor set
        MIP_DATA_DESC_SHARED_GPS_TIME,                    // Data field descriptor
        extract_mip_shared_gps_timestamp_data_from_field, // Callback
        &filter_gps_timestamp                             // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[1],                  // Data handler
        MIP_FILTER_DATA_DESC_SET,                  // Data descriptor set
        MIP_DATA_DESC_FILTER_FILTER_STATUS,        // Data field descriptor
        extract_mip_filter_status_data_from_field, // Callback
        &filter_status                             // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[2],                        // Data handler
        MIP_FILTER_DATA_DESC_SET,                        // Data descriptor set
        MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES,           // Data field descriptor
        extract_mip_filter_euler_angles_data_from_field, // Callback
        &filter_euler_angles                             // Data field out
    );

    // Register a custom callback for the event field
    mip_interface_register_field_callback(
        &device,
        &filter_data_handlers[3],          // Data handler
        MIP_FILTER_DATA_DESC_SET,          // Data descriptor set
        MIP_DATA_DESC_SHARED_EVENT_SOURCE, // Data field descriptor
        handle_event_triggers,             // Callback
        NULL                               // Data field out
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmd_result = mip_base_resume(&device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... waiting for filter to initialize.\n");

    mip_filter_mode current_state = filter_status.filter_state;

    // Wait for the device to initialize
    while (filter_status.filter_state != MIP_FILTER_MODE_INIT)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        mip_interface_update(
            &device,
            0,    // Time to wait
            false // From command
        );

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }
    }

    mip_timestamp previous_print_time = 0;

    // Device loop
    // TODO: Update loop condition to allow for exiting
    while (true)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        mip_interface_update(
            &device,
            0,    // Time to wait
            false // From command
        );

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }

        const mip_timestamp delta_time = get_delta_time();

        // Print out data at 10 Hz (1000ms / 100ms)
        if (delta_time - previous_print_time >= 100)
        {
            if (filter_status.filter_state >= MIP_FILTER_MODE_VERT_GYRO)
            {
                MICROSTRAIN_LOG_INFO("TOW = %.3f: Euler Angles = [%f %f]\n",
                    filter_gps_timestamp.tow,
                    filter_euler_angles.roll,
                    filter_euler_angles.pitch
                );
            }

            previous_print_time = delta_time;
        }
    }

    terminate(&device_port, "Example Completed Successfully.\n", true);

    return 0;
}

// Custom logging handler callback
void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
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

// Configure Filter data message format
void configure_filter_message_format(mip_interface* _device)
{
    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t filter_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_FILTER_DATA_DESC_SET, // Data descriptor set
        &filter_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get filter base rate!\n");
    }

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation  = filter_base_rate / filter_sample_rate;

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate filter_descriptors[3] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for filter data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_FILTER_DATA_DESC_SET,                                   // Data Descriptor
        sizeof(filter_descriptors) / sizeof(filter_descriptors[0]), // Size of the array
        filter_descriptors                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set message format for filter data!\n");
    }
}

// Set up a trigger for filter euler angles
void configure_event_triggers(mip_interface* _device)
{
    // Configure a threshold trigger
    mip_3dm_event_trigger_command_parameters event_parameters;
    event_parameters.threshold.desc_set   = MIP_FILTER_DATA_DESC_SET;
    event_parameters.threshold.field_desc = MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES;

    // X-axis (roll)
    event_parameters.threshold.param_id = 1;

    // Configure the high and low thresholds for the trigger window
    event_parameters.threshold.type       = MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW;
    event_parameters.threshold.low_thres  = 45.0 * M_PI / 180.0;                   // Note: Command expects radians. Converting 45 degrees into radians
    event_parameters.threshold.high_thres = -event_parameters.threshold.low_thres; // -45 degrees

    // Note: This is independent of the param_id
    uint8_t trigger_instance_id = 1;

    MICROSTRAIN_LOG_INFO("Configuring threshold event trigger for roll on trigger instance ID %d.\n", trigger_instance_id);
    mip_cmd_result cmd_result = mip_3dm_write_event_trigger(
        _device,
        trigger_instance_id,
        MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD, // Trigger type
        &event_parameters                             // Trigger parameters to set
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set pitch event parameters!\n");
    }

    // Use the same trigger configuration, but set it to the y-axis (pitch)
    event_parameters.threshold.param_id = 2;

    // Note: This is independent of the param_id
    trigger_instance_id = 2;

    MICROSTRAIN_LOG_INFO("Configuring threshold event trigger for pitch on trigger instance ID %d.\n", trigger_instance_id);
    cmd_result = mip_3dm_write_event_trigger(
        _device,
        trigger_instance_id,
        MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD, // Trigger type
        &event_parameters                             // Trigger parameters to set
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set roll event parameters!\n");
    }
}

// Note: Event trigger instance IDs do not need to match the Action instance IDs
void configure_event_actions(mip_interface* _device)
{
    mip_3dm_event_action_command_parameters event_action_parameters;
    event_action_parameters.message.desc_set       = MIP_FILTER_DATA_DESC_SET;
    event_action_parameters.message.decimation     = 0;
    event_action_parameters.message.num_fields     = 1;
    event_action_parameters.message.descriptors[0] = MIP_DATA_DESC_SHARED_EVENT_SOURCE;

    // Note: These are independent of each other and do not need to be the same
    // The tigger instance ID should match the configured trigger instance ID the action should be tied to
    uint8_t action_instance_id  = 1;
    uint8_t trigger_instance_id = 1;

    MICROSTRAIN_LOG_INFO("Configuring message action instance ID %d for trigger instance ID %d (roll).\n",
        action_instance_id,
        trigger_instance_id
    );
    // Configure an action for event trigger 1 (roll)
    mip_cmd_result cmd_result = mip_3dm_write_event_action(
        _device,
        action_instance_id,
        trigger_instance_id,                       // Trigger instance ID to link to
        MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, // Action type
        &event_action_parameters                   // Action parameters to set
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set roll action parameters!\n");
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
    cmd_result = mip_3dm_write_event_action(
        _device,
        action_instance_id,
        trigger_instance_id,                       // Trigger instance ID to link to
        MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, // Action type
        &event_action_parameters                   // Action parameters to set
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set pitch action parameters!\n");
    }
}

// Enable the events
void enable_events(mip_interface* _device)
{
    uint8_t event_trigger_instance_id = 1;

    // Enable the roll event trigger
    MICROSTRAIN_LOG_INFO("Enabling event trigger instance ID %d (roll).\n", event_trigger_instance_id);
    mip_cmd_result cmd_result = mip_3dm_write_event_control(
        _device,
        event_trigger_instance_id,                 // Event trigger instance ID to enable
        MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED // Event control mode
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not enable roll event!\n");
    }

    event_trigger_instance_id = 2;

    // Enable the pitch event trigger
    MICROSTRAIN_LOG_INFO("Enabling event trigger instance ID %d (pitch).\n", event_trigger_instance_id);
    cmd_result = mip_3dm_write_event_control(
        _device,
        event_trigger_instance_id,                 // Event trigger instance ID to enable
        MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED // Event control mode
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not enable pitch event!\n");
    }
}

// Handler for filter event source field
void handle_event_triggers(void* _user, const mip_field_view* _field, mip_timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip_shared_event_source_data event_source;

    if (!extract_mip_shared_event_source_data_from_field(_field, &event_source))
    {
        return;
    }

    // Event trigger instance ID 1 (roll)
    if (event_source.trigger_id == 1)
    {
        MICROSTRAIN_LOG_WARN("Roll event triggered! Trigger ID: %d.\n", event_source.trigger_id);
    }
    // Event trigger instance ID 2 (pitch)
    else if (event_source.trigger_id == 2)
    {
        MICROSTRAIN_LOG_WARN("Pitch event triggered! Trigger ID: %d.\n", event_source.trigger_id);
    }
}

// Initialize and reset the filter
void initialize_filter(mip_interface* _device)
{
    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to reset the navigation filter.\n");
    mip_cmd_result cmd_result = mip_filter_reset(_device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not reset the navigation filter!\n");
    }
}

// Display the filter change status
void display_filter_state(const mip_filter_mode _filter_state)
{
    const char*   header_message     = "The filter has entered";
    const uint8_t filter_state_value = (uint8_t)_filter_state;

    switch (_filter_state)
    {
        case MIP_FILTER_MODE_INIT:
        {
            MICROSTRAIN_LOG_INFO("%s initialization mode. (%d) MIP_FILTER_MODE_INIT\n",
                header_message,
                filter_state_value
            );

            break;
        }
        case MIP_FILTER_MODE_VERT_GYRO:
        {
            MICROSTRAIN_LOG_INFO("%s vertical gyro mode. (%d) MIP_FILTER_MODE_VERT_GYRO\n",
                header_message,
                filter_state_value
            );

            break;
        }
        case MIP_FILTER_MODE_AHRS:
        {
            MICROSTRAIN_LOG_INFO("%s AHRS mode. (%d) MIP_FILTER_MODE_AHRS\n",
                header_message,
                filter_state_value
            );

            break;
        }
        case MIP_FILTER_MODE_FULL_NAV:
        {
            MICROSTRAIN_LOG_INFO("%s full navigation mode. (%d) MIP_FILTER_MODE_FULL_NAV\n",
                header_message,
                filter_state_value
            );

            break;
        }
        default:
        {
            MICROSTRAIN_LOG_INFO("%s startup mode. (%d) STARTUP\n",
                header_message,
                filter_state_value
            );

            break;
        }
    }
}

// Get the time delta since the application started (in milliseconds)
mip_timestamp get_delta_time()
{
    time_t t;
    time(&t);

    // Get the time difference since application start
    double delta = difftime(t, g_start_time);

    // Convert to milliseconds
    return (mip_timestamp)(delta * 1000);
}

// Send packet handler callback
bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length)
{
    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the bytes written to the device
    size_t bytes_written;

    // Send the packet to the device
    return serial_port_write(device_port, _data, _length, &bytes_written);
}

// Receive packet handler callback
bool mip_interface_user_recv_from_device(mip_interface* _device, uint8_t* _buffer, size_t _max_length,
    mip_timeout _wait_time, bool _from_cmd, size_t* _length_out, mip_timestamp* _timestamp_out)
{
    // Unused parameter
    (void)_from_cmd;

    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the time that packet was received
    *_timestamp_out = get_delta_time();

    // Read the packet from the device
    return serial_port_read(device_port, _buffer, _max_length, (int)_wait_time, _length_out);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize a MIP device and send some commands to prepare for configuration
///
/// @param _device Device to initialize
/// @param _device_port Serial port to use for the device connection
/// @param _baudrate Baudrate to open the connection with
///
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate)
{
    MICROSTRAIN_LOG_INFO("Initializing device interface.\n");
    mip_interface_init(
        _device,
        mip_timeout_from_baudrate(_baudrate), // Set the base timeout for commands (milliseconds)
        2000,                                 // Set the base timeout for command replies (milliseconds)
        &mip_interface_user_send_to_device,   // User-defined send packet callback
        &mip_interface_user_recv_from_device, // User-defined receive packet callback
        &mip_interface_default_update,        // Default update callback
        (void*)_device_port                   // Cast the device port for use in the callbacks
    );

    // Create a command result to check/print results when running commands
    mip_cmd_result cmd_result;

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    cmd_result = mip_base_ping(_device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting device to idle.\n");
    cmd_result = mip_base_set_idle(_device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting device information.\n");
    mip_base_device_info device_info;
    cmd_result = mip_base_get_device_info(_device, &device_info);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = device_info.firmware_version / 1000;
    const uint16_t minor = device_info.firmware_version / 100 % 10;
    const uint16_t patch = device_info.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16];
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d",
        major,
        minor,
        patch
    );

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name",             device_info.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number",     device_info.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number",    device_info.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number",       device_info.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options",          device_info.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n",  "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    MICROSTRAIN_LOG_INFO("Loading default settings.\n");
    cmd_result = mip_3dm_default_device_settings(_device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not load device default settings!\n");
    }
}

// Print an error message and close the application
void terminate(serial_port* _device_port, const char* _message, const bool _successful)
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

    if (_device_port == NULL)
    {
        // Initialize the device interface with a serial port connection
        MICROSTRAIN_LOG_ERROR("Serial port not set for the device interface. Cannot close the serial port.\n");
    }
    else
    {
        if (serial_port_is_open(_device_port))
        {
            MICROSTRAIN_LOG_INFO("Closing the serial port.\n");

            if (!serial_port_close(_device_port))
            {
                MICROSTRAIN_LOG_ERROR("Failed to close the serial port!\n");
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
void command_failure_terminate(mip_interface* _device, mip_cmd_result _cmd_result, const char* _format, ...)
{
    va_list args;
    va_start(args, _format);
    MICROSTRAIN_LOG_ERROR_V(_format, args);
    va_end(args);

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmd_result, mip_cmd_result_to_string(_cmd_result));

    if (_device == NULL)
    {
        terminate(NULL, "", false);
    }
    else
    {
        // Get the serial connection pointer that was set during device initialization
        serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

        terminate(device_port, "", false);
    }
}
