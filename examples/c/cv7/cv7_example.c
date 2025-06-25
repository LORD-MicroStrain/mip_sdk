////////////////////////////////////////////////////////////////////////////////
/// cv7_example.c
///
/// Example set-up program for the CV7 using C
///
/// This example shows a typical setup for the CV7 sensor using C.
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
#include <microstrain/connections/serial/serial_port.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_filter.h>
#include <mip/definitions/data_filter.h>
#include <mip/definitions/data_sensor.h>
#include <mip/definitions/data_shared.h>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Message format configuration
void configure_sensor_message_format(mip_interface* device);
void configure_filter_message_format(mip_interface* device);

// Event configuration
void configure_event_triggers(mip_interface* device);
void configure_event_actions(mip_interface* device);
void enable_events(mip_interface* device);
void handle_event_triggers(void* user, const mip_field_view* field, mip_timestamp timestamp);

void display_filter_state(const mip_filter_mode filter_state);

mip_timestamp get_current_timestamp();

// Device callbacks used for reading and writing packets
bool mip_interface_user_send_to_device(mip_interface* mip_device, const uint8_t* data, size_t length);
bool mip_interface_user_recv_from_device(mip_interface* mip_device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, bool from_cmd,
    size_t* length_out, mip_timestamp* timestamp_out);

void initialize_device(mip_interface* device, serial_port* device_port, uint32_t baudrate);

void terminate(mip_interface* device, const char* message, mip_cmd_result cmd_result);

// Global time variable to get_timestamp()
time_t g_start_time;

int main(int argc, const char* argv[])
{
    // Unused variables
    (void)argc;
    (void)argv;

    // TODO: Update to the correct port name
    // Set the port name for the connection (Serial)
#ifdef _WIN32
    const char* port_name = "COM8";
#else // Unix
    const char* port_name = "/dev/ttyUSB0";
#endif // _WIN32

    // TODO: Update to the correct baudrate
    // Set the baudrate for the connection (Serial)
    const uint32_t baudrate = 115200;

    // Initialize the connection
    printf("Initializing serial port.\n");
    serial_port device_port;
    serial_port_init(&device_port);

    // Record program start time for use with get_timestamp()
    time(&g_start_time);

    printf("Connecting to the device on port %s with %d baudrate.\n", port_name, baudrate);

    // Open the connection to the device
    if (!serial_port_open(&device_port, port_name, baudrate))
    {
        printf("ERROR: Could not open device port!\n");
        return 1;
    }

    // Initialize the MIP device and send commands to prepare for configuration/use
    mip_interface device;
    initialize_device(&device, &device_port, baudrate);

    // Configure the message format for sensor data
    configure_sensor_message_format(&device);

    // Configure the message format for filter data
    configure_filter_message_format(&device);

    // Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
    configure_event_triggers(&device);
    configure_event_actions(&device);
    enable_events(&device);

    // Configure Sensor-to-Vehicle Transformation
    printf("Configuring sensor-to-vehicle transformation.\n");
    mip_cmd_result cmd_result = mip_3dm_write_sensor_2_vehicle_transform_euler(
        &device,
        0.0f, // Roll
        0.0f, // Pitch
        0.0f  // Yaw
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(&device, "Could not set sensor-to-vehicle transformation!\n", cmd_result);
    }

    // Configure Filter Aiding Measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    printf("Configuring filter aiding measurement enable.\n");
    cmd_result = mip_filter_write_aiding_measurement_enable(
        &device,                                                                 // Device
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, // Aiding Source
        true                                                                     // Enabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(&device, "Could not set filter aiding measurement enable!\n", cmd_result);
    }

    // Reset the filter
    // Note: This is good to do after filter setup is complete
    printf("Resetting the filter.\n");
    cmd_result = mip_filter_reset(&device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(&device, "Could not reset the filter!\n", cmd_result);
    }

    // Register data callbacks

    // Sensor data callbacks
    printf("Registering sensor data callbacks.\n");

    mip_dispatch_handler sensor_data_handlers[4];

    // Data stores for sensor data
    mip_shared_gps_timestamp_data sensor_gps_time;
    mip_sensor_scaled_accel_data  sensor_scaled_accel;
    mip_sensor_scaled_gyro_data   sensor_scaled_gyro;
    mip_sensor_scaled_mag_data    sensor_scaled_mag;

    mip_interface_register_extractor(
        &device,
        &sensor_data_handlers[0],
        MIP_SENSOR_DATA_DESC_SET,
        MIP_DATA_DESC_SHARED_GPS_TIME,
        extract_mip_shared_gps_timestamp_data_from_field,
        &sensor_gps_time
    );

    mip_interface_register_extractor(
        &device,
        &sensor_data_handlers[1],
        MIP_SENSOR_DATA_DESC_SET,
        MIP_DATA_DESC_SENSOR_ACCEL_SCALED,
        extract_mip_sensor_scaled_accel_data_from_field,
        &sensor_scaled_accel
    );

    mip_interface_register_extractor(
        &device,
        &sensor_data_handlers[2],
        MIP_SENSOR_DATA_DESC_SET,
        MIP_DATA_DESC_SENSOR_GYRO_SCALED,
        extract_mip_sensor_scaled_gyro_data_from_field,
        &sensor_scaled_gyro
    );

    mip_interface_register_extractor(
        &device,
        &sensor_data_handlers[3],
        MIP_SENSOR_DATA_DESC_SET,
        MIP_DATA_DESC_SENSOR_MAG_SCALED,
        extract_mip_sensor_scaled_mag_data_from_field,
        &sensor_scaled_mag
    );

    // Filter data callbacks
    printf("Registering filter data callbacks.\n");

    mip_dispatch_handler filter_data_handlers[5];

    // Data stores for filter data
    mip_shared_gps_timestamp_data filter_gps_timestamp;
    mip_filter_status_data        filter_status;
    mip_filter_euler_angles_data  filter_euler_angles;
    mip_filter_position_llh_data  filter_position_llh;

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[0],
        MIP_FILTER_DATA_DESC_SET,
        MIP_DATA_DESC_SHARED_GPS_TIME,
        extract_mip_shared_gps_timestamp_data_from_field,
        &filter_gps_timestamp
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[1],
        MIP_FILTER_DATA_DESC_SET,
        MIP_DATA_DESC_FILTER_FILTER_STATUS,
        extract_mip_filter_status_data_from_field,
        &filter_status
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[2],
        MIP_FILTER_DATA_DESC_SET,
        MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES,
        extract_mip_filter_euler_angles_data_from_field,
        &filter_euler_angles
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[3],
        MIP_FILTER_DATA_DESC_SET,
        MIP_DATA_DESC_FILTER_POS_LLH,
        extract_mip_filter_position_llh_data_from_field,
        &filter_position_llh
    );

    mip_interface_register_field_callback(
        &device,
        &filter_data_handlers[4],
        MIP_FILTER_DATA_DESC_SET,
        MIP_DATA_DESC_SHARED_EVENT_SOURCE,
        handle_event_triggers,
        NULL
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    printf("Resuming the device.\n");
    cmd_result = mip_base_resume(&device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(&device, "Could not resume the device!\n", cmd_result);
    }

    printf("Sensor is configured... waiting for filter to initialize.\n");

    mip_filter_mode current_state = filter_status.filter_state;

    // Wait for the device to initialize
    while (filter_status.filter_state != MIP_FILTER_MODE_INIT)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        mip_interface_update(&device, 0, false);

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }
    }

    mip_timestamp previous_print_timestamp = 0;

    // Device loop
    // TODO: Update loop condition to allow for exiting
    while (true)
    {
        // Update the device state
        mip_interface_update(&device, 0, false);

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }

        // Print out data at 10 Hz
        const mip_timestamp current_time = get_current_timestamp();

        if (current_time - previous_print_timestamp >= 100)
        {
            if (filter_status.filter_state >= MIP_FILTER_MODE_AHRS)
            {
                printf("TOW = %f: ATT_EULER = [%f %f %f]\n",
                    filter_gps_timestamp.tow,
                    filter_euler_angles.roll,
                    filter_euler_angles.pitch,
                    filter_euler_angles.yaw
                );
            }
            else if (filter_status.filter_state >= MIP_FILTER_MODE_VERT_GYRO)
            {
                printf("TOW = %f: ATT_EULER = [%f %f]\n",
                    filter_gps_timestamp.tow,
                    filter_euler_angles.roll,
                    filter_euler_angles.pitch
                );
            }

            if (filter_status.filter_state >= MIP_FILTER_MODE_FULL_NAV)
            {
                printf("TOW = %f: POS_LLH = [%f %f %f]\n",
                    filter_gps_timestamp.tow,
                    filter_position_llh.latitude,
                    filter_position_llh.longitude,
                    filter_position_llh.ellipsoid_height
                );
            }

            previous_print_timestamp = current_time;
        }
    }

    printf("Example Completed Successfully.\n");
    printf("Closing the serial port and exiting.\n");
    serial_port_close(&device_port);

    return 0;
}

// Configure Sensor data message format
void configure_sensor_message_format(mip_interface* device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet (shown in GNSS setup)

    printf("Getting the base rate for sensor data.\n");
    uint16_t sensor_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not get sensor base rate format!\n", cmd_result);
    }

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation  = sensor_base_rate / sensor_sample_rate;

    const mip_descriptor_rate sensor_descriptors[4] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,     sensor_decimation },
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,  sensor_decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,   sensor_decimation },
    };

    printf("Configuring message format for sensor data.\n");
    cmd_result = mip_3dm_write_message_format(
        device,
        MIP_SENSOR_DATA_DESC_SET,
        sizeof(sensor_descriptors) / sizeof(sensor_descriptors[0]),
        sensor_descriptors
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set sensor message format!\n", cmd_result);
    }
}

// Configure Filter data message format
void configure_filter_message_format(mip_interface* device)
{
    printf("Getting the base rate for filter data.\n");
    uint16_t filter_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not get filter base rate format!\n", cmd_result);
    }

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation  = filter_base_rate / filter_sample_rate;

    const mip_descriptor_rate filter_descriptors[3] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
    };

    printf("Configuring message format for filter data.\n");
    cmd_result = mip_3dm_write_message_format(
        device,
        MIP_FILTER_DATA_DESC_SET,
        sizeof(filter_descriptors) / sizeof(filter_descriptors[0]),
        filter_descriptors
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set filter message format!\n", cmd_result);
    }
}

// Set up a trigger for filter euler angles
void configure_event_triggers(mip_interface* device)
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

    printf("Configuring threshold event trigger for roll on trigger instance ID %d.\n", trigger_instance_id);
    mip_cmd_result cmd_result = mip_3dm_write_event_trigger(
        device,                                       // Device
        trigger_instance_id,                          // Trigger instance ID
        MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD, // Trigger type
        &event_parameters                             // Trigger parameters
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set pitch event parameters!\n", cmd_result);
    }

    // Use the same trigger configuration, but set it to the y-axis (pitch)
    event_parameters.threshold.param_id = 2;

    // Note: This is independent of the param_id
    trigger_instance_id = 2;

    printf("Configuring threshold event trigger for pitch on trigger instance ID %d.\n", trigger_instance_id);
    cmd_result = mip_3dm_write_event_trigger(
        device,                                       // Device
        trigger_instance_id,                          // Trigger instance ID
        MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD, // Trigger type
        &event_parameters                             // Trigger parameters
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set roll event parameters!\n", cmd_result);
    }
}

// Note: Event trigger instance IDs do not need to match the Action instance IDs
void configure_event_actions(mip_interface* device)
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

    printf("Configuring message action instance ID %d for trigger instance ID %d (roll).\n",
        action_instance_id,
        trigger_instance_id
    );
    // Configure an action for event trigger 1 (roll)
    mip_cmd_result cmd_result = mip_3dm_write_event_action(
        device,                                    // Device
        action_instance_id,                        // Action instance ID
        trigger_instance_id,                       // Trigger instance ID to link to
        MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, // Action type
        &event_action_parameters                   // Action parameters
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set roll action parameters!\n", cmd_result);
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
    cmd_result = mip_3dm_write_event_action(
        device,                                    // Device
        action_instance_id,                        // Action instance ID
        trigger_instance_id,                       // Trigger instance ID to link to
        MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, // Action type
        &event_action_parameters                   // Action parameters
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set pitch action parameters!\n", cmd_result);
    }
}

// Enable the events
void enable_events(mip_interface* device)
{
    uint8_t event_trigger_instance_id = 1;

    // Enable the roll event trigger
    printf("Enabling event trigger instance ID %d (roll).\n", event_trigger_instance_id);
    mip_cmd_result cmd_result = mip_3dm_write_event_control(
        device,                                    // Device
        event_trigger_instance_id,                 // Event trigger instance ID to enable
        MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED // Event control mode
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not enable roll event!\n", cmd_result);
    }

    event_trigger_instance_id = 2;

    // Enable the pitch event trigger
    printf("Enabling event trigger instance ID %d (pitch).\n", event_trigger_instance_id);
    cmd_result = mip_3dm_write_event_control(
        device,                                    // Device
        event_trigger_instance_id,                 // Event trigger instance ID to enable
        MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED // Event control mode
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not enable pitch event!\n", cmd_result);
    }
}

// Handler for filter event source field
void handle_event_triggers(void* user, const mip_field_view* field, mip_timestamp timestamp)
{
    // Unused variables
    (void)user;
    (void)timestamp;

    mip_shared_event_source_data event_source;

    if (!extract_mip_shared_event_source_data_from_field(field, &event_source))
    {
        return;
    }

    // Event trigger instance ID 1 (roll)
    if (event_source.trigger_id == 1)
    {
        printf("EVENT: Roll event triggered! Trigger ID: %d\n", event_source.trigger_id);
    }
    // Event trigger instance ID 2 (pitch)
    else if (event_source.trigger_id == 2)
    {
        printf("EVENT: Pitch event triggered! Trigger ID: %d\n", event_source.trigger_id);
    }
}

void display_filter_state(const mip_filter_mode filter_state)
{
    printf("The filter has entered ");

    switch (filter_state)
    {
        case MIP_FILTER_MODE_INIT:
        {
            printf("initialization mode. INIT (%d)", (uint8_t)filter_state);
            break;
        }
        case MIP_FILTER_MODE_VERT_GYRO:
        {
            printf("vertical gyro mode. VERT_GYRO (%d)", (uint8_t)filter_state);
            break;
        }
        case MIP_FILTER_MODE_AHRS:
        {
            printf("AHRS mode. AHRS (%d)", (uint8_t)filter_state);
            break;
        }
        case MIP_FILTER_MODE_FULL_NAV:
        {
            printf("full navigation mode. FULL_NAV (%d)", (uint8_t)filter_state);
            break;
        }
        default:
        {
            printf("startup mode. STARTUP (%d)", (uint8_t)filter_state);
            break;
        }
    }

    printf("\n");
}

mip_timestamp get_current_timestamp()
{
    time_t t;
    time(&t);

    double delta = difftime(t, g_start_time);

    return (mip_timestamp)(delta * 1000);
}

bool mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length)
{
    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(device);

    if (device_port == NULL)
    {
        printf("ERROR: serial_port pointer not set in mip_interface_init()\n");
        return false;
    }

    // Get the bytes written to the device
    size_t bytes_written;

    // Send the packet to the device
    return serial_port_write(device_port, data, length, &bytes_written);
}

bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, bool from_cmd,
    size_t* length_out, mip_timestamp* timestamp_out)
{
    // Unused variable
    (void)from_cmd;

    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(device);

    if (device_port == NULL)
    {
        printf("ERROR: serial_port pointer not set in mip_interface_init()\n");
        return false;
    }

    // Get the time that packet was received
    *timestamp_out = get_current_timestamp();

    // Read the packet from the device
    return serial_port_read(device_port, buffer, max_length, (int)wait_time, length_out);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize a MIP device and send some commands to prepare for configuration
///
/// @param device      Device to initialize
/// @param device_port Serial port to use for the device connection
/// @param baudrate    Baudrate to open the connection with
///
void initialize_device(mip_interface* device, serial_port* device_port, uint32_t baudrate)
{
    printf("Initializing device interface.\n");
    mip_interface_init(
        device,
        mip_timeout_from_baudrate(baudrate),  // Set the base timeout for commands (milliseconds)
        500,                                  // Set the base timeout for command replies (milliseconds)
        &mip_interface_user_send_to_device,   // User-defined send packet callback
        &mip_interface_user_recv_from_device, // User-defined receive packet callback
        &mip_interface_default_update,        // Default update callback
        (void*)device_port                    // Cast the device port for use in the callbacks
    );

    // Create a command result to check/print results when running commands
    mip_cmd_result cmd_result;

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    printf("Pinging the device.\n");
    cmd_result = mip_base_ping(device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not ping the device!", cmd_result);
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    printf("Setting device to idle.\n");
    cmd_result = mip_base_set_idle(device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not set the device to idle!\n", cmd_result);
    }

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    printf("Loading default settings.\n");
    cmd_result = mip_3dm_default_device_settings(device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        terminate(device, "Could not load device default settings!\n", cmd_result);
    }
}

// Print an error message and terminate the program after closing the serial port
void terminate(mip_interface* device, const char* message, mip_cmd_result cmd_result)
{
    printf("ERROR: %s Command Result: %d %s\n", message, cmd_result, mip_cmd_result_to_string(cmd_result));

    if (device != NULL)
    {
        // Get the serial connection pointer that was set during device initialization
        serial_port* device_port = (serial_port*)mip_interface_user_pointer(device);

        if (device_port == NULL)
        {
            printf("ERROR: serial_port pointer not set in mip_interface_init(). Cannot close the port.\n");
        }
        else
        {
            printf("Closing the serial port.\n");
            serial_port_close(device_port);
        }
    }

    printf("Exiting the program.\n");

    exit(1);
}
