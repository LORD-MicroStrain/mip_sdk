////////////////////////////////////////////////////////////////////////////////
/// @file 7_series_ins_example.c
///
/// @defgroup _7_series_ins_example_c 7-Series INS Example [C]
///
/// @ingroup examples_c
///
/// @brief Example setup program for the 3DM-CV7-INS, and 3DM-GV7-INS using C
///
/// @details This example shows a basic setup to configure the navigation filter
///          with external heading, and GNSS position and velocity as the
///          heading sources to stream filter data using simulated external
///          aiding measurements for the 3DM-CV7-INS, and 3DM-GV7-INS with
///          external aiding measurements using C. This is not an exhaustive
///          example of all settings for those devices. If this example does not
///          meet your specific setup needs, please consult the MIP SDK API
///          documentation for the proper commands.
///
/// @section _7_series_ins_example_c_license License
///
/// @copyright Copyright (c) 2025 MicroStrain by HBK
///            Licensed under MIT License
///
/// @{
///

// Include the MicroStrain Serial connection header
#include <microstrain/connections/serial/serial_port.h>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/mip_interface.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_aiding.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_filter.h>
#include <mip/definitions/data_filter.h>
#include <mip/definitions/data_shared.h>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
/// @brief  Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static const char* PORT_NAME = "COM1";
#else  // Unix
static const char* PORT_NAME = "/dev/ttyACM0";
#endif // _WIN32

/// @brief  Set the baudrate for the connection (Serial/USB)
/// @note For native serial connections this needs to be 115200 due to the device default settings command
// Use mip_base_*_comm_speed() to write and save the baudrate on the device
static const uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
/// @brief Streaming rate in Hz
static const uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
/// @brief Example run time
static const uint32_t RUN_TIME_SECONDS = 30;
////////////////////////////////////////////////////////////////////////////////

///
/// @} group _7_series_ins_example_c
////////////////////////////////////////////////////////////////////////////////

// Time of arrival latency in nanoseconds
// Note: This is the time it takes to package the command before it arrives and is typically around 100 ms
static const uint64_t TIME_OF_ARRIVAL_LATENCY_NS = 100 * 1000000;

// Frame config identifiers
static const uint8_t HEADING_FRAME_CONFIG_ID       = 1;
static const uint8_t GNSS_FRAME_CONFIG_ID          = 2;
static const uint8_t BODY_VELOCITY_FRAME_CONFIG_ID = 3;

// Custom logging handler callback
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
static void capture_gyro_bias(mip_interface* _device);

// Filter message format configuration
static void configure_filter_message_format(mip_interface* _device);

// External aiding configuration
static void configure_external_aiding_heading(mip_interface* _device);
static void configure_external_aiding_gnss_antenna(mip_interface* _device);
static void configure_external_aiding_ned_velocity(mip_interface* _device);

// Filter initialization
static void initialize_filter(mip_interface* _device);

// Utility to display filter state changes
static void display_filter_state(const mip_filter_mode _filter_state);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
static mip_timestamp get_current_timestamp();

// Device callbacks used for reading and writing packets
static bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length);
static bool mip_interface_user_recv_from_device(
    mip_interface* _device, uint8_t* _buffer, size_t _max_length, mip_timeout _wait_time, bool _from_cmd,
    size_t* _length_out, mip_timestamp* _timestamp_out
);

// Common device initialization procedure
static void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Utilities to send simulated external data to the device
// Note: All of this data should ideally come from a valid external source
static void send_simulated_heading_data(mip_interface* _device, const mip_time* _aiding_time);
static void send_simulated_position_data(mip_interface* _device, const mip_time* _aiding_time);
static void send_simulated_ned_velocity_data(mip_interface* _device, const mip_time* _aiding_time);
static void send_simulated_vehicle_frame_velocity_data(mip_interface* _device, const mip_time* _aiding_time);

// Utility functions the handle application closing and printing error messages
static void terminate(serial_port* _device_port, const char* _message, const bool _successful);
static void exit_from_command(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...);

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
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    serial_port device_port;
    serial_port_init(&device_port);

    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the connection to the device
    if (!serial_port_open(&device_port, PORT_NAME, BAUDRATE))
    {
        terminate(&device_port, "Could not open the connection!\n", false);
    }

    mip_interface device;
    initialize_device(&device, &device_port, BAUDRATE);

    // Capture gyro bias
    capture_gyro_bias(&device);

    // Configure the message format for filter data
    configure_filter_message_format(&device);

    // Configure the external aiding measurements
    configure_external_aiding_heading(&device);
    configure_external_aiding_gnss_antenna(&device);
    configure_external_aiding_ned_velocity(&device);

    // Initialize the navigation filter
    initialize_filter(&device);

    // Register filter data callbacks
    MICROSTRAIN_LOG_INFO("Registering filter data callbacks.\n");

    mip_dispatch_handler filter_data_handlers[5];

    // Data stores for filter data
    mip_shared_gps_timestamp_data filter_gps_timestamp;
    mip_filter_status_data        filter_status;
    mip_filter_position_llh_data  filter_position_llh;
    mip_filter_velocity_ned_data  filter_velocity_ned;
    mip_filter_euler_angles_data  filter_euler_angles;

    // Register the callbacks for the filter fields

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[0],
        MIP_FILTER_DATA_DESC_SET,                     // Data descriptor set
        MIP_DATA_DESC_SHARED_GPS_TIME,                // Data field descriptor set
        extract_mip_filter_timestamp_data_from_field, // Callback
        &filter_gps_timestamp                         // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[1],
        MIP_FILTER_DATA_DESC_SET,                  // Data descriptor set
        MIP_DATA_DESC_FILTER_FILTER_STATUS,        // Data field descriptor set
        extract_mip_filter_status_data_from_field, // Callback
        &filter_status                             // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[2],
        MIP_FILTER_DATA_DESC_SET,                        // Data descriptor set
        MIP_DATA_DESC_FILTER_POS_LLH,                    // Data field descriptor set
        extract_mip_filter_position_llh_data_from_field, // Callback
        &filter_position_llh                             // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[3],
        MIP_FILTER_DATA_DESC_SET,                        // Data descriptor set
        MIP_DATA_DESC_FILTER_VEL_NED,                    // Data field descriptor set
        extract_mip_filter_velocity_ned_data_from_field, // Callback
        &filter_velocity_ned                             // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &filter_data_handlers[4],
        MIP_FILTER_DATA_DESC_SET,                        // Data descriptor set
        MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES,           // Data field descriptor set
        extract_mip_filter_euler_angles_data_from_field, // Callback
        &filter_euler_angles                             // Data field out
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    const mip_cmd_result cmd_result = mip_base_resume(&device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("The device is configured... waiting for the filter to enter full navigation mode.\n");

    mip_filter_mode current_state = filter_status.filter_state;

    mip_timestamp previous_external_data_timestamp = 0;

    const mip_time external_measurement_time = {
        MIP_TIME_TIMEBASE_TIME_OF_ARRIVAL,
        1,                         // Reserved (needs to be 1)
        TIME_OF_ARRIVAL_LATENCY_NS // Nanoseconds
    };

    // Wait for the device to initialize
    while (filter_status.filter_state < MIP_FILTER_MODE_FULL_NAV)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        mip_interface_update(
            &device,
            10,   // Time to wait
            false // From command
        );

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }

        const mip_timestamp current_timestamp = get_current_timestamp();

        // Send the external updates every 500 ms
        if (current_timestamp - previous_external_data_timestamp >= 500)
        {
            send_simulated_heading_data(&device, &external_measurement_time);
            send_simulated_position_data(&device, &external_measurement_time);
            send_simulated_ned_velocity_data(&device, &external_measurement_time);
            send_simulated_vehicle_frame_velocity_data(&device, &external_measurement_time);

            previous_external_data_timestamp = current_timestamp;
        }
    }

    // Get the start time of the device update loop to handle exiting the application
    const mip_timestamp loop_start_time = get_current_timestamp();

    mip_timestamp previous_print_timestamp = 0;

    // Running loop
    // Exit after a predetermined time in seconds
    while (get_current_timestamp() - loop_start_time <= RUN_TIME_SECONDS * 1000)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        mip_interface_update(
            &device,
            10,   // Time to wait
            false // From command
        );

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }

        const mip_timestamp current_timestamp = get_current_timestamp();

        // Send the external updates every 500 ms
        if (current_timestamp - previous_external_data_timestamp >= 500)
        {
            send_simulated_heading_data(&device, &external_measurement_time);
            send_simulated_position_data(&device, &external_measurement_time);
            send_simulated_ned_velocity_data(&device, &external_measurement_time);
            send_simulated_vehicle_frame_velocity_data(&device, &external_measurement_time);

            previous_external_data_timestamp = current_timestamp;
        }

        // Print out data based on the sample rate (1000 ms / SAMPLE_RATE_HZ)
        if (current_timestamp - previous_print_timestamp >= 1000 / SAMPLE_RATE_HZ)
        {
            if (filter_status.filter_state >= MIP_FILTER_MODE_FULL_NAV)
            {
                MICROSTRAIN_LOG_INFO(
                    "%s = %10.3f%16s = [%9.6f, %9.6f, %9.6f]%16s = [%9.6f, %9.6f, %9.6f]%16s = [%9.6f, %9.6f, %9.6f]\n",
                    "TOW",
                    filter_gps_timestamp.tow,
                    "LLH Position",
                    filter_position_llh.latitude,
                    filter_position_llh.longitude,
                    filter_position_llh.ellipsoid_height,
                    "NED Velocity",
                    filter_velocity_ned.north,
                    filter_velocity_ned.east,
                    filter_velocity_ned.down,
                    "Euler Angles",
                    filter_euler_angles.roll,
                    filter_euler_angles.pitch,
                    filter_euler_angles.yaw
                );
            }

            previous_print_timestamp = current_timestamp;
        }
    }

    terminate(&device_port, "Example Completed Successfully.\n", true);

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
/// @ingroup _7_series_ins_example_c
///
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
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
/// @param _device Pointer to the initialized MIP device interface
///
/// @ingroup _7_series_ins_example_c
///
static void capture_gyro_bias(mip_interface* _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip_cmd_queue*    cmd_queue        = mip_interface_cmd_queue(_device);
    const mip_timeout previous_timeout = mip_cmd_queue_base_reply_timeout(cmd_queue);
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previous_timeout);

    // Note: The default is 15 s (15,000 ms)
    const uint16_t capture_duration            = 15000;
    const uint16_t increased_cmd_reply_timeout = capture_duration + 1000;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n", increased_cmd_reply_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, increased_cmd_reply_timeout);

    mip_vector3f gyro_bias = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: When capturing gyro bias, the device needs to remain still on a flat surface
    MICROSTRAIN_LOG_WARN("About to capture gyro bias for %.2g seconds!\n", (float)capture_duration / 1000.0f);
    MICROSTRAIN_LOG_WARN("Please do not move the device during this time!\n");
    MICROSTRAIN_LOG_WARN("Press 'Enter' when ready...");

    // Wait for anything to be entered
    const int confirm_capture = getc(stdin);
    (void)confirm_capture; // Unused

    MICROSTRAIN_LOG_WARN("Capturing gyro bias...\n");
    const mip_cmd_result cmd_result = mip_3dm_capture_gyro_bias(
        _device,
        capture_duration, // Capture duration (ms)
        gyro_bias         // Gyro bias out (result of the capture)
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f, %f, %f]\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previous_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, previous_timeout);
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
///             - LLH position
///             - NED velocity
///             - Euler angles
///
/// @param _device Pointer to the initialized MIP device interface
///
/// @ingroup _7_series_ins_example_c
///
static void configure_filter_message_format(mip_interface* _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t       filter_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_FILTER_DATA_DESC_SET, // Data descriptor set
        &filter_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the base rate for filter data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > filter_base_rate)
    {
        exit_from_command(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            filter_base_rate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t filter_decimation = filter_base_rate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating filter base rate %d by %d to stream data at %dHz.\n",
        filter_base_rate,
        filter_decimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate filter_descriptors[5] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_POS_LLH,          filter_decimation },
        { MIP_DATA_DESC_FILTER_VEL_NED,          filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for filter data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_FILTER_DATA_DESC_SET,                                   // Data descriptor set
        sizeof(filter_descriptors) / sizeof(filter_descriptors[0]), // Number of descriptors to include
        filter_descriptors                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure message format for filter data!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures a heading reference frame for external aiding measurements
///
/// @details Sets up a heading reference frame for external sensor data:
///             - Translation: [0, 0, 0] m
///             - Rotation: [0, 0, 0] deg (no rotation)
///
///          The frame is configured with tracking enabled and uses an Euler
///          angle rotation format.
///
/// @param _device Pointer to the initialized MIP device interface
///
/// @note This function is typically called during device initialization to
///       establish the coordinate system relationships for external
///       measurements. Frame IDs correspond to those used in external
///       measurement functions.
///
/// @ingroup _7_series_ins_example_c
///
static void configure_external_aiding_heading(mip_interface* _device)
{
    MICROSTRAIN_LOG_INFO("Configuring the reference frame for external heading.\n");
    const mip_vector3f external_heading_translation = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    mip_aiding_frame_config_command_rotation external_heading_rotation;
    external_heading_rotation.euler[0] = 0.0f; // Roll
    external_heading_rotation.euler[1] = 0.0f; // Pitch
    external_heading_rotation.euler[2] = 0.0f; // Yaw

    const mip_cmd_result cmd_result = mip_aiding_write_frame_config(
        _device,
        HEADING_FRAME_CONFIG_ID,
        MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER,
        true, // Tracking enabled
        external_heading_translation,
        &external_heading_rotation
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure the reference frame for external heading!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures a GNSS antenna reference frame for external aiding
///        measurements
///
/// @details Sets up a GNSS antenna reference frame for external sensor data:
///             - Translation: [0, 1, 0] m (1m offset in Y-axis)
///             - Rotation: [0, 0, 0] deg (no rotation)
///
///          The frame is configured with tracking enabled and uses an Euler
///          angle rotation format.
///
/// @param _device Pointer to the initialized MIP device interface
///
/// @note This function is typically called during device initialization to
///       establish the coordinate system relationships for external
///       measurements. Frame IDs correspond to those used in external
///       measurement functions.
///
/// @ingroup _7_series_ins_example_c
///
static void configure_external_aiding_gnss_antenna(mip_interface* _device)
{
    MICROSTRAIN_LOG_INFO("Configuring the reference frame for external GNSS antenna.\n");
    const mip_vector3f external_gnss_antenna_translation = {
        0.0f, // X
        1.0f, // Y
        0.0f  // Z
    };

    mip_aiding_frame_config_command_rotation external_gnss_antenna_rotation;
    external_gnss_antenna_rotation.euler[0] = 0.0f; // Roll
    external_gnss_antenna_rotation.euler[1] = 0.0f; // Pitch
    external_gnss_antenna_rotation.euler[2] = 0.0f; // Yaw

    const mip_cmd_result cmd_result = mip_aiding_write_frame_config(
        _device,
        GNSS_FRAME_CONFIG_ID,
        MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER,
        true, // Tracking enabled
        external_gnss_antenna_translation,
        &external_gnss_antenna_rotation
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure the reference frame for external GNSS antenna!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures a body frame velocity reference frame for external aiding
///        measurements
///
/// @details Sets up a body frame velocity reference frame for external sensor
///          data:
///             - Translation: [1, 0, 0] m (1m offset in X-axis)
///             - Rotation: [0, 0, 90] deg (90 deg yaw rotation)
///
///          The frame is configured with tracking enabled and uses an Euler
///          angle rotation format.
///
/// @param _device Reference to the initialized MIP device interface
///
/// @note This function is typically called during device initialization to
///       establish the coordinate system relationships for external
///       measurements. Frame IDs correspond to those used in external
///       measurement functions.
///
/// @ingroup _7_series_ins_example_c
///
static void configure_external_aiding_ned_velocity(mip_interface* _device)
{
    MICROSTRAIN_LOG_INFO("Configuring the reference frame for external body frame velocity.\n");
    const mip_vector3f external_body_frame_velocity_translation = {
        1.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    mip_aiding_frame_config_command_rotation external_body_frame_velocity_rotation;
    external_body_frame_velocity_rotation.euler[0] = 0.0f;                           // Roll
    external_body_frame_velocity_rotation.euler[1] = 0.0f;                           // Pitch
    external_body_frame_velocity_rotation.euler[2] = (float)(90.0f * M_PI / 180.0f); // Yaw at 90 degrees (as radians)

    const mip_cmd_result cmd_result = mip_aiding_write_frame_config(
        _device,
        BODY_VELOCITY_FRAME_CONFIG_ID,
        MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER,
        true, // Tracking enabled
        external_body_frame_velocity_translation,
        &external_body_frame_velocity_rotation
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure the reference frame for external body frame velocity!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and resets the navigation filter
///
/// @details Configures the navigation filter by:
///          1. Enabling GNSS position and velocity aiding measurements
///          2. Enabling external heading aiding measurements
///          3. Configuring filter initialization settings:
///             - Setting initial position and velocity to zero
///             - Enabling automatic position/velocity/attitude determination
///             - Configuring external aiding kinematic alignment
///          4. Resetting the filter to apply new settings
///
/// @param _device Pointer to the initialized MIP device interface
///
/// @ingroup _7_series_ins_example_c
///
static void initialize_filter(mip_interface* _device)
{
    // Configure Filter Aiding Measurements
    MICROSTRAIN_LOG_INFO("Enabling the aiding measurement source for GNSS position and velocity.\n");
    mip_cmd_result cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, // Aiding Source type
        true                                                                     // Enabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(
            _device,
            cmd_result,
            "Could not enable the aiding measurement source for GNSS position and velocity!\n"
        );
    }

    MICROSTRAIN_LOG_INFO("Enabling the aiding measurement source for external heading.\n");
    cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_EXTERNAL_HEADING, // Aiding Source type
        true                                                                         // Enabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not enable the aiding measurement source for external heading!\n");
    }

    // Configure the filter initialization

    const mip_vector3f initial_position = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    const mip_vector3f initial_velocity = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: This is the default setting on the device and will automatically configure
    // the initial conditions required to initialize the filter based on the alignment selector
    // and readings from the device's sensors
    const mip_filter_initialization_configuration_command_initial_condition_source initial_condition_source =
        MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT;

    const mip_filter_initialization_configuration_command_alignment_selector initial_alignment_selector =
        MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC |
        MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_EXTERNAL;

    MICROSTRAIN_LOG_INFO("Setting the filter initialization configuration.\n");
    cmd_result = mip_filter_write_initialization_configuration(
        _device,
        0, // Initialize the filter after receiving the filter run command (disabled)
        initial_condition_source,
        initial_alignment_selector, // Bitfield value
        0.0f,                       // Initial heading
        0.0f,                       // Initial pitch
        0.0f,                       // Initial roll
        initial_position,
        initial_velocity,
        MIP_FILTER_REFERENCE_FRAME_LLH // Reference frame selector
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not set the filter initialization configuration!\n");
    }

    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to reset the navigation filter.\n");
    cmd_result = mip_filter_reset(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not reset the navigation filter!\n");
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
/// @param _filter_state Current filter mode from the MIP device interface
///
/// @ingroup _7_series_ins_example_c
///
static void display_filter_state(const mip_filter_mode _filter_state)
{
    const char* mode_description = "startup";
    const char* mode_type        = "STARTUP";

    switch (_filter_state)
    {
        case MIP_FILTER_MODE_INIT:
        {
            mode_description = "initialization";
            mode_type        = "MIP_FILTER_MODE_INIT";
            break;
        }
        case MIP_FILTER_MODE_VERT_GYRO:
        {
            mode_description = "vertical gyro";
            mode_type        = "MIP_FILTER_MODE_VERT_GYRO";
            break;
        }
        case MIP_FILTER_MODE_AHRS:
        {
            mode_description = "AHRS";
            mode_type        = "MIP_FILTER_MODE_AHRS";
            break;
        }
        case MIP_FILTER_MODE_FULL_NAV:
        {
            mode_description = "full navigation";
            mode_type        = "MIP_FILTER_MODE_FULL_NAV";
            break;
        }
        default:
        {
            break;
        }
    }

    MICROSTRAIN_LOG_INFO("The filter has entered %s mode. (%d) %s\n", mode_description, (uint8_t)_filter_state, mode_type);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the current system timestamp in milliseconds
///
/// @details Provides basic timestamping using system time:
///          - Returns milliseconds since Unix epoch
///          - Uses timespec_get() with UTC time base
///          - Returns 0 if time cannot be obtained
///
/// @note Update this function to use a different time source if needed for
///       your specific application requirements
///
/// @return Current system time in milliseconds since epoch
///
/// @ingroup _7_series_ins_example_c
///
static mip_timestamp get_current_timestamp()
{
    struct timespec ts;

    // Get system UTC time since epoch
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC)
    {
        return 0;
    }

    // Get the time in milliseconds
    return (mip_timestamp)ts.tv_sec * 1000 + (mip_timestamp)ts.tv_nsec / 1000000;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles sending packets to the device
///
/// @details Implements the MIP device interface send callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Writes data buffer to serial port
///
/// @param _device MIP device interface containing the connection
/// @param _data Buffer containing packet data to send
/// @param _length Number of bytes to send
///
/// @return True if send was successful, false otherwise
///
/// @ingroup _7_series_ins_example_c
///
static bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length)
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

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles receiving packets from the device
///
/// @details Implements the MIP device interface receive callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Reads available data into provided buffer
///          - Timestamps the received data
///
/// @param _device MIP device interface containing the connection
/// @param _buffer Buffer to store received data
/// @param _max_length Maximum number of bytes to read
/// @param _wait_time How long to wait for data in milliseconds
/// @param _from_cmd Whether this read is from a command response (unused)
/// @param _length_out Number of bytes actually read
/// @param _timestamp_out Timestamp when data was received
///
/// @return True if receive was successful, false otherwise
///
/// @ingroup _7_series_ins_example_c
///
static bool mip_interface_user_recv_from_device(
    mip_interface* _device, uint8_t* _buffer, size_t _max_length, mip_timeout _wait_time, bool _from_cmd,
    size_t* _length_out, mip_timestamp* _timestamp_out
)
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

    // Get the time that the packet was received (system epoch UTC time in milliseconds)
    *_timestamp_out = get_current_timestamp();

    // Read the packet from the device
    return serial_port_read(device_port, _buffer, _max_length, (int)_wait_time, _length_out);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and configures a MIP device interface
///
/// @details Performs a complete device initialization sequence:
///          1. Sets up a MIP device interface with specified timeouts and
///             callbacks
///          2. Verifies device communication with a ping command
///          3. Sets the device to idle mode to ensure reliable configuration
///          4. Queries and displays detailed device information
///          5. Loads default device settings for a known state
///
/// @param _device Pointer to a MIP device interface to initialize
/// @param _device_port Pointer to an initialized serial port for device
///                     communication
/// @param _baudrate Serial communication baudrate for the device
///
/// @ingroup _7_series_ins_example_c
///
static void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate)
{
    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip_interface_init(
        _device,
        mip_timeout_from_baudrate(_baudrate), // Set the base timeout for commands (milliseconds)
        2000,                                 // Set the base timeout for command replies (milliseconds)
        &mip_interface_user_send_to_device,   // User-defined send packet callback
        &mip_interface_user_recv_from_device, // User-defined receive packet callback
        &mip_interface_default_update,        // Default update callback
        (void*)_device_port                   // Cast the device port for use in the callbacks
    );

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    mip_cmd_result cmd_result = mip_base_ping(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmd_result = mip_base_set_idle(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
    mip_base_device_info device_info;
    cmd_result = mip_base_get_device_info(_device, &device_info);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = device_info.firmware_version / 1000;
    const uint16_t minor = device_info.firmware_version / 100 % 10;
    const uint16_t patch = device_info.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16];
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d", major, minor, patch);

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name", device_info.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number", device_info.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number", device_info.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number", device_info.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options", device_info.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n", "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    MICROSTRAIN_LOG_INFO("Loading device default settings.\n");
    cmd_result = mip_3dm_default_device_settings(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        // Note: Default settings will reset the baudrate to 115200 and may cause connection issues
        if (cmd_result == MIP_STATUS_TIMEDOUT && BAUDRATE != 115200)
        {
            MICROSTRAIN_LOG_WARN("On a native serial connections the baudrate needs to be 115200 for this example to run.\n");
        }

        exit_from_command(_device, cmd_result, "Could not load device default settings!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Sends simulated external heading measurements to the device
///
/// @details Provides simulated true heading data to the device for filter
///          aiding. Uses fixed values:
///          - Heading:     0.0 deg (North)
///          - Uncertainty: 0.001 radians
///
/// @param _device Pointer to the initialized MIP device interface
/// @param _aiding_time Pointer to the timestamp for the external measurement
///
/// @note Issues warning if the command fails but does not terminate execution.
///       Used for testing external aiding functionality with known data.
///
/// @ingroup _7_series_ins_example_c
///
static void send_simulated_heading_data(mip_interface* _device, const mip_time* _aiding_time)
{
    const float heading     = 0.0f;
    const float uncertainty = 0.001f;

    const mip_cmd_result cmd_result = mip_aiding_heading_true(
        _device,
        _aiding_time,
        HEADING_FRAME_CONFIG_ID,
        heading,
        uncertainty,
        0x0001 // Valid flags (true/false)
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        MICROSTRAIN_LOG_WARN("Failed to send external true heading to the device!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Sends simulated external position measurements to the device
///
/// @details Provides simulated LLH position data to the device for filter
///          aiding. Uses fixed coordinates for MicroStrain headquarters:
///          - Latitude:    44.437 deg N
///          - Longitude:   73.106 deg W
///          - Height:      122.0 m
///          - Uncertainty: 1.0 m in all axes
///
/// @param _device Pointer to the initialized MIP device interface
/// @param _aiding_time Pointer to the timestamp for the external measurement
///
/// @note Issues warning if the command fails but does not terminate execution.
///       Used for testing external aiding functionality with a known location.
///
/// @ingroup _7_series_ins_example_c
///
static void send_simulated_position_data(mip_interface* _device, const mip_time* _aiding_time)
{
    // Coordinates for MicroStrain headquarters
    const double latitude  = 44.43729093897896;
    const double longitude = -73.10628129871753;
    const double height    = 122.0;

    const mip_vector3f uncertainty = {
        1.0f, // X
        1.0f, // Y
        1.0f  // Z
    };

    const mip_cmd_result cmd_result = mip_aiding_pos_llh(
        _device,
        _aiding_time,
        GNSS_FRAME_CONFIG_ID,
        latitude,
        longitude,
        height,
        uncertainty,
        MIP_AIDING_POS_LLH_COMMAND_VALID_FLAGS_ALL
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        MICROSTRAIN_LOG_WARN("Failed to send external LLH position to the device!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Sends simulated external NED velocity measurements to the device
///
/// @details Provides simulated North-East-Down velocity data to the device for
///          filter aiding. Uses stationary target values:
///          - Velocity:    [0, 0, 0] m/s (stationary)
///          - Uncertainty: 0.1 m/s in all axes
///
/// @param _device Pointer to the initialized MIP device interface
/// @param _aiding_time Pointer to the timestamp for the external measurement
///
/// @note Issues warning if the command fails but does not terminate execution.
///       Used for testing external aiding functionality with stationary data.
///
/// @ingroup _7_series_ins_example_c
///
static void send_simulated_ned_velocity_data(mip_interface* _device, const mip_time* _aiding_time)
{
    const mip_vector3f velocity = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    const mip_vector3f uncertainty = {
        0.1f, // X
        0.1f, // Y
        0.1f  // Z
    };

    const mip_cmd_result cmd_result = mip_aiding_vel_ned(
        _device,
        _aiding_time,
        GNSS_FRAME_CONFIG_ID,
        velocity,
        uncertainty,
        MIP_AIDING_VEL_NED_COMMAND_VALID_FLAGS_ALL
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        MICROSTRAIN_LOG_WARN("Failed to send external NED velocity to the device!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Sends simulated external vehicle frame velocity measurements to the
///        device
///
/// @details Provides simulated body-frame velocity data to the device for
///          filter aiding. Uses stationary target values:
///          - Velocity:    [0, 0, 0] m/s (stationary in body frame)
///          - Uncertainty: 0.1 m/s in all axes
///
/// @param _device Pointer to the initialized MIP device interface
/// @param _aiding_time Pointer to the timestamp for the external measurement
///
/// @note Issues warning if the command fails but does not terminate execution.
///       Used for testing external aiding functionality with vehicle-relative data.
///
/// @ingroup _7_series_ins_example_c
///
static void send_simulated_vehicle_frame_velocity_data(mip_interface* _device, const mip_time* _aiding_time)
{
    const mip_vector3f velocity = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    const mip_vector3f uncertainty = {
        0.1f, // X
        0.1f, // Y
        0.1f  // Z
    };

    const mip_cmd_result cmd_result = mip_aiding_vel_body_frame(
        _device,
        _aiding_time,
        BODY_VELOCITY_FRAME_CONFIG_ID,
        velocity,
        uncertainty,
        MIP_AIDING_VEL_BODY_FRAME_COMMAND_VALID_FLAGS_ALL
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        MICROSTRAIN_LOG_WARN("Failed to send external body frame velocity to the device!\n");
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
/// @param _device_port Serial port connection to close
/// @param _message Error message to display
/// @param _successful Whether termination is due to success or failure
///
/// @ingroup _7_series_ins_example_c
///
static void terminate(serial_port* _device_port, const char* _message, const bool _successful)
{
    if (_message != NULL && strlen(_message) != 0)
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
        MICROSTRAIN_LOG_ERROR("Connection not set for the device interface. Cannot close the connection.\n");
    }
    else
    {
        if (serial_port_is_open(_device_port))
        {
            MICROSTRAIN_LOG_INFO("Closing the connection.\n");

            if (!serial_port_close(_device_port))
            {
                MICROSTRAIN_LOG_ERROR("Failed to close the connection!\n");
            }
        }
    }

    MICROSTRAIN_LOG_INFO("Press 'Enter' to exit the program.\n");

    // Make sure the console remains open
    const int confirm_exit = getc(stdin);
    (void)confirm_exit; // Unused

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
/// @param _cmd_result Result code from a failed command
/// @param _format Printf-style format string for error message
/// @param ... Variable arguments for format string
///
/// @ingroup _7_series_ins_example_c
///
static void exit_from_command(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...)
{
    if (_format != NULL && strlen(_format) != 0)
    {
        va_list args;
        va_start(args, _format);
        MICROSTRAIN_LOG_ERROR_V(_format, args);
        va_end(args);
    }

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmd_result, mip_cmd_result_to_string(_cmd_result));

    if (_device == NULL)
    {
        terminate(NULL, "", false);
    }
    else
    {
        // Get the connection pointer that was set during device initialization
        serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

        terminate(device_port, "", false);
    }
}
