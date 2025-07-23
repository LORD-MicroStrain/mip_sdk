////////////////////////////////////////////////////////////////////////////////
/// 7_series_gnss_ins_example.c
///
/// Example setup program for the 3DM-GQ7-GNSS/INS, and 3DM-CV7-GNSS/INS using C
///
/// This example shows a typical setup for the 3DM-GQ7-GNSS/INS and
/// 3DM-CV7-GNSS/INS sensors in a wheeled-vehicle application using C.
/// It is not an exhaustive example of all settings for those devices.
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
#include <microstrain/connections/serial/serial_port.h>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/mip_interface.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_filter.h>
#include <mip/definitions/data_filter.h>
#include <mip/definitions/data_gnss.h>
#include <mip/definitions/data_shared.h>

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
// Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static const char* PORT_NAME = "COM1";
#else // Unix
static const char* PORT_NAME = "/dev/ttyUSB0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
static const uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
// Streaming rate in Hz
static const uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
// Example run time
static const uint32_t RUN_TIME_SECONDS = 30;
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
void capture_gyro_bias(mip_interface* _device);

// GNSS message format configuration
void configure_gnss_message_format(mip_interface* _device);

// Filter message format configuration
void configure_filter_message_format(mip_interface* _device);

// Antenna configuration
void configure_antennas(mip_interface* _device);

// Filter initialization
void initialize_filter(mip_interface* _device);

// Utilities to display state changes
void display_gnss_fix_state(const mip_gnss_fix_info_data* _fix_info_array, const uint8_t _array_index);
void display_filter_state(const mip_filter_mode _filter_state);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
mip_timestamp get_current_timestamp();

// Device callbacks used for reading and writing packets
bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length);
bool mip_interface_user_recv_from_device(mip_interface* _device, uint8_t* _buffer, size_t _max_length,
    mip_timeout _wait_time, bool _from_cmd, size_t* _length_out, mip_timestamp* _timestamp_out);

// Common device initialization procedure
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Utility functions the handle application closing and printing error messages
void terminate(serial_port* _device_port, const char* _message, const bool _successful);
void command_failure_terminate(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format,
    ...);

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
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

    // Configure the message format for GNSS data
    configure_gnss_message_format(&device);

    // Configure the message format for filter data
    configure_filter_message_format(&device);

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

    // Configure the wheeled-vehicle constraint
    MICROSTRAIN_LOG_INFO("Enabling the wheeled-vehicle constraint.\n");
    cmd_result = mip_filter_write_wheeled_vehicle_constraint_control(
        &device,
        1 // Enabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not enable the wheeled-vehicle constraint!\n");
    }

    // Configure the GNSS antenna offsets
    configure_antennas(&device);

    // Initialize the navigation filter
    initialize_filter(&device);

    // Register data callbacks

    // Register GNSS data callbacks
    MICROSTRAIN_LOG_INFO("Registering GNSS data callbacks.\n");

    mip_dispatch_handler gnss_data_handlers[2];

    // Data stores for GNSS data
    mip_gnss_fix_info_data gnss_fix_info[2]; // GNSS 1 & 2

    // Register the callbacks for the GNSS fields

    mip_interface_register_extractor(
        &device,
        &gnss_data_handlers[0],
        MIP_GNSS1_DATA_DESC_SET,                   // Data descriptor set
        MIP_DATA_DESC_GNSS_FIX_INFO,               // Data field descriptor set
        extract_mip_gnss_fix_info_data_from_field, // Callback
        &gnss_fix_info[0]                          // Data field out
    );

    mip_interface_register_extractor(
        &device,
        &gnss_data_handlers[1],
        MIP_GNSS2_DATA_DESC_SET,                   // Data descriptor set
        MIP_DATA_DESC_GNSS_FIX_INFO,               // Data field descriptor set
        extract_mip_gnss_fix_info_data_from_field, // Callback
        &gnss_fix_info[1]                          // Data field out
    );

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
    cmd_result = mip_base_resume(&device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... waiting for filter to enter full navigation mode.\n");

    mip_gnss_fix_info_data_fix_type current_fix_type[2] = {
        gnss_fix_info[0].fix_type,
        gnss_fix_info[1].fix_type
    };
    mip_filter_mode current_state = filter_status.filter_state;

    // Wait for the device to initialize
    while (filter_status.filter_state != MIP_FILTER_MODE_FULL_NAV)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        mip_interface_update(
            &device,
            0,    // Time to wait
            false // From command
        );

        // Check for fix type state changes for each antenna
        for (uint8_t gnss_index = 0; gnss_index < 2; ++gnss_index)
        {
            // Fix type state change for GNSS
            if (current_fix_type[gnss_index] != gnss_fix_info[gnss_index].fix_type)
            {
                display_gnss_fix_state(gnss_fix_info, gnss_index);
                current_fix_type[gnss_index] = gnss_fix_info[gnss_index].fix_type;
            }
        }

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }
    }

    // Get the start time of the device update loop to handle exiting the application
    const mip_timestamp loop_start_time = get_current_timestamp();

    mip_timestamp previous_print_timestamp = 0;

    // Device loop
    // Exit after predetermined time in seconds
    while (get_current_timestamp() - loop_start_time <= RUN_TIME_SECONDS * 1000)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        mip_interface_update(
            &device,
            0,    // Time to wait
            false // From command
        );

        // Check for fix type state changes for each antenna
        for (uint8_t gnss_index = 0; gnss_index < 2; ++gnss_index)
        {
            // Fix type state change for GNSS
            if (current_fix_type[gnss_index] != gnss_fix_info[gnss_index].fix_type)
            {
                display_gnss_fix_state(gnss_fix_info, gnss_index);
                current_fix_type[gnss_index] = gnss_fix_info[gnss_index].fix_type;
            }
        }

        // Filter state change
        if (current_state != filter_status.filter_state)
        {
            display_filter_state(filter_status.filter_state);
            current_state = filter_status.filter_state;
        }

        const mip_timestamp current_timestamp = get_current_timestamp();

        // Print out data based on the sample rate (1000 ms / SAMPLE_RATE_HZ)
        if (current_timestamp - previous_print_timestamp >= 1000 / SAMPLE_RATE_HZ)
        {
            if (filter_status.filter_state >= MIP_FILTER_MODE_VERT_GYRO)
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
void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
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

////////////////////////////////////////////////////////////////////////////////
/// @brief Captures and configures device gyro bias
///
/// @param _device Pointer to the initialized MIP device interface
///
void capture_gyro_bias(mip_interface* _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip_cmd_queue*    cmd_queue        = mip_interface_cmd_queue(_device);
    const mip_timeout previous_timeout = mip_cmd_queue_base_reply_timeout(cmd_queue);
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previous_timeout);

    // Note: The default is 15 s (15,000 ms)
    const uint16_t capture_duration            = 15000;
    const uint16_t increased_cmd_reply_timeout = capture_duration + 1000;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n",
        increased_cmd_reply_timeout
    );
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, increased_cmd_reply_timeout);

    mip_vector3f gyro_bias = {
        0.0f,
        0.0f,
        0.0f
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
        command_failure_terminate(_device, cmd_result, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f, %f, %f]\n",
        gyro_bias[0],
        gyro_bias[1],
        gyro_bias[2]
    );

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previous_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, previous_timeout);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for dual GNSS data streaming
///
/// @details Sets up GNSS data output by:
///          1. Querying device base rates
///          2. Validating desired sample rate against base rates
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - Fix information
///
/// @param _device Pointer to the initialized MIP device interface
///
void configure_gnss_message_format(mip_interface* _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for GNSS 1 data.\n");
    uint16_t       gnss_base_rate_1;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_GNSS1_DATA_DESC_SET, // Data descriptor set
        &gnss_base_rate_1        // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get GNSS 1 base rate!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > gnss_base_rate_1)
    {
        command_failure_terminate(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            gnss_base_rate_1
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t gnss_decimation_1 = gnss_base_rate_1 / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO("Decimating GNSS 1 base rate %d by %d to stream data at %dHz.\n",
        gnss_base_rate_1,
        gnss_decimation_1,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate gnss_descriptors_1[1] = {
        { MIP_DATA_DESC_GNSS_FIX_INFO, gnss_decimation_1 }
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for GNSS 1 data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_GNSS1_DATA_DESC_SET,                                    // Data descriptor set
        sizeof(gnss_descriptors_1) / sizeof(gnss_descriptors_1[0]), // Number of descriptors to include
        gnss_descriptors_1                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set message format for GNSS 1 data!\n");
    }

    // Note: Typically, the base rates for all GNSS descriptors will be the same, but adding this for completeness
    MICROSTRAIN_LOG_INFO("Getting the base rate for GNSS 2 data.\n");
    uint16_t gnss_base_rate_2;
    cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_GNSS2_DATA_DESC_SET, // Data descriptor set
        &gnss_base_rate_2        // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get GNSS 2 base rate!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > gnss_base_rate_2)
    {
        command_failure_terminate(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            gnss_base_rate_2
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t gnss_decimation_2 = gnss_base_rate_2 / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO("Decimating GNSS 2 base rate %d by %d to stream data at %dHz.\n",
        gnss_base_rate_2,
        gnss_decimation_2,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate gnss_descriptors_2[1] = {
        { MIP_DATA_DESC_GNSS_FIX_INFO, gnss_decimation_2 }
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for GNSS 2 data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_GNSS2_DATA_DESC_SET,                                    // Data descriptor set
        sizeof(gnss_descriptors_2) / sizeof(gnss_descriptors_2[0]), // Number of descriptors to include
        gnss_descriptors_2                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set message format for GNSS 2 data!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for filter data streaming
///
/// @details Sets up filter data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - GPS time
///             - Filter status
///             - LLH position
///             - NED velocity
///             - Euler angles
///
/// @param _device Pointer to the initialized MIP device interface
///
void configure_filter_message_format(mip_interface* _device)
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
        command_failure_terminate(_device, cmd_result, "Could not get filter base rate!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > filter_base_rate)
    {
        command_failure_terminate(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            filter_base_rate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t filter_decimation = filter_base_rate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO("Decimating filter base rate %d by %d to stream data at %dHz.\n",
        filter_base_rate,
        filter_decimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate filter_descriptors[5] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME, filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS, filter_decimation },
        { MIP_DATA_DESC_FILTER_POS_LLH, filter_decimation },
        { MIP_DATA_DESC_FILTER_VEL_NED, filter_decimation },
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
        command_failure_terminate(_device, cmd_result, "Could not set message format for filter data!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures the GNSS antenna offsets for dual-antenna systems
///
/// @details Sets up the physical antenna offset values relative to the device's
///          reference point for both GNSS antennas. The offsets are specified
///          in meters using 3D vectors:
///          - X: forward/back
///          - Y: left/right
///          - Z: up/down
///
/// @param _device Pointer to the initialized MIP device interface
///
/// @note Offset values are specific to physical device setup and may need to be
///       adjusted based on actual antenna placement. The antenna IDs correspond
///       to:
///       - ID 1: Primary GNSS antenna
///       - ID 2: Secondary GNSS antenna
///
void configure_antennas(mip_interface* _device)
{
    // Configure GNSS 1 antenna offset (in meters)
    const mip_vector3f antenna_offset_1 = {
        -0.25f,
        0.0f,
        0.0f
    };

    MICROSTRAIN_LOG_INFO("Configuring GNSS 1 antenna offset for [%gm, %gm, %gm].\n",
        antenna_offset_1[0],
        antenna_offset_1[1],
        antenna_offset_1[2]
    );
    mip_cmd_result cmd_result = mip_filter_write_multi_antenna_offset(
        _device,
        1, // Receiver/Antenna ID
        antenna_offset_1
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set GNSS 1 antenna offset!\n");
    }

    // Configure GNSS 2 antenna offset (in meters)
    const mip_vector3f antenna_offset_2 = {
        0.25f,
        0.0f,
        0.0f
    };

    MICROSTRAIN_LOG_INFO("Configuring GNSS 2 antenna offset for [%gm, %gm, %gm].\n",
        antenna_offset_2[0],
        antenna_offset_2[1],
        antenna_offset_2[2]
    );
    cmd_result = mip_filter_write_multi_antenna_offset(
        _device,
        2, // Receiver/Antenna ID
        antenna_offset_2
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set GNSS 2 antenna offset!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and resets the navigation filter
///
/// @details Configures the navigation filter by:
///          1. Enabling GNSS position and velocity aiding measurements
///          2. Enabling dual-antenna GNSS heading aiding
///          3. Configuring filter initialization settings:
///             - Setting initial position and velocity to zero
///             - Enabling automatic position/velocity/attitude determination
///             - Configuring dual-antenna kinematic alignment
///          4. Resetting the filter to apply new settings
///
/// @param _device Pointer to the initialized MIP device interface
///
void initialize_filter(mip_interface* _device)
{
    // Configure Filter Aiding Measurements (GNSS position/velocity and dual antenna [aka gnss heading])

    MICROSTRAIN_LOG_INFO("Configuring filter aiding measurement enable for GNSS position and velocity.\n");
    mip_cmd_result cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, // Aiding Source type
        true                                                                     // Enabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(
            _device,
            cmd_result,
            "Could not set filter aiding measurement enable for GNSS position and velocity!\n"
        );
    }

    MICROSTRAIN_LOG_INFO("Configuring filter aiding measurement enable for GNSS heading.\n");
    cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_HEADING, // Aiding Source type
        true                                                                     // Enabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(
            _device,
            cmd_result,
            "Could not set filter aiding measurement enable for GNSS heading!\n"
        );
    }

    // Configure the filter initialization

    const mip_vector3f initial_position = {
        0.0f,
        0.0f,
        0.0f
    };

    const mip_vector3f initial_velocity = {
        0.0f,
        0.0f,
        0.0f
    };

    // Note: This is the default setting on the device and will automatically configure
    // the initial conditions required to initialize the filter based on the alignment selector
    // and readings from the device's sensors
    const mip_filter_initialization_configuration_command_initial_condition_source initial_condition_source =
        MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT;

    const mip_filter_initialization_configuration_command_alignment_selector initial_alignment_selector =
        MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA |
        MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC;

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
        command_failure_terminate(_device, cmd_result, "Could not set the filter initialization configuration!\n");
    }

    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to reset the navigation filter.\n");
    cmd_result = mip_filter_reset(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not reset the navigation filter!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Displays the current GNSS fix state for a specific antenna when
///        changes occur
///
/// @details Outputs readable messages for GNSS fix state transitions:
///          - 3D fix
///          - 2D fix
///          - Time-only fix
///          - No fix
///          - Invalid fix
///          - RTK float
///          - RTK fixed
///          - Differential fix
///          Also indicates whether the current fix is valid based on flag
///          checks.
///
/// @param _fix_info_array Pointer to an array of GNSS fix information
///                        structures containing fix types and validity flags
/// @param _array_index Zero-based index into the fix info array identifying
///                     which GNSS receiver to report (0 = primary antenna,
///                     1 = secondary antenna)
///
void display_gnss_fix_state(const mip_gnss_fix_info_data* _fix_info_array, const uint8_t _array_index)
{
    const uint8_t antenna_id         = _array_index + 1;
    char          header_message[16] = "";
    snprintf(header_message, sizeof(header_message) / sizeof(header_message[0]), "GNSS %d acquired", antenna_id);
    const uint8_t fix_type_value = (uint8_t)_fix_info_array[_array_index].fix_type;

    switch (_fix_info_array[_array_index].fix_type)
    {
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_3D:
        {
            MICROSTRAIN_LOG_INFO("%s a 3D fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_3D\n",
                header_message,
                fix_type_value
            );

            break;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_2D:
        {
            MICROSTRAIN_LOG_INFO("%s a 2D fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_2D\n",
                header_message,
                fix_type_value
            );

            break;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_TIME_ONLY:
        {
            MICROSTRAIN_LOG_INFO("%s a time only fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_TIME_ONLY\n",
                header_message,
                fix_type_value
            );

            break;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_NONE:
        {
            MICROSTRAIN_LOG_INFO("GNSS has no fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_NONE\n",
                fix_type_value
            );

            // No fix, exit early
            return;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_INVALID:
        {
            MICROSTRAIN_LOG_INFO("%s an invalid fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_INVALID\n",
                header_message,
                fix_type_value
            );

            // The fix is already invalid, exit early
            return;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FLOAT:
        {
            MICROSTRAIN_LOG_INFO("%s an RTK float fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FLOAT\n",
                header_message,
                fix_type_value
            );

            break;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FIXED:
        {
            MICROSTRAIN_LOG_INFO("%s an RTK fixed fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FIXED\n",
                header_message,
                fix_type_value
            );

            break;
        }
        case MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_DIFFERENTIAL:
        {
            MICROSTRAIN_LOG_INFO("%s a differential fix. (%d) MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_DIFFERENTIAL\n",
                header_message,
                fix_type_value
            );

            break;
        }
        default:
        {
            break;
        }
    }

    const char* valid_entry_display =
        (_fix_info_array[_array_index].valid_flags & MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_TYPE) != 0 ?
            "valid" :
            "invalid";

    // Confirm a valid fix was acquired
    MICROSTRAIN_LOG_INFO("The current GNSS %d fix is %s.\n", antenna_id, valid_entry_display);
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
mip_timestamp get_current_timestamp()
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
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate)
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
        command_failure_terminate(_device, cmd_result, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmd_result = mip_base_set_idle(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
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
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name", device_info.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number", device_info.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number", device_info.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number", device_info.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options", device_info.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n", "Firmware Version", firmwareVersion);
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
void command_failure_terminate(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...)
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
        // Get the connection pointer that was set during device initialization
        serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

        terminate(device_port, "", false);
    }
}
