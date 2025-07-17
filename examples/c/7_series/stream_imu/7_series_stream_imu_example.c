////////////////////////////////////////////////////////////////////////////////
/// 7_series_stream_imu_example.c
///
/// Example setup program for streaming IMU data on 7-series devices using C
///
/// This example shows a basic setup for streaming IMU data on 7-series devices
/// using C.
/// It is not an exhaustive example of all streaming options.
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
#include <mip/mip_dispatch.h>
#include <mip/mip_interface.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/data_sensor.h>

#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
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

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
mip_timestamp get_current_timestamp();

// Device callbacks used for reading and writing packets
bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length);
bool mip_interface_user_recv_from_device(mip_interface* _device, uint8_t* _buffer, size_t _max_length,
    mip_timeout _wait_time, bool _from_cmd, size_t* _length_out, mip_timestamp* _timestamp_out);

// Common device initialization procedure
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Utility to help check if the device supports a descriptor
bool is_descriptor_supported(const uint8_t _descriptor_set, const uint8_t _field_descriptor,
    const uint16_t* _supported_descriptors, const uint8_t _supported_descriptor_count);

// Message format configuration
void configure_sensor_message_format(mip_interface* _device, const uint16_t* _supported_descriptors,
    const uint8_t _supported_descriptor_count);

// Callback handlers
void packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp);
void accel_field_callback(void* _user, const mip_field_view* _field_view, mip_timestamp _timestamp);
void gyro_field_callback(void* _user, const mip_field_view* _field_view, mip_timestamp _timestamp);
void mag_field_callback(void* _user, const mip_field_view* _field_view, mip_timestamp _timestamp);

// Utility functions the handle application closing and printing error messages
void terminate(serial_port* _device_port, const char* _message, const bool _successful);
void command_failure_terminate(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...);

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

    // Get supported descriptors to check if certain data descriptors are supported for streaming
    MICROSTRAIN_LOG_INFO("Getting supported descriptors for the device.\n");
    uint8_t  descriptors_count          = 0;
    uint16_t supported_descriptors[256] = { 0 };
    mip_cmd_result cmd_result = mip_base_get_device_descriptors(
        &device,
        supported_descriptors,                                            // Descriptors array from the device
        sizeof(supported_descriptors) / sizeof(supported_descriptors[0]), // Max array size
        &descriptors_count                                                // Descriptor count returned from the device
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not get supported descriptors!\n");
    }

    // Some devices have a large number of descriptors
    // The extended descriptors command can get the remaining descriptors
    MICROSTRAIN_LOG_INFO("Getting extended supported descriptors for the device.\n");
    uint8_t extended_descriptors_count = 0;
    cmd_result = mip_base_get_extended_descriptors(
        &device,
        &supported_descriptors[descriptors_count],                                            // Append to the existing array
        sizeof(supported_descriptors) / sizeof(supported_descriptors[0]) - descriptors_count, // Remaining size of the array
        &extended_descriptors_count
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not get extended supported descriptors!\n");
    }

    // Configure the message format for sensor data
    configure_sensor_message_format(&device, supported_descriptors, descriptors_count + extended_descriptors_count);

    // Register packet and data callbacks

    // Generic packet callback
    MICROSTRAIN_LOG_INFO("Registering a generic packet callback.\n");

    mip_dispatch_handler packet_handler;

    // Register the callback for packets
    mip_interface_register_packet_callback(
        &device,
        &packet_handler,
        MIP_DISPATCH_ANY_DATA_SET, // Data field descriptor set
        false,                     // Process after field callback
        &packet_callback,          // Callback
        NULL                       // User data
    );

    // Sensor data callbacks
    MICROSTRAIN_LOG_INFO("Registering sensor data callbacks.\n");

    mip_dispatch_handler sensor_data_handlers[3];

    // Register the callbacks for the sensor fields

    mip_interface_register_field_callback(
        &device,
        &sensor_data_handlers[0],
        MIP_SENSOR_DATA_DESC_SET,          // Data descriptor set
        MIP_DATA_DESC_SENSOR_ACCEL_SCALED, // Data field descriptor set
        accel_field_callback,              // Callback
        NULL                               // User data
    );

    mip_interface_register_field_callback(
        &device,
        &sensor_data_handlers[1],
        MIP_SENSOR_DATA_DESC_SET,         // Data descriptor set
        MIP_DATA_DESC_SENSOR_GYRO_SCALED, // Data field descriptor set
        gyro_field_callback,              // Callback
        NULL                              // User data
    );

    // Note: Even if the device doesn't support mag data, registering this won't break anything
    // The callback will just never be called
    mip_interface_register_field_callback(
        &device,
        &sensor_data_handlers[2],
        MIP_SENSOR_DATA_DESC_SET,        // Data descriptor set
        MIP_DATA_DESC_SENSOR_MAG_SCALED, // Data field descriptor set
        mag_field_callback,              // Callback
        NULL                             // User data
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmd_result = mip_base_resume(&device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... waiting for data.\n");

    // Get the start time of the device update loop to handle exiting the application
    const mip_timestamp loop_start_time = get_current_timestamp();

    // Device loop
    // Exit after predetermined time in seconds
    while (get_current_timestamp() - loop_start_time <= RUN_TIME_SECONDS * 1000)
    {
        // Update the device state
        // Note: This will update the device callbacks
        mip_interface_update(
            &device,
            0,    // Time to wait
            false // From command
        );
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

////////////////////////////////////////////////////////////////////////////////
/// @brief Determines if the device supports a specific descriptor
///
/// @details Checks if a given descriptor set and field descriptor combination
///          is supported by comparing against an array of supported
///          descriptors. The device returns descriptors as composite values
///          (e.g., 0x8001 from 0x80,0x01).
///
/// @param _descriptor_set The high byte of the composite descriptor
/// @param _field_descriptor The low byte of the composite descriptor
/// @param _supported_descriptors Array of supported composite descriptors
/// @param _supported_descriptor_count Number of entries in the array
///
/// @returns true if the descriptor combination is supported, false otherwise
///
bool is_descriptor_supported(const uint8_t _descriptor_set, const uint8_t _field_descriptor,
    const uint16_t* _supported_descriptors, const uint8_t _supported_descriptor_count)
{
    // Combine the 2 descriptors for proper comparison
    const uint16_t composite_descriptor = (uint16_t)_descriptor_set << 8 | (uint16_t)_field_descriptor;

    for (uint8_t index = 0; index < _supported_descriptor_count; ++index)
    {
        if (_supported_descriptors[index] == composite_descriptor)
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
/// @param _device Pointer to the initialized MIP device interface
/// @param _supported_descriptors Array of descriptors supported by the device
/// @param _supported_descriptor_count Number of descriptors in the array
///
void configure_sensor_message_format(mip_interface* _device, const uint16_t* _supported_descriptors,
    const uint8_t _supported_descriptor_count)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t sensor_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_SENSOR_DATA_DESC_SET, // Data descriptor set
        &sensor_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get sensor base rate!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > sensor_base_rate)
    {
        command_failure_terminate(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            sensor_base_rate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t sensor_decimation  = sensor_base_rate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO("Decimating sensor base rate %d by %d to stream data at %dHz.\n",
        sensor_base_rate,
        sensor_decimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate sensor_descriptors[3] = {
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,  sensor_decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,   sensor_decimation }
    };

    // Assume all descriptors are supported until the check below
    uint8_t sensor_descriptor_count = sizeof(sensor_descriptors) / sizeof(sensor_descriptors[0]);

    MICROSTRAIN_LOG_INFO("Checking if the device supports magnetometer data.\n");

    // Not all devices have a magnetometer
    if (is_descriptor_supported(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED, _supported_descriptors,
        _supported_descriptor_count))
    {
        MICROSTRAIN_LOG_INFO("The device supports magnetometer data.\n");
    }
    else
    {
        MICROSTRAIN_LOG_INFO("The device does not support magnetometer data.\n");

        // Don't include the magnetometer data descriptor in the array for message format configuration
        --sensor_descriptor_count;
    }

    MICROSTRAIN_LOG_INFO("Configuring message format for sensor data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_SENSOR_DATA_DESC_SET, // Data descriptor set
        sensor_descriptor_count,  // Number of descriptors to include
        sensor_descriptors        // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not set message format for sensor data!\n");
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
/// @param _packet_view Pointer to the received MIP packet
/// @param _timestamp Timestamp when the packet was received
///
void packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp)
{
    // Unused parameter
    (void)_user;

    // Create a buffer for printing purposes
    char field_descriptors_buffer[255] = { 0 };
    int  buffer_offset                 = 0;

    // Field object for iterating the packet and extracting each field
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    // Iterate the packet and extract each field
    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        buffer_offset += snprintf(
            &field_descriptors_buffer[buffer_offset],
            sizeof(field_descriptors_buffer) / sizeof(field_descriptors_buffer[0]) - buffer_offset,
            " 0x%02X,",
            mip_field_field_descriptor(&field_view)
        );
    }

    // Trim off the last comma
    if (buffer_offset > 0)
    {
        field_descriptors_buffer[buffer_offset - 1] = '\0';
    }

    MICROSTRAIN_LOG_INFO("Received a packet at %" PRIu64 " with descriptor set 0x%02X:%s\n",
        _timestamp,
        mip_packet_descriptor_set(_packet_view),
        field_descriptors_buffer
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
/// @param _field_view Pointer to the field containing accelerometer data
/// @param _timestamp Timestamp indicating when the field was received from the
///                   device (unused in this implementation)
///
void accel_field_callback(void* _user, const mip_field_view* _field_view, mip_timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip_sensor_scaled_accel_data scaled_accel_data;

    if (extract_mip_sensor_scaled_accel_data_from_field(_field_view, &scaled_accel_data))
    {
        MICROSTRAIN_LOG_INFO("%-17s (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Accel Data",
            MIP_SENSOR_DATA_DESC_SET,          // Data descriptor set
            MIP_DATA_DESC_SENSOR_ACCEL_SCALED, // Data field descriptor set
            scaled_accel_data.scaled_accel[0], // X
            scaled_accel_data.scaled_accel[1], // Y
            scaled_accel_data.scaled_accel[2]  // Z
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
/// @param _field_view Pointer to the field containing gyroscope data
/// @param _timestamp Timestamp indicating when the field was received from the
///                   device (unused in this implementation)
///
void gyro_field_callback(void* _user, const mip_field_view* _field_view, mip_timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip_sensor_scaled_gyro_data scaled_gyro_data;

    if (extract_mip_sensor_scaled_gyro_data_from_field(_field_view, &scaled_gyro_data))
    {
        MICROSTRAIN_LOG_INFO("%-17s (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Gyro Data",
            MIP_SENSOR_DATA_DESC_SET,         // Data descriptor set
            MIP_DATA_DESC_SENSOR_GYRO_SCALED, // Data field descriptor set
            scaled_gyro_data.scaled_gyro[0],  // X
            scaled_gyro_data.scaled_gyro[1],  // Y
            scaled_gyro_data.scaled_gyro[2]   // Z
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
/// @param _field_view Pointer to the field containing magnetometer data
/// @param _timestamp Timestamp indicating when the field was received from the
///                   device (unused in this implementation)
///
void mag_field_callback(void* _user, const mip_field_view* _field_view, mip_timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    mip_sensor_scaled_mag_data scaled_mag_data;

    if (extract_mip_sensor_scaled_mag_data_from_field(_field_view, &scaled_mag_data))
    {
        MICROSTRAIN_LOG_INFO("%-17s (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Mag Data",
            MIP_SENSOR_DATA_DESC_SET,        // Data descriptor set
            MIP_DATA_DESC_SENSOR_MAG_SCALED, // Data field descriptor set
            scaled_mag_data.scaled_mag[0],   // X
            scaled_mag_data.scaled_mag[1],   // Y
            scaled_mag_data.scaled_mag[2]    // Z
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
