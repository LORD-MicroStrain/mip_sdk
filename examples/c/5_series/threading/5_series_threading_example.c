////////////////////////////////////////////////////////////////////////////////
/// 5_series_threading_example.c
///
/// Example program to demonstrate multithreading for data collection and
/// command updates on 5-series devices using C
///
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

// Include the MicroStrain timestamping header
#include <microstrain/embedded_time.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
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

#ifdef __APPLE__
// Clang doesn't support threads.h
// Adding basic functionality from threads.h for Clang support
#include <pthread.h>

typedef pthread_t thrd_t;
typedef int (thrd_start_t)(void*);

enum { thrd_success, thrd_busy, thrd_error, thrd_nomem, thrd_timedout };

int thrd_create(thrd_t* __thr, void* __func, void* __arg);
int thrd_join(thrd_t __thr, int* __res);
int thrd_sleep(const struct timespec* __time_point, struct timespec* __remaining);
void thrd_yield(void);
#else
#include <threads.h>
#endif // __APPLE__

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
// Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static const char* PORT_NAME = "COM1";
#else // Unix
static const char* PORT_NAME = "/dev/ttyACM0";
#endif // _WIN32

// Set the baudrate for the connection (Serial/USB)
static const uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
// Streaming rate in Hz
static const uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
// Example run time
static const uint32_t RUN_TIME_SECONDS = 30;

// TODO: Enable/disable data collection threading
// Use this to test the behaviors of threading
#define USE_THREADS 1
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Device callbacks used for reading and writing packets
bool mip_interface_user_send_to_device(const mip_interface* _device, const uint8_t* _data, const size_t _byte_count,
    size_t* _bytes_written_out);
bool mip_interface_user_recv_from_device(const mip_interface* _device, uint8_t* _buffer, const size_t _buffer_size,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command);

// Common device initialization procedure
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Message format configuration
void configure_sensor_message_format(mip_interface* _device);

// Packet callback handler
void packet_callback(void* _user, const mip_packet_view* _packet_view, const mip_timestamp _timestamp);

#if USE_THREADS
// Basic structure for thread data
typedef struct thread_data
{
    mip_interface* device;
    volatile bool  running;
} thread_data_t;

// Threaded functions
bool update_device(mip_interface* _device, const mip_timeout _wait_time, const bool _from_command);
int data_collection_thread(void* _thread_data);
#endif // USE_THREADS

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
    MICROSTRAIN_LOG_INFO("Initializing the connection on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
    serial_port device_port;
    serial_port_init(&device_port, PORT_NAME, BAUDRATE, NULL);

    MICROSTRAIN_LOG_INFO("Connecting to the device.\n");

    // Open the connection to the device
    if (!serial_port_open(&device_port))
    {
        terminate(&device_port, "Could not open the connection!\n", false);
    }

    mip_interface device;
    initialize_device(&device, &device_port, BAUDRATE);

    // Configure the message format for sensor data
    configure_sensor_message_format(&device);

    // Sensor data packet callback
    MICROSTRAIN_LOG_INFO("Registering a sensor data packet callback.\n");

    mip_dispatch_handler packet_handler;

    // Register the callback for packets
    mip_interface_register_packet_callback(
        &device,
        &packet_handler,
        MIP_SENSOR_DATA_DESC_SET, // Data field descriptor set
        false,                    // Process after field callback
        &packet_callback,         // Callback
        NULL                      // User data
    );

#if USE_THREADS
    MICROSTRAIN_LOG_INFO("Initializing the device update function for threading.\n");
    // Note: This allows the update function to be split into command and data updates across multiple threads
    mip_interface_set_update_function(&device, &update_device);

    thread_data_t data = {
        .device  = &device,
        .running = true
    };

    MICROSTRAIN_LOG_INFO("Creating the data collection thread.\n");
    thrd_t data_thread;
    thrd_create(&data_thread, data_collection_thread, (void*)&data);
#endif // USE_THREADS

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    const mip_cmd_result cmd_result = mip_base_resume(&device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... waiting for data.\n");

    // Get the start time of the device update loop to handle exiting the application
    const mip_timestamp loop_start_time = microstrain_get_current_timestamp();

    // Running loop
    // Exit after a predetermined time in seconds
    while (microstrain_get_current_timestamp() - loop_start_time <= RUN_TIME_SECONDS * 1000)
    {
        // Stress testing the device with ping
        // This attempts to trigger race conditions across threads
        // Note: Only one thread at a time can safely send commands
        MICROSTRAIN_LOG_WARN("Running device stress test!\n");
        for (uint8_t counter = 0; counter < 100; ++counter)
        {
            // Note: Sending commands calls the device update function every time
            mip_base_ping(&device);
        }
    }

#if USE_THREADS
    // Signal the data collection thread to stop
    data.running = false;

    // Join the thread back before exiting the program
    MICROSTRAIN_LOG_INFO("Waiting for the thread to join.\n");
    int thread_return_code; // Return code from the thread function (Unused)
    if (thrd_join(data_thread, &thread_return_code) != thrd_success)
    {
        terminate(&device_port, "Failed to join the thread!\n", false);
    }
#endif // USE_THREADS

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
/// @brief Handles sending packets to the device
///
/// @details Implements the MIP device interface send callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Writes data buffer to serial port
///
/// @param _device MIP device interface containing the connection
/// @param _data Buffer containing packet data to send
/// @param _byte_count Number of bytes to send
/// @param _bytes_written_out Number of actual bytes written
///
/// @return True if send was successful, false otherwise
///
bool mip_interface_user_send_to_device(const mip_interface* _device, const uint8_t* _data, const size_t _byte_count,
    size_t* _bytes_written_out)
{
    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_connection_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the bytes written to the device
    size_t bytes_written;

    // Send the packet to the device
    if (!serial_port_write(device_port, _data, _byte_count, &bytes_written))
    {
        return false;
    }

    if (_bytes_written_out != NULL)
    {
        *_bytes_written_out = bytes_written;
    }

    return true;
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
/// @param _buffer_size Maximum number of bytes to read
/// @param _wait_time How long to wait for data in milliseconds
/// @param _bytes_read_out Number of bytes actually read
/// @param _timestamp_out Timestamp when data was received
/// @param _from_command Whether this read is from a command response (unused)
///
/// @return True if receive was successful, false otherwise
///
bool mip_interface_user_recv_from_device(const mip_interface* _device, uint8_t* _buffer, const size_t _buffer_size,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command)
{
    // Unused parameter
    (void)_from_command;

    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_connection_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the time that the packet was received (system epoch UTC time in milliseconds)
    *_timestamp_out = microstrain_get_current_timestamp();

    // Read the packet from the device
    return serial_port_read(device_port, _buffer, _buffer_size, _wait_time, _bytes_read_out, _timestamp_out);
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
        (void*)_device_port,                  // Connection pointer
        NULL                                  // Optional user data
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
/// @brief Configures message format for sensor data streaming
///
/// @details Sets up sensor data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - Scaled accelerometer
///
/// @param _device Pointer to the initialized MIP device interface
void configure_sensor_message_format(mip_interface* _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t sensor_base_rate;
    mip_cmd_result cmd_result = mip_3dm_imu_get_base_rate(
        _device,
        &sensor_base_rate // Base rate out
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
    const mip_descriptor_rate sensor_descriptors[1] = {
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for sensor data.\n");
    cmd_result = mip_3dm_write_imu_message_format(
        _device,
        sizeof(sensor_descriptors) / sizeof(sensor_descriptors[0]), // Number of descriptors to include
        sensor_descriptors                                          // Descriptor array
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
void packet_callback(void* _user, const mip_packet_view* _packet_view, const mip_timestamp _timestamp)
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

#if USE_THREADS
////////////////////////////////////////////////////////////////////////////////
/// @brief Updates the device state based on command or data collection context
///
/// @details Handles device updates differently depending on whether called from
///          command handling or data collection:
///          - For data collection (_from_cmd = false): Performs normal device
///            updates
///          - For commands (_from_cmd = true): Sleeps briefly to save power
///            while avoiding command timeouts
///
/// @param _device Pointer to the MIP device interface
/// @param _wait_time Time to wait for updates (typically used only from
///                   commands)
/// @param _from_command True if called from command handling, false for data
///                      collection
///
/// @returns true if the update was successful, false on error.
///          Always returns true when called from commands to avoid race
///          conditions.
///
bool update_device(mip_interface* _device, const mip_timeout _wait_time, const bool _from_command)
{
    // Do normal updates only if not called from a command handler
    // Note: This is the separation between the main/other thread and the data collection thread
    if (!_from_command)
    {
        return mip_interface_default_update(_device, _wait_time, _from_command);
    }

    // Create a 5-millisecond timeout
    const struct timespec ts = {
        .tv_sec  = 0,           // 0 Seconds
        .tv_nsec = 5 * 1000000  // 5 Milliseconds
    };

    // Sleep for a bit to save power
    // Note: Waiting too long in here will cause commands to timeout
    thrd_sleep(&ts, NULL);

    // Note: This needs to return true to avoid terminating the data collection thread
    // Note: Returning false may cause a race condition (see comments in mip_interface_wait_for_reply)
    return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles continuous data collection from the device in a separate
///        thread
///
/// @details Main function for the data collection thread that:
///          1. Continuously updates device state for receiving data
///          2. Clears command queue if the connection closes to avoid
///             deadlocks
///          3. Yields thread time when possible
///          4. Runs until the running state becomes false
///
/// @param _thread_data Pointer to a thread data structure containing:
///                     - device: MIP device interface
///                     - running: boolean controlling thread execution
///
/// @returns 0 (return value unused)
///
int data_collection_thread(void* _thread_data)
{
    MICROSTRAIN_LOG_INFO("Data collection thread created!\n");

    const thread_data_t* thread_data = (thread_data_t*)_thread_data;

    while (thread_data->running)
    {
        // Update the device for data collection
        if (!mip_interface_update(
            thread_data->device,
            0,      // Wait time for the update (Typically only used from commands)
            false)) // From a command (Note: This update is for data handling, not from a command)
        {
            // Avoid deadlocks if the connection is closed
            mip_cmd_queue* cmd_queue = mip_interface_cmd_queue(thread_data->device);
            assert(cmd_queue);
            mip_cmd_queue_clear(cmd_queue);

            break;
        }

        thrd_yield();
    }

    // Return value unused
    return 0;
}
#endif // USE_THREADS

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
        serial_port* device_port = (serial_port*)mip_interface_connection_pointer(_device);

        terminate(device_port, "", false);
    }
}

#ifdef __APPLE__
// pthread wrappers for unsupported threads.h functionality used in this example

// pthread wrapper for threads.h thrd_create
int thrd_create(thrd_t* __thr, void* __func, void* __arg)
{
    return pthread_create(__thr, NULL, __func, __arg);
}

// pthread wrapper for threads.h thrd_join
int thrd_join(thrd_t __thr, int* __res)
{
    return pthread_join(__thr, (void**)__res);
}

// sleep wrapper for threads.h thrd_sleep
int thrd_sleep(const struct timespec* __time_point, struct timespec* __remaining)
{
    // Sleep for some duration
    return nanosleep(__time_point, __remaining);
}

// pthread wrapper for threads.h thrd_yield
void thrd_yield()
{
    sched_yield();
}
#endif // __APPLE__
