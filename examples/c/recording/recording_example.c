////////////////////////////////////////////////////////////////////////////////
/// recording_example.c
///
/// Example program to record factory streaming from any MIP-enabled MicroStrain
/// device using C
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

#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

// Switch between the raw recording interface functions and wrapper functions within the connection interface
// Note: Set to 0 to use the raw recording interface instead
#define CONNECTION_RECORDING_WRAPPERS 1

// Create a user-managed recording stream instead of delegating it to the connection interface
// Note: Set to 0 to show user-managed stream usage
#define USER_RECORDING_STREAMS 0

// TODO: Update with desired recording file names
// Note: Streams may also be used in place of files
static const char* RECEIVED_BYTES_BINARY = "received_bytes.bin";
static const char* SENT_BYTES_BINARY     = "sent_bytes.bin";
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Device callbacks used for reading and writing packets
bool mip_interface_user_send_to_device(const mip_interface* _device, const uint8_t* _data, const size_t _byte_count,
    size_t* _bytes_written_out);
bool mip_interface_user_recv_from_device(const mip_interface* _device, uint8_t* _buffer, const size_t _buffer_size,
    const uint32_t _wait_time, size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out,
    const bool _from_command);

// Utility function to get supported data descriptors for the device
void get_supported_data_descriptors(const uint16_t* _supported_descriptors, const uint8_t _supported_descriptor_count,
    uint8_t* _supported_data_descriptors_out, const uint8_t _supported_data_descriptors_size);

// Common device initialization procedure
void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Stop the application on signal interrupt
void signal_interrupt_handler(const int _signal_value);

// Utility functions the handle application closing and printing error messages
void terminate(serial_port* _device_port, const char* _message, const bool _successful);
void command_failure_terminate(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format,
    ...);

// Running state of the application
volatile sig_atomic_t g_running = 1;

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Initialize the custom logger to print messages/errors as they occur
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);

    // Configure the signal interrupt handler to terminate the application
    if (signal(SIGINT, signal_interrupt_handler) == SIG_ERR)
    {
        terminate(NULL, "Failed to set signal handler!\n", false);
    }

    // Initialize the recording interface
    MICROSTRAIN_LOG_INFO("Initializing the recording interface.\n");
    recording_connection recording_connection;
    recording_connection_init(&recording_connection);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
    serial_port device_port;
    serial_port_init(&device_port, PORT_NAME, BAUDRATE, &recording_connection);

    // Open the recording files for the connection
    MICROSTRAIN_LOG_INFO("Opening connection recording files. Receive: '%s'    Send: '%s'\n",
        RECEIVED_BYTES_BINARY,
        SENT_BYTES_BINARY
    );

#if USER_RECORDING_STREAMS
    // Manual stream creation and management
    FILE* receive_stream = fopen(RECEIVED_BYTES_BINARY, "wb");
    FILE* send_stream    = fopen(SENT_BYTES_BINARY, "wb");

#if CONNECTION_RECORDING_WRAPPERS
    // Note: The connection interfaces offer recording function wrappers for convenience purposes demonstrated here
    // The recording connection needs to be passed to the connection 'init' call before using them
    serial_port_init_recording_streams(&device_port, receive_stream, send_stream);
#else // Raw recording
    // Note: The recording interface can be used instead of the convenience functionality in the connection interfaces
    recording_connection_init_streams(&recording_connection, receive_stream, send_stream);
#endif // CONNECTION_RECORDING_WRAPPERS
#else // Recording manages the streams
#if CONNECTION_RECORDING_WRAPPERS
    // Note: The connection interfaces offer recording function wrappers for convenience purposes demonstrated here
    // The recording connection needs to be passed to the connection 'init' call before using them
    serial_port_open_recording_files(&device_port, RECEIVED_BYTES_BINARY, SENT_BYTES_BINARY);
#else // Raw recording
    // Note: The recording interface can be used instead of the convenience functionality in the connection interfaces
    recording_connection_open_files(&recording_connection, RECEIVED_BYTES_BINARY, SENT_BYTES_BINARY);
#endif // CONNECTION_RECORDING_WRAPPERS
#endif // USER_RECORDING_STREAMS

    MICROSTRAIN_LOG_INFO("Connecting to the device.\n");
    // Open the connection to the device
    if (!serial_port_open(&device_port))
    {
        terminate(&device_port, "Could not open the connection!\n", false);
    }

    mip_interface device;
    initialize_device(&device, &device_port, BAUDRATE);

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    const mip_cmd_result cmd_result = mip_base_resume(&device);
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("Sensor is configured... recording data.\n");
    MICROSTRAIN_LOG_INFO("Press Ctrl+C to stop recording and exit.\n");

    // Previous print time for the recording status message
    microstrain_embedded_timestamp previous_print_time = microstrain_get_current_timestamp();
    uint8_t ellipses_count = 0;

    // Display a recording status
    MICROSTRAIN_LOG_INFO("Recording");

    // Device loop
    // Exit after a signal interrupt
    while (g_running)
    {
        // Update the device state
        // Note: This will update the device callbacks
        mip_interface_update(
            &device,
            0,    // Time to wait
            false // From command
        );

        // Redraw ellipses after the recording message
        if (microstrain_get_current_timestamp() - previous_print_time >= 750)
        {
            previous_print_time = microstrain_get_current_timestamp();

            if (++ellipses_count == 4)
            {
                // Move the cursor before the ellipses and clear it
                printf("\033[3D\033[0K");
            }
            else
            {
                // Print the ellipses
                printf(".");
            }

            // Reset the ellipses
            ellipses_count %= 4;
        }
    }

    // New line for terminating the application
    printf("\n");

#if USER_RECORDING_STREAMS
    // Note: Even if streams are user-managed, recording and connection interfaces can still be used to close the
    // streams using 'recording_connection_close_streams' and '*_close_recording_streams'
    fclose(receive_stream);
    fclose(send_stream);
#endif // USER_RECORDING_STREAMS

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
/// @brief Extracts supported data descriptor sets from a list of device
///        descriptors
///
/// @details Parses a list of device descriptors and extracts only the data
///          descriptor sets (i.e., excludes field descriptors). The function
///          filters for unique data descriptors and stores them in the provided
///          output array. Processing stops when a zero descriptor is
///          encountered or when the output array reaches capacity.
///
/// @param _supported_descriptors Array of supported composite descriptors from
///                               the device
/// @param _supported_descriptor_count Number of descriptors in the input array
/// @param _supported_data_descriptors_out Output array for unique data
///                                        descriptors
/// @param _supported_data_descriptors_size Maximum capacity of the output array
///
void get_supported_data_descriptors(const uint16_t* _supported_descriptors, const uint8_t _supported_descriptor_count,
    uint8_t* _supported_data_descriptors_out, const uint8_t _supported_data_descriptors_size)
{
    uint8_t last_index = 0;

    for (uint16_t descriptor_index = 0; descriptor_index < _supported_descriptor_count; ++descriptor_index)
    {
        // Extract the descriptor set from the composite descriptor
        const uint8_t supported_descriptor_set = (_supported_descriptors[descriptor_index] & 0xFF00) >> 8;

        // Initialized the array with 0 can break on a 0 descriptor
        if (supported_descriptor_set == 0)
        {
            break;
        }

        // Store supported data descriptors
        if (mip_is_data_descriptor_set(supported_descriptor_set))
        {
            // Only add unique descriptors
            if (last_index == 0 || _supported_data_descriptors_out[last_index - 1] != supported_descriptor_set)
            {
                _supported_data_descriptors_out[last_index] = supported_descriptor_set;

                ++last_index;

                assert(last_index <= _supported_data_descriptors_size);

                // Cannot add more descriptors (increase array size)
                if (last_index == _supported_data_descriptors_size)
                {
                    break;
                }
            }
        }
    }
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
///          6. Enables factory data streaming
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

    // Get supported descriptors to check if certain data descriptors are supported for streaming
    MICROSTRAIN_LOG_INFO("Getting supported descriptors for the device.\n");
    uint8_t  descriptors_count          = 0;
    uint16_t supported_descriptors[256] = { 0 };
    cmd_result = mip_base_get_device_descriptors(
        _device,
        supported_descriptors,                                            // Descriptors array from the device
        sizeof(supported_descriptors) / sizeof(supported_descriptors[0]), // Max array size
        &descriptors_count                                                // Descriptor count returned from the device
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get supported descriptors!\n");
    }

    // Some devices have a large number of descriptors
    // The extended descriptors command can get the remaining descriptors
    MICROSTRAIN_LOG_INFO("Getting extended supported descriptors for the device.\n");
    uint8_t extended_descriptors_count = 0;
    cmd_result = mip_base_get_extended_descriptors(
        _device,
        &supported_descriptors[descriptors_count],                                            // Append to the existing array
        sizeof(supported_descriptors) / sizeof(supported_descriptors[0]) - descriptors_count, // Remaining size of the array
        &extended_descriptors_count
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not get extended supported descriptors!\n");
    }

    uint8_t supported_data_descriptors[16] = { 0 };
    const uint8_t supported_data_descriptor_size =
        sizeof(supported_data_descriptors) / sizeof(supported_data_descriptors[0]);

    get_supported_data_descriptors(supported_descriptors, descriptors_count + extended_descriptors_count,
        supported_data_descriptors, supported_data_descriptor_size);

    MICROSTRAIN_LOG_INFO("Resetting factory support data streaming.\n");
    cmd_result = mip_3dm_factory_streaming(_device, MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_OVERWRITE, 0);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        command_failure_terminate(_device, cmd_result, "Could not reset factory support data streaming!\n");
    }

    // Enable streaming for supported data descriptors
    for (uint8_t data_descriptor_index = 0; data_descriptor_index < supported_data_descriptor_size;
        ++data_descriptor_index)
    {
        // Any 0 value was from initialization
        if (supported_data_descriptors[data_descriptor_index] == 0)
        {
            break;
        }

        cmd_result = mip_3dm_write_datastream_control(_device, supported_data_descriptors[data_descriptor_index], true);

        if (!mip_cmd_result_is_ack(cmd_result))
        {
            command_failure_terminate(
                _device,
                cmd_result,
                "Could not enable streaming for data descriptor 0x%02X!\n",
                supported_data_descriptors[data_descriptor_index]
            );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Signal interrupt callback handler
///
/// @details Handles the signal interrupt callback and sets the application
///          running flag to false (0)
///
/// @param _signal_value Value of the signal (unused in this implementation)
///
void signal_interrupt_handler(const int _signal_value)
{
    // Unused parameter
    (void)_signal_value;

    // Stop the application
    g_running = 0;
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

#if !USER_RECORDING_STREAMS
#if CONNECTION_RECORDING_WRAPPERS
        if (serial_port_recording_enabled(_device_port))
        {
            MICROSTRAIN_LOG_INFO("Closing the recording streams.\n");

            // Note: This can also be used for user-defined streams
            serial_port_close_recording_streams(_device_port);
        }
#else // Raw recording
        if (recording_connection_enabled(_device_port->recording_connection))
        {
            MICROSTRAIN_LOG_INFO("Closing the recording streams.\n");

            // Note: This can also be used for user-defined streams
            recording_connection_close_streams(_device_port->recording_connection);
        }
#endif // CONNECTION_RECORDING_WRAPPERS
#endif // !USER_RECORDING_STREAMS
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
