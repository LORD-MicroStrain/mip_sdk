////////////////////////////////////////////////////////////////////////////////
/// device_info_example.c
///
/// Example program to print device information from any MIP-enabled MicroStrain
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

// Include all necessary MIP headers
#include <mip/mip_interface.h>
#include <mip/definitions/commands_base.h>
#include <mip/mip_metadata.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>


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

volatile sig_atomic_t g_stop = false;

void signal_stop(int sig)
{
    (void)sig;

    g_stop = true;
}

void packet_callback(void* p, const mip_packet_view* packet, mip_timestamp ts)
{
    (void)p;
    (void)ts;

    mip_pretty_print_packet(packet, MICROSTRAIN_LOG_LEVEL_INFO);
}

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
        MICROSTRAIN_LOG_FATAL("Could not open the connection!\n");
        return 1;
    }

    mip_interface device;

    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip_interface_init(
        &device,
        mip_timeout_from_baudrate(BAUDRATE),  // Set the base timeout for commands (milliseconds)
        2000,                                 // Set the base timeout for command replies (milliseconds)
        &mip_interface_user_send_to_device,   // User-defined send packet callback
        &mip_interface_user_recv_from_device, // User-defined receive packet callback
        &mip_interface_default_update,        // Default update callback
        &device_port                          // Cast the device port for use in the callbacks
    );

    mip_dispatch_handler handler;
    mip_interface_register_packet_callback(&device, &handler, MIP_DISPATCH_ANY_DATA_SET, false, &packet_callback, NULL);

    // Setup a signal handler to catch ctrl+C.
    signal(SIGTERM, &signal_stop);

    while(!g_stop)
    {
        if(!mip_interface_update(&device, 100, false))
        {
            MICROSTRAIN_LOG_FATAL("Failed to read from device!\n");
            return 1;
        }
    }

    MICROSTRAIN_LOG_INFO("Received Ctrl+C, stopping.\n");

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
