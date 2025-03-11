#include "example_utils.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>


serial_port device_port;
time_t startTime;


void mip_example_init()
{
    serial_port_init(&device_port);

    // Record program start time for use with difftime in getTimestamp().
    time(&startTime);

    // Initialize the MIP logger before opening the port so we can print errors if they occur
    MICROSTRAIN_LOG_INIT(&mip_example_log, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
}


void mip_example_log(void* user, const microstrain_log_level level, const char* fmt, va_list args)
{
    (void)user;

    switch (level)
    {
    case MICROSTRAIN_LOG_LEVEL_FATAL:
    case MICROSTRAIN_LOG_LEVEL_ERROR:
        vfprintf(stderr, fmt, args);
        break;
    default:
        vprintf(fmt, args);
        break;
    }
}


mip_timestamp get_current_timestamp()
{
    time_t t;
    time(&t);

    double delta = difftime(t, startTime);

    return (mip_timestamp)(delta * 1000);
}


bool mip_interface_user_recv_from_device(mip_interface* device_, uint8_t* buffer, size_t max_length, mip_timeout wait_time, bool from_cmd, size_t* length_out, mip_timestamp* timestamp_out)
{
    (void)device_;
    (void)from_cmd;

    *timestamp_out = get_current_timestamp();

    return serial_port_read(&device_port, buffer, max_length, (int)wait_time, length_out);
}


bool mip_interface_user_send_to_device(mip_interface* device_, const uint8_t* data, size_t length)
{
    (void)device_;

    size_t bytes_written;
    if (!serial_port_write(&device_port, data, length, &bytes_written))
        return false;

    return true;
}


void exit_gracefully(const char *message)
{
    if(message)
        printf("%s\n", message);

    // Close com port
    if(serial_port_is_open(&device_port))
        serial_port_close(&device_port);

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    getchar();
#endif

    exit(0);
}


void displayFilterState(const mip_filter_mode filter_state, char **current_state, bool isFiveSeries)
{
    char *read_state = "";
    if (filter_state == MIP_FILTER_MODE_INIT)
        read_state = isFiveSeries ? "GX5_INIT (1)" : "INIT (1)";
    else if (filter_state == MIP_FILTER_MODE_VERT_GYRO)
        read_state = isFiveSeries ? "GX5_RUN_SOLUTION_VALID (2)" : "VERT_GYRO (2)";
    else if (filter_state == MIP_FILTER_MODE_AHRS)
        read_state = isFiveSeries ? "GX5_RUN_SOLUTION_ERROR (3)" : "AHRS (3)";
    else if (filter_state == MIP_FILTER_MODE_FULL_NAV)
        read_state = "FULL_NAV (4)";
    else
        read_state = "STARTUP (0)";

    if (strcmp(read_state, *current_state) != 0)
    {
        printf("FILTER STATE: %s\n", read_state);
        *current_state = read_state;
    }
}
