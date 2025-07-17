#pragma once

#include <mip/definitions/data_filter.h>

#include <microstrain/logging.h>
#include <microstrain/connections/serial/serial_port.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

serial_port device_port;
time_t startTime;

void mip_example_log(void* user, const microstrain_log_level level, const char* fmt, va_list args)
{
    (void)user;

    switch (level)
    {
        case MICROSTRAIN_LOG_LEVEL_FATAL:
        case MICROSTRAIN_LOG_LEVEL_ERROR:
        {
            vfprintf(stderr, fmt, args);
            break;
        }
        default:
        {
            vprintf(fmt, args);
            break;
        }
    }
}

void mip_example_init()
{
    serial_port_init(&device_port);

    // Record program start time for use with difftime in getTimestamp().
    time(&startTime);

    // Initialize the MIP logger before opening the port so we can print errors if they occur
    MICROSTRAIN_LOG_INIT(&mip_example_log, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
}

mip_timestamp get_current_timestamp()
{
    time_t t;
    time(&t);

    double delta = difftime(t, startTime);

    return (mip_timestamp)(delta * 1000);
}

bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, bool from_cmd, size_t* length_out, mip_timestamp* timestamp_out)
{
    (void)device;
    (void)from_cmd;

    *timestamp_out = get_current_timestamp();

    return serial_port_read(&device_port, buffer, max_length, (int)wait_time, length_out);
}

bool mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length)
{
    (void)device;

    size_t bytes_written;
    return serial_port_write(&device_port, data, length, &bytes_written);
}

void exit_gracefully(const char* message)
{
    if (message)
    {
        printf("%s\n", message);
    }

    // Close com port
    if (serial_port_is_open(&device_port))
    {
        serial_port_close(&device_port);
    }

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    system("pause");
#endif

    exit(0);
}

// Displays current filter state for the connected device if it has changed.
void display_filter_state(const mip_filter_mode filter_state, char** current_state, bool is_five_series)
{
    char* read_state = "";

    switch (filter_state)
    {
        case MIP_FILTER_MODE_INIT:
        {
            read_state = is_five_series ? "GX5_INIT (1)" : "INIT (1)";
            break;
        }
        case MIP_FILTER_MODE_VERT_GYRO:
        {
            read_state = is_five_series ? "GX5_RUN_SOLUTION_VALID (2)" : "VERT_GYRO (2)";
            break;
        }
        case MIP_FILTER_MODE_AHRS:
        {
            read_state = is_five_series ? "GX5_RUN_SOLUTION_ERROR (3)" : "AHRS (3)";
            break;
        }
        case MIP_FILTER_MODE_FULL_NAV:
        {
            read_state = "FULL_NAV (4)";
            break;
        }
        default:
        {
            read_state = "STARTUP (0)";
            break;
        }
    }

    if (strcmp(read_state, *current_state) != 0)
    {
        printf("FILTER STATE: %s\n", read_state);
        *current_state = read_state;
    }
}
