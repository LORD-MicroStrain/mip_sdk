#pragma once

#include <microstrain/platform.h>

#if defined MICROSTRAIN_PLATFORM_WINDOWS

#if !defined WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // !WIN32_LEAN_AND_MEAN

#include <windows.h>
#else // Unix
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif // MICROSTRAIN_PLATFORM_WINDOWS

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain_platform  Platform specific utilities
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@defgroup microstrain_serial  Serial Port
///
///@brief A simple implementation for reading and writing to/from a serial port.
///
///@{

typedef struct serial_port
{
#if defined MICROSTRAIN_PLATFORM_WINDOWS
    HANDLE handle;
#else // Unix
    int handle;
#endif // MICROSTRAIN_PLATFORM_WINDOWS
} serial_port;

void serial_port_init(serial_port* port);
bool serial_port_open(serial_port* port, const char* port_str, int baudrate);
bool serial_port_set_baudrate(serial_port* port, int baudrate);
bool serial_port_close(serial_port* port);
bool serial_port_write(serial_port* port, const void* buffer, size_t num_bytes, size_t* bytes_written);
bool serial_port_read(serial_port* port, void* buffer, size_t num_bytes, int wait_time, size_t* bytes_read);
uint32_t serial_port_read_count(serial_port* port);
bool serial_port_is_open(const serial_port* port);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif // __cplusplus
