#pragma once

#ifdef WIN32
#include <windows.h>
#else
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_extras  Extra utilities
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_serial  Serial Port
///
///@brief A simple implementation for reading and writing to/from a serial port.
///
///@{

typedef struct serial_port
{
    bool is_open;
#ifdef WIN32 //Windows
    HANDLE handle;
#else //Linux
    int handle;
#endif
} serial_port;


bool serial_port_open(serial_port *port, const char *port_str, int baudrate);
bool serial_port_close(serial_port *port);
bool serial_port_write(serial_port *port, const void *buffer, size_t num_bytes, size_t *bytes_written);
bool serial_port_read(serial_port *port, void *buffer, size_t num_bytes, size_t *bytes_read);
uint32_t serial_port_read_count(serial_port *port);
bool serial_port_is_open(serial_port *port);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
