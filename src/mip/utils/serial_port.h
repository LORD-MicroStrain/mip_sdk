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
///@defgroup Simple implementation for reading and writing to a serial port
///
///@{

struct serial_port
{
    bool is_open;
#ifdef WIN32 //Windows
    HANDLE handle;
#else //Linux
    int handle;
#endif
};


bool serial_port_open(struct serial_port *port, const char *port_str, int baudrate);
bool serial_port_close(struct serial_port *port);
bool serial_port_write(struct serial_port *port, const void *buffer, size_t num_bytes, size_t *bytes_written);
bool serial_port_read(struct serial_port *port, void *buffer, size_t num_bytes, size_t *bytes_read);
uint32_t serial_port_read_count(struct serial_port *port);
bool serial_port_is_open(struct serial_port *port);

///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
