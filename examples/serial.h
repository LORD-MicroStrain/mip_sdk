/////////////////////////////////////////////////////////////////////////////
//
// Simple C Serial Port Definition File
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SERIAL_HPP
#define _SERIAL_HPP

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32
    #include <windows.h>

#else

    #include <sys/ioctl.h>
    #include <termios.h> // terminal io (serial port) interface
    #include <fcntl.h>  // File control definitions
    #include <errno.h>  // Error number definitions
    #include <unistd.h> // close() definition

#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define COM_PORT_BUFFER_SIZE  0x200

////////////////////////////////////////////////////////////////////////////////
//
// Serial Port Structure
//
////////////////////////////////////////////////////////////////////////////////

typedef struct _serial_port
{
  bool is_open;

  #ifdef _WIN32 //Windows
      HANDLE handle;
  #else //Linux
	 int handle;
  #endif

}serial_port;



bool serial_port_open(serial_port *port, const char *port_str, int baudrate);
bool serial_port_close(serial_port *port);
bool serial_port_write(serial_port *port, const void *buffer, uint32_t num_bytes, uint32_t *bytes_written);
bool serial_port_read(serial_port *port, void *buffer, uint32_t num_bytes, uint32_t *bytes_read);
uint32_t serial_port_read_count(serial_port *port);
bool serial_port_is_open(serial_port *port);

#ifndef _WIN32 //Linux only
  speed_t baud_rate_to_speed(int baud_rate);
#endif

#endif
