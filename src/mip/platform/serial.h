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

#include <mip/mip_interface.h>

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



bool serial_port_open(serial_port *port, char *port_str, int baudrate);
bool serial_port_close(serial_port *port);
bool serial_port_write(serial_port *port, void *buffer, uint32_t num_bytes, uint32_t *bytes_written);
bool serial_port_read(serial_port *port, void *buffer, uint32_t num_bytes, uint32_t *bytes_read);
uint32_t serial_port_read_count(serial_port *port);
bool serial_port_is_open(serial_port *port);

  #ifndef _WIN32 //Linux only
    speed_t baud_rate_to_speed(int baud_rate);
  #endif

#endif




/*
#ifdef _WIN32
    typedef HANDLE serial_device_interface_handle;
#else
    typedef int serial_device_interface_handle;
#endif

struct serial_device_interface
{
};
*/

bool serial_device_interface_open(serial_port* port, const char* port_str, int baudrate);
bool serial_device_interface_close(serial_port* port);
bool serial_device_interface_update(serial_port* port, struct mip_interface* device);
bool serial_device_interface_send_to_device(serial_port* port, struct mip_interface* device, const uint8_t* data, size_t length);