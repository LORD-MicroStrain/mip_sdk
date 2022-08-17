/////////////////////////////////////////////////////////////////////////////
//
// Simple C++ Serial Port Class Definition File
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SERIAL_CLASS_HPP
#define _SERIAL_CLASS_HPP

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "serial.h"

////////////////////////////////////////////////////////////////////////////////
//
// Serial Port Class 
//
////////////////////////////////////////////////////////////////////////////////

class SerialPort
{
 private:
  serial_port m_port;

 public:
  bool Open(char *port_str, int baudrate) { return serial_port_open(&m_port, port_str, baudrate); };
  bool Close() { return serial_port_close(&m_port); };
  bool Write(void *buffer, uint32_t num_bytes, uint32_t *bytes_written) { return serial_port_write(&m_port, buffer, num_bytes, bytes_written); };
  bool Read(void *buffer, uint32_t num_bytes, uint32_t *bytes_read) { return serial_port_read(&m_port, buffer, num_bytes, bytes_read);};
  uint32_t Count() { return serial_port_read_count(&m_port);};
  bool IsOpen() {return serial_port_is_open(&m_port);};

};


#endif
