
#include "serial_port.h"

#define COM_PORT_BUFFER_SIZE  0x200

#ifndef WIN32 //Unix only
speed_t baud_rate_to_speed(int baud_rate)
{
    switch(baud_rate)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
#ifdef __linux__ //Linux onnly baudrates
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
#endif
    default:
        return -1;
    }
}
#endif

bool serial_port_open(struct serial_port *port, const char *port_str, int baudrate)
{
    if(port_str == NULL)
        return false;

#ifdef WIN32
    BOOL   ready;
    DCB    dcb;

    //Connect to the provided com port
    port->handle = CreateFile(port_str, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    
    //Check for an invalid handle
    if(port->handle == INVALID_HANDLE_VALUE)
    {
        printf( "\nError: Unable to open com port (%d)\n", GetLastError( ) );
        return false;
    }

    //Setup the com port buffer sizes
    if(SetupComm(port->handle, COM_PORT_BUFFER_SIZE, COM_PORT_BUFFER_SIZE) == 0)
        return false;
    
    //Set the timeouts
    COMMTIMEOUTS timeouts;
    GetCommTimeouts(port->handle, &timeouts);

    // Set the new timeouts
    timeouts.ReadIntervalTimeout         = 1;
    timeouts.ReadTotalTimeoutMultiplier  = 1;
    timeouts.ReadTotalTimeoutConstant    = 1;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant   = 1;

    SetCommTimeouts(port->handle, &timeouts);

    //Setup the com port parameters
    ready = GetCommState(port->handle, &dcb);
    
    //Close the serial port, mutex, and exit
    if(!ready)
    {
        CloseHandle(port->handle);
        return false;
    }

    dcb.BaudRate      = baudrate;   //Baudrate is typically 115200 
    dcb.ByteSize      = 8;          //Charsize is 8,  default for MicroStrain
    dcb.Parity        = NOPARITY;   //Parity is none, default for MicroStrain
    dcb.StopBits      = ONESTOPBIT; //Stopbits is 1,  default for MicroStrain
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl   = DTR_CONTROL_ENABLE;

    ready = SetCommState(port->handle, &dcb);
    
    //Close the serial port and exit
    if(!ready)
    {
        CloseHandle(port->handle);
        return false;
    }
    
#else //Linux

    port->handle = open(port_str, O_RDWR | O_NOCTTY | O_SYNC);

    if (port->handle < 0)
    {
        return false;
    }

    // Set up baud rate and other serial device options
    struct termios serial_port_settings;
    if (tcgetattr(port->handle, &serial_port_settings) < 0)
        return false;

    if (cfsetispeed(&serial_port_settings, baud_rate_to_speed(baudrate)) < 0 || cfsetospeed(&serial_port_settings, baud_rate_to_speed(baudrate)) < 0)
        return false;

    // Other serial settings to match MSCL
    serial_port_settings.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    serial_port_settings.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    serial_port_settings.c_oflag &= (tcflag_t)~(OPOST);
    serial_port_settings.c_iflag &= (tcflag_t)~(INLCR | IGNCR | ICRNL | IGNBRK);

    // Set no parity, 8 bits per byte, no flow control.
    serial_port_settings.c_cflag = (serial_port_settings.c_cflag & (tcflag_t)~(CSIZE|CSTOPB|INPCK|ISTRIP|PARENB|PARODD)) | CS8;
    serial_port_settings.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);

    // Persist the settings
    if(tcsetattr(port->handle, TCSANOW, &serial_port_settings) < 0)
        return false;
    
    // Flush any waiting data
    tcflush(port->handle, TCIOFLUSH);

#endif

    //Success
    port->is_open = true;
    return true;
}

bool serial_port_close(struct serial_port *port)
{
    if(!port->is_open)
        return false;

#ifdef WIN32 //Windows

    //Close the serial port
    CloseHandle(port->handle);

#else //Linux
    close(port->handle);
#endif

    port->is_open = false;
    return true;
}

bool serial_port_write(struct serial_port *port, const void *buffer, size_t num_bytes, size_t *bytes_written)
{
 
    *bytes_written = 0;

    //Check for a valid port handle
    if(!port->is_open)
        return false;
    
#ifdef WIN32 //Windows
    DWORD  local_bytes_written;

    //Call the windows write function
    if(WriteFile(port->handle, buffer, num_bytes, &local_bytes_written, NULL))
    {
        *bytes_written = local_bytes_written;
    
        if(*bytes_written == num_bytes)
            return true;
    }

#else //Linux
    *bytes_written = write(port->handle, buffer, num_bytes);

    if(*bytes_written == num_bytes)
        return true;
    
#endif

    return false;
}

bool serial_port_read(struct serial_port *port, void *buffer, size_t num_bytes, size_t *bytes_read)
{
 
    //Set the bytes read to zero
    *bytes_read = 0;

    //Check for a valid port handle
    if(!port->is_open)
        return false;
  
#ifdef WIN32 //Windows
    DWORD  local_bytes_read;

    //Call the windows read function
    if(!ReadFile(port->handle, buffer, num_bytes, &local_bytes_read, NULL))
        return false;
    *bytes_read = local_bytes_read;

 #else //Linux
    *bytes_read = read(port->handle, buffer, num_bytes);

    if(*bytes_read == (size_t)-1 && errno != EAGAIN)
        return false;

#endif

    return true;
}

uint32_t serial_port_read_count(struct serial_port *port)
{
    //Check for a valid port handle
    if(!port->is_open)
        return 0;
 
#ifdef WIN32 //Windows
    COMSTAT com_status;
    DWORD   errors;
    
    //This function gets the current com status
    if(ClearCommError(port->handle, &errors, &com_status))
    {
        return com_status.cbInQue;
    }
    
#else //Linux
    int bytes_available;
    ioctl(port->handle, FIONREAD, &bytes_available);

    return bytes_available;
#endif

    return 0;
}

bool serial_port_is_open(struct serial_port *port)
{
    return port->is_open;
}
