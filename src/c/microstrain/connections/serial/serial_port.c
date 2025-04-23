#include "serial_port.h"

#include <microstrain/logging.h>

#if defined MICROSTRAIN_PLATFORM_WINDOWS
#include <ctype.h>
#include <stdlib.h>
#elif defined MICROSTRAIN_PLATFORM_APPLE
#include <IOKit/serial/ioss.h>
#endif // MICROSTRAIN_PLATFORM_WINDOWS

#define COM_PORT_BUFFER_SIZE  0x200

#if !defined MICROSTRAIN_PLATFORM_WINDOWS

#define INVALID_HANDLE_VALUE -1

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
#if defined MICROSTRAIN_PLATFORM_LINUX
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
#endif // MICROSTRAIN_PLATFORM_LINUX
    default:
        return -1;
    }
}
#endif // !MICROSTRAIN_PLATFORM_WINDOWS

void serial_port_init(serial_port* port)
{
    port->handle = INVALID_HANDLE_VALUE;
}

bool serial_port_open(serial_port* port, const char* port_str, int baudrate)
{
    if (port_str == NULL)
        return false;

    MICROSTRAIN_LOG_DEBUG("Opening serial port %s at %d\n", port_str, baudrate);

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    BOOL   ready;
    DCB    dcb;

    // Prepend '\\.\' to the com port if not already present.
    bool added_prefix = false;
    const char* tmp_port_str = port_str;
    size_t port_str_len = strlen(port_str);

    // Only prepend if port_str is of the form 'COMx'
    if (port_str_len >= 4 && toupper(port_str[0]) == 'C' && toupper(port_str[1]) == 'O' && toupper(port_str[2]) == 'M' && isdigit(port_str[3]))
    {
        char* tmp = (char*)malloc(port_str_len + 4 + 1);
        if (!tmp)
            return false;

        tmp[0] = '\\';
        tmp[1] = '\\';
        tmp[2] = '.';
        tmp[3] = '\\';
        memcpy(&tmp[4], port_str, port_str_len+1);

        added_prefix = true;
        tmp_port_str = tmp;
    }

    // Connect to the provided com port
    port->handle = CreateFileA(tmp_port_str, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    // Ensure that the free() call in the following 'if' block doesn't clobber an error value
    DWORD last_error = GetLastError();
    (void)last_error;

    // If the port string was modified
    if (added_prefix)
    {
        free((char*)tmp_port_str);
        tmp_port_str = NULL;
    }

    // Check for an invalid handle
    if (port->handle == INVALID_HANDLE_VALUE)
    {
        MICROSTRAIN_LOG_ERROR("Unable to open com port (%d)\n", last_error);
        return false;
    }

    // Set up the com port buffer sizes
    if (SetupComm(port->handle, COM_PORT_BUFFER_SIZE, COM_PORT_BUFFER_SIZE) == 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to setup com port buffer size (%d)\n", last_error);
        CloseHandle(port->handle);
        port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

    // Set the timeouts

    COMMTIMEOUTS timeouts;
    GetCommTimeouts(port->handle, &timeouts);

    // Set the new timeouts
    timeouts.ReadIntervalTimeout         = 10;
    timeouts.ReadTotalTimeoutMultiplier  = 1;
    timeouts.ReadTotalTimeoutConstant    = 10;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant   = 10;

    SetCommTimeouts(port->handle, &timeouts);

    // Set up the com port parameters
    ready = GetCommState(port->handle, &dcb);

    // Close the serial port, mutex, and exit
    if (!ready)
    {
        MICROSTRAIN_LOG_ERROR("Unable to get com state\n");
        CloseHandle(port->handle);
        port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

    dcb.BaudRate      = baudrate;   // Baudrate is typically 115200
    dcb.ByteSize      = 8;          // Char size is 8, default for MicroStrain
    dcb.Parity        = NOPARITY;   // Parity is none, default for MicroStrain
    dcb.StopBits      = ONESTOPBIT; // Stop bits is 1, default for MicroStrain
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl   = DTR_CONTROL_ENABLE;

    ready = SetCommState(port->handle, &dcb);

    // Close the serial port and exit
    if (!ready)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set com state\n");
        CloseHandle(port->handle);
        port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

#else // Unix
#if defined MICROSTRAIN_PLATFORM_LINUX
    port->handle = open(port_str, O_RDWR | O_NOCTTY | O_SYNC);
#else // Apple
    port->handle = open(port_str, O_RDWR | O_NOCTTY | O_NDELAY);
#endif // !MICROSTRAIN_PLATFORM_LINUX

    if (port->handle < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to open port (%d): %s\n", errno, strerror(errno));
        return false;
    }

    if (ioctl(port->handle, TIOCEXCL) < 0)
    {
        MICROSTRAIN_LOG_WARN("Unable to set exclusive mode on serial port (%d): %s\n", errno, strerror(errno));
    }

    // Set up baud rate and other serial device options
    struct termios serial_port_settings;
    if (tcgetattr(port->handle, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to get serial port settings (%d): %s\n", errno, strerror(errno));
        close(port->handle);
        port->handle = -1;
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_LINUX
    if (cfsetispeed(&serial_port_settings, baud_rate_to_speed(baudrate)) < 0 || cfsetospeed(&serial_port_settings, baud_rate_to_speed(baudrate)) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        close(port->handle);
        port->handle = -1;
        return false;
    }
#endif // MICROSTRAIN_PLATFORM_LINUX

    // Other serial settings to match MSCL
    serial_port_settings.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    serial_port_settings.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    serial_port_settings.c_oflag &= (tcflag_t)~(OPOST);
    serial_port_settings.c_iflag &= (tcflag_t)~(INLCR | IGNCR | ICRNL | IGNBRK);

    // Set no parity, 8 bits per byte, no flow control.
    serial_port_settings.c_cflag = (serial_port_settings.c_cflag & (tcflag_t)~(CSIZE|CSTOPB|INPCK|ISTRIP|PARENB|PARODD)) | CS8;
    serial_port_settings.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);

    // Persist the settings
    if (tcsetattr(port->handle, TCSANOW, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to save serial port settings (%d): %s\n", errno, strerror(errno));
        close(port->handle);
        port->handle = -1;
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_APPLE
    speed_t speed = baudrate;
    if (ioctl(port->handle, IOSSIOSPEED, &speed) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        close(port->handle);
        port->handle = -1;
        return false;
    }
#endif // MICROSTRAIN_PLATFORM_APPLE

    // Flush any waiting data
    tcflush(port->handle, TCIOFLUSH);

#endif // MICROSTRAIN_PLATFORM_WINDOWS

    // Success
    return true;
}

bool serial_port_set_baudrate(serial_port* port, int baudrate)
{
    if (!serial_port_is_open(port))
        return false;

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    DCB dcb;

    if (GetCommState(port->handle, &dcb) == 0)
    {
        MICROSTRAIN_LOG_ERROR("GetCommState() failed with error code %d\n", GetLastError());
        return false;
    }

    dcb.BaudRate = baudrate;

    if (SetCommState(port->handle, &dcb) == 0)
    {
        MICROSTRAIN_LOG_ERROR("SetCommState() failed with error code %d\n", GetLastError());
        return false;
    }
#elif defined MICROSTRAIN_PLATFORM_LINUX
    // Get existing settings
    struct termios serial_port_settings;
    if (tcgetattr(port->handle, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to get serial port settings (%d): %s\n", errno, strerror(errno));
        return false;
    }

    if (cfsetispeed(&serial_port_settings, baud_rate_to_speed(baudrate)) < 0 || cfsetospeed(&serial_port_settings, baud_rate_to_speed(baudrate)) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        return false;
    }

    // Persist the settings
    if (tcsetattr(port->handle, TCSANOW, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to save serial port settings (%d): %s\n", errno, strerror(errno));
        return false;
    }
#elif defined MICROSTRAIN_PLATFORM_APPLE
    speed_t speed = baudrate;
    if (ioctl(port->handle, IOSSIOSPEED, &speed) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        return false;
    }
#else // Unknown
    (void)port;
    (void)baudrate;

    return false;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    return true;
}

bool serial_port_close(serial_port* port)
{
    if (!serial_port_is_open(port))
        return false;

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    CloseHandle(port->handle);
#else // Unix
    close(port->handle);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    port->handle = INVALID_HANDLE_VALUE;

    return true;
}

bool serial_port_write(serial_port* port, const void* buffer, size_t num_bytes, size_t* bytes_written)
{
    *bytes_written = 0;

    // Check for a valid port handle
    if (!serial_port_is_open(port))
        return false;

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    DWORD  local_bytes_written;

    // Call the windows write function
    if (WriteFile(port->handle, buffer, (DWORD)num_bytes, &local_bytes_written, NULL))
    {
        *bytes_written = local_bytes_written;

        if (*bytes_written == num_bytes)
            return true;
    }
#else // Unix
    *bytes_written = write(port->handle, buffer, num_bytes);

    if (*bytes_written == num_bytes)
        return true;
    else if (*bytes_written == (size_t)-1)
        MICROSTRAIN_LOG_ERROR("Failed to write serial data (%d): %s\n", errno, strerror(errno));

#endif // MICROSTRAIN_PLATFORM_WINDOWS

    return false;
}

bool serial_port_read(serial_port* port, void* buffer, size_t num_bytes, int wait_time, size_t* bytes_read)
{
    // Set the bytes read to zero
    *bytes_read = 0;

    // Check for a valid port handle
    if (!serial_port_is_open(port))
        return false;

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    if (wait_time == 0)
    {
        uint32_t bytes_available = serial_port_read_count(port);

        DWORD last_error = GetLastError();
        if (last_error != 0)
        {
            MICROSTRAIN_LOG_ERROR("Failed to read serial port. Error: %lx\n", last_error);
            serial_port_close(port);
            return false;
        }

        if (bytes_available == 0)
            return true;

        // Don't let Windows block on the read
        if (bytes_available < num_bytes)
            num_bytes = bytes_available;
    }

    DWORD  local_bytes_read;

    // Call the windows read function
    if (!ReadFile(port->handle, buffer, (DWORD)num_bytes, &local_bytes_read, NULL))
        return false;
    *bytes_read = (size_t)local_bytes_read;

 #else // Unix
    // Poll the device before attempting to read any data, so we will only block for 10ms if there is no data available
    struct pollfd poll_fd = { .fd = port->handle, .events = POLLIN };
    int poll_status = poll(&poll_fd, 1, wait_time);

    // Keep reading and polling while there is still data available
    if (poll_status == -1)
    {
        MICROSTRAIN_LOG_ERROR("Failed to poll serial port (%d): %s\n", errno, strerror(errno));
        return false;
    }
    else if (poll_fd.revents & POLLHUP)
    {
        MICROSTRAIN_LOG_ERROR("Poll encountered HUP, closing device");
        serial_port_close(port);
        return false;
    }
    else if (poll_fd.revents & POLLERR || poll_fd.revents & POLLNVAL)
    {
        MICROSTRAIN_LOG_ERROR("Poll encountered error\n");
        return false;
    }
    else if (poll_status > 0 && poll_fd.revents & POLLIN)
    {
        ssize_t local_bytes_read = read(port->handle, buffer, num_bytes);

        if (local_bytes_read == (ssize_t)-1 && errno != EAGAIN)
        {
            MICROSTRAIN_LOG_ERROR("Failed to read serial data (%d): %s\n", errno, strerror(errno));
            return false;
        }
        if (local_bytes_read >= 0)
        {
            *bytes_read = local_bytes_read;
        }
    }

#endif // MICROSTRAIN_PLATFORM_WINDOWS

    return true;
}

uint32_t serial_port_read_count(serial_port* port)
{
#if defined MICROSTRAIN_PLATFORM_WINDOWS
    // Clear the last error, if any
    SetLastError(0);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    // Check for a valid port handle
    if (!serial_port_is_open(port))
        return 0;

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    COMSTAT com_status;
    DWORD   errors;

    // This function gets the current com status
    if (ClearCommError(port->handle, &errors, &com_status))
    {
        return com_status.cbInQue;
    }

#else // Unix
    int bytes_available;
    ioctl(port->handle, FIONREAD, &bytes_available);

    return bytes_available;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    return 0;
}

bool serial_port_is_open(const serial_port* port)
{
#if defined MICROSTRAIN_PLATFORM_WINDOWS
    return port->handle != INVALID_HANDLE_VALUE;
#else // Unix
    return port->handle >= 0;
#endif // MICROSTRAIN_PLATFORM_WINDOWS
}
