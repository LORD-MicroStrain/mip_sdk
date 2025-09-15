#include "microstrain/connections/serial/serial_port.h"

#include <microstrain/logging.h>

#include <assert.h>
#include <string.h>

#if defined MICROSTRAIN_PLATFORM_WINDOWS
#include <ctype.h>
#include <stdlib.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // !WIN32_LEAN_AND_MEAN

#include <windows.h>
#else // MICROSTRAIN_PLATFORM_UNIX
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#if defined MICROSTRAIN_PLATFORM_APPLE
#include <IOKit/serial/ioss.h>
#endif // MICROSTRAIN_PLATFORM_APPLE
#endif // MICROSTRAIN_PLATFORM_WINDOWS

#define COM_PORT_BUFFER_SIZE 0x200

#if !defined MICROSTRAIN_PLATFORM_WINDOWS

#define INVALID_HANDLE_VALUE -1

#ifdef __cplusplus
namespace microstrain {
namespace C {
#endif // __cplusplus

speed_t baud_rate_to_speed(int _baudrate)
{
    switch (_baudrate)
    {
        case 9600:
        {
            return B9600;
        }
        case 19200:
        {
            return B19200;
        }
        case 38400:
        {
            return B38400;
        }
        case 57600:
        {
            return B57600;
        }
        case 115200:
        {
            return B115200;
        }
        case 230400:
        {
            return B230400;
        }
#if defined MICROSTRAIN_PLATFORM_LINUX
        case 460800:
        {
            return B460800;
        }
        case 500000:
        {
            return B500000;
        }
        case 576000:
        {
            return B576000;
        }
        case 921600:
        {
            return B921600;
        }
        case 1000000:
        {
            return B1000000;
        }
        case 1152000:
        {
            return B1152000;
        }
        case 1500000:
        {
            return B1500000;
        }
        case 2000000:
        {
            return B2000000;
        }
        case 2500000:
        {
            return B2500000;
        }
        case 3000000:
        {
            return B3000000;
        }
        case 3500000:
        {
            return B3500000;
        }
        case 4000000:
        {
            return B4000000;
        }
#endif // MICROSTRAIN_PLATFORM_LINUX
        default:
        {
            return -1;
        }
    }
}
#endif // !MICROSTRAIN_PLATFORM_WINDOWS

void serial_port_init(serial_port* _serial_port, const char* _port_name, const uint32_t _baudrate,
    recording_connection* _recording_connection)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to initialize\n");

        assert(_serial_port);

        return;
    }

    _serial_port->handle               = INVALID_HANDLE_VALUE;
    _serial_port->port_name            = _port_name;
    _serial_port->baudrate             = _baudrate;
    _serial_port->recording_connection = _recording_connection;
}

bool serial_port_open(serial_port* _serial_port)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to open\n");

        assert(_serial_port);

        return false;
    }

    if (serial_port_is_open(_serial_port))
    {
        return true;
    }

    if (_serial_port->port_name == NULL)
    {
        return false;
    }

    MICROSTRAIN_LOG_DEBUG("Opening serial port %s at %d\n", _serial_port->port_name, _serial_port->baudrate);

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    // Prepend '\\.\' to the com port if not already present.
    bool added_prefix = false;
    const char*  tmp_port_str = _serial_port->port_name;
    const size_t port_str_len = strlen(_serial_port->port_name);

    // Only prepend if the port name is of the form 'COMx'
    if (port_str_len >= 4 &&
        toupper(_serial_port->port_name[0]) == 'C' &&
        toupper(_serial_port->port_name[1]) == 'O' &&
        toupper(_serial_port->port_name[2]) == 'M' &&
        isdigit(_serial_port->port_name[3]))
    {
        char* tmp = (char*)malloc(port_str_len + 4 + 1);
        if (!tmp)
        {
            return false;
        }

        tmp[0] = '\\';
        tmp[1] = '\\';
        tmp[2] = '.';
        tmp[3] = '\\';
        memcpy(&tmp[4], _serial_port->port_name, port_str_len + 1);

        added_prefix = true;
        tmp_port_str = tmp;
    }

    // Connect to the provided com port
    _serial_port->handle = CreateFileA((LPCSTR)tmp_port_str, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0,
        NULL);

    // Ensure that the free() call in the following 'if' block doesn't clobber an error value
    const DWORD last_error = GetLastError();
    (void)last_error;

    // If the port string was modified
    if (added_prefix)
    {
        free((char*)tmp_port_str);
        tmp_port_str = NULL;
    }

    // Check for an invalid handle
    if (_serial_port->handle == INVALID_HANDLE_VALUE)
    {
        MICROSTRAIN_LOG_ERROR("Unable to open com port (%d)\n", last_error);
        return false;
    }

    // Set up the com port buffer sizes
    if (SetupComm(_serial_port->handle, COM_PORT_BUFFER_SIZE, COM_PORT_BUFFER_SIZE) == 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to setup com port buffer size (%d)\n", last_error);
        CloseHandle(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

    // Set the timeouts

    COMMTIMEOUTS timeouts;
    GetCommTimeouts(_serial_port->handle, &timeouts);

    // Set the new timeouts
    timeouts.ReadIntervalTimeout         = 10;
    timeouts.ReadTotalTimeoutMultiplier  = 1;
    timeouts.ReadTotalTimeoutConstant    = 10;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant   = 10;

    SetCommTimeouts(_serial_port->handle, &timeouts);

    DCB dcb;

    // Set up the com port parameters
    BOOL ready = GetCommState(_serial_port->handle, &dcb);

    // Close the serial port, mutex, and exit
    if (!ready)
    {
        MICROSTRAIN_LOG_ERROR("Unable to get com state\n");
        CloseHandle(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

    dcb.BaudRate      = _serial_port->baudrate; // Baudrate is typically 115200
    dcb.ByteSize      = 8;                      // Char size is 8, default for MicroStrain
    dcb.Parity        = NOPARITY;               // Parity is none, default for MicroStrain
    dcb.StopBits      = ONESTOPBIT;             // Stop bit is 1, default for MicroStrain
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl   = DTR_CONTROL_ENABLE;

    ready = SetCommState(_serial_port->handle, &dcb);

    // Close the serial port and exit
    if (!ready)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set com state\n");
        CloseHandle(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }
#else // MICROSTRAIN_PLATFORM_UNIX
#if defined MICROSTRAIN_PLATFORM_LINUX
    _serial_port->handle = open(_serial_port->port_name, O_RDWR | O_NOCTTY | O_SYNC);
#else // MICROSTRAIN_PLATFORM_APPLE
    _serial_port->handle = open(_serial_port->port_name, O_RDWR | O_NOCTTY | O_NDELAY);
#endif // !MICROSTRAIN_PLATFORM_LINUX

    if (_serial_port->handle < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to open port (%d): %s\n", errno, strerror(errno));
        return false;
    }

    if (ioctl(_serial_port->handle, TIOCEXCL) < 0)
    {
        MICROSTRAIN_LOG_WARN("Unable to set exclusive mode on serial port (%d): %s\n", errno, strerror(errno));
    }

    // Set up baud rate and other serial device options
    struct termios serial_port_settings;
    if (tcgetattr(_serial_port->handle, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to get serial port settings (%d): %s\n", errno, strerror(errno));
        close(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_LINUX
    if (cfsetispeed(&serial_port_settings, baud_rate_to_speed(_serial_port->baudrate)) < 0 ||
        cfsetospeed(&serial_port_settings, baud_rate_to_speed(_serial_port->baudrate)) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        close(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }
#endif // MICROSTRAIN_PLATFORM_LINUX

    // Other serial settings
    serial_port_settings.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    serial_port_settings.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    serial_port_settings.c_oflag &= (tcflag_t)~(OPOST);
    serial_port_settings.c_iflag &= (tcflag_t)~(INLCR | IGNCR | ICRNL | IGNBRK);

    // Set no parity, 8 bits per byte, no flow control.
    serial_port_settings.c_cflag = (serial_port_settings.c_cflag &
        (tcflag_t)~(CSIZE | CSTOPB | INPCK | ISTRIP | PARENB | PARODD)) | CS8;
    serial_port_settings.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);

    // Persist the settings
    if (tcsetattr(_serial_port->handle, TCSANOW, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to save serial port settings (%d): %s\n", errno, strerror(errno));
        close(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_APPLE
    speed_t speed = _serial_port->baudrate;
    if (ioctl(_serial_port->handle, IOSSIOSPEED, &speed) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        close(_serial_port->handle);
        _serial_port->handle = INVALID_HANDLE_VALUE;
        return false;
    }
#endif // MICROSTRAIN_PLATFORM_APPLE
    // Flush any waiting data
    tcflush(_serial_port->handle, TCIOFLUSH);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    // Success
    return true;
}

bool serial_port_reopen(serial_port* _serial_port, const char* _port_name, const uint32_t _baudrate)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to reopen\n");

        assert(_serial_port);

        return false;
    }

    // Make sure the port is closed
    if (!serial_port_close(_serial_port))
    {
        MICROSTRAIN_LOG_ERROR("Failed to close the serial port to open a new connection\n");

        return false;
    }

    // Update the connection parameters
    _serial_port->port_name = _port_name;
    _serial_port->baudrate  = _baudrate;

    // Open the port with the new parameters
    return serial_port_open(_serial_port);
}

bool serial_port_close(serial_port* _serial_port)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to close\n");

        assert(_serial_port);

        return false;
    }

    if (!serial_port_is_open(_serial_port))
    {
        return true;
    }

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    const bool closed = (bool)CloseHandle(_serial_port->handle);
#else // MICROSTRAIN_PLATFORM_UNIX
    const bool closed = close(_serial_port->handle);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    if (closed)
    {
        _serial_port->handle = INVALID_HANDLE_VALUE;
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Unable to close serial port %s\n", _serial_port->port_name);
    }

    return closed;
}

bool serial_port_is_open(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        return _serial_port->handle != INVALID_HANDLE_VALUE;
    }

    MICROSTRAIN_LOG_ERROR("No serial port to check the open state\n");

    assert(_serial_port);

    return false;
}

bool serial_port_read(serial_port* _serial_port, uint8_t* _buffer, size_t _byte_count, const uint32_t _wait_time,
    size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to read from\n");

        assert(_serial_port);

        return false;
    }

    // Set the bytes read to zero
    *_bytes_read_out = 0;

    // Check for a valid port handle
    if (!serial_port_is_open(_serial_port))
    {
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    if (_wait_time == 0)
    {
        const uint32_t bytes_available = serial_port_read_count(_serial_port);

        const DWORD last_error = GetLastError();
        if (last_error != 0)
        {
            MICROSTRAIN_LOG_ERROR("Failed to read serial port. Error: %lx\n", last_error);
            serial_port_close(_serial_port);
            return false;
        }

        if (bytes_available == 0)
        {
            return true;
        }

        // Don't let Windows block on the read
        if (bytes_available < _byte_count)
        {
            _byte_count = bytes_available;
        }
    }

    DWORD local_bytes_read;

    // Call the windows read function
    if (!ReadFile(_serial_port->handle, _buffer, (DWORD)_byte_count, &local_bytes_read, NULL))
    {
        return false;
    }

    *_bytes_read_out = (size_t)local_bytes_read;
 #else // MICROSTRAIN_PLATFORM_UNIX
    // Poll the device before attempting to read any data, so we will only block for 10ms if there is no data available
    struct pollfd poll_fd = { .fd = _serial_port->handle, .events = POLLIN };
    int poll_status = poll(&poll_fd, 1, _wait_time);

    // Keep reading and polling while there is still data available
    if (poll_status == -1)
    {
        MICROSTRAIN_LOG_ERROR("Failed to poll serial port (%d): %s\n", errno, strerror(errno));
        return false;
    }
    else if (poll_fd.revents & POLLHUP)
    {
        MICROSTRAIN_LOG_ERROR("Poll encountered HUP, closing device");
        serial_port_close(_serial_port);
        return false;
    }
    else if (poll_fd.revents & POLLERR || poll_fd.revents & POLLNVAL)
    {
        MICROSTRAIN_LOG_ERROR("Poll encountered error\n");
        return false;
    }
    else if (poll_status > 0 && poll_fd.revents & POLLIN)
    {
        ssize_t local_bytes_read = read(_serial_port->handle, _buffer, _byte_count);

        if (local_bytes_read == (ssize_t)-1 && errno != EAGAIN)
        {
            MICROSTRAIN_LOG_ERROR("Failed to read serial data (%d): %s\n", errno, strerror(errno));
            return false;
        }
        if (local_bytes_read >= 0)
        {
            *_bytes_read_out = local_bytes_read;
        }
    }
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    *_timestamp_out = microstrain_get_current_timestamp();

    // Write to a recording file for received data if it has been set
    if (serial_port_receive_recording_enabled(_serial_port))
    {
        recording_connection_write_received_bytes(_serial_port->recording_connection, _buffer, _byte_count, NULL);
    }

    return true;
}

bool serial_port_write(serial_port* _serial_port, const uint8_t* _buffer, size_t _byte_count,
    size_t* _bytes_written_out)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to write to\n");

        assert(_serial_port);

        return false;
    }

    assert(_bytes_written_out);

    *_bytes_written_out = 0;

    // Check for a valid port handle
    if (!serial_port_is_open(_serial_port))
    {
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    DWORD local_bytes_written;

    // Call the windows write function
    if (WriteFile(_serial_port->handle, _buffer, (DWORD)_byte_count, &local_bytes_written, NULL))
    {
        *_bytes_written_out = local_bytes_written;
    }
#else // MICROSTRAIN_PLATFORM_UNIX
    *_bytes_written_out = write(_serial_port->handle, _buffer, _byte_count);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    if (*_bytes_written_out == _byte_count)
    {
        // Write to a recording file for sent data if it has been set
        if (serial_port_send_recording_enabled(_serial_port))
        {
            recording_connection_write_sent_bytes(_serial_port->recording_connection, _buffer, _byte_count, NULL);
        }

        return true;
    }
#if defined MICROSTRAIN_PLATFORM_UNIX
    if (*_bytes_written_out == (size_t)-1)
    {
        MICROSTRAIN_LOG_ERROR("Failed to write serial data (%d): %s\n", errno, strerror(errno));
    }
#endif // MICROSTRAIN_PLATFORM_UNIX

    return false;
}

bool serial_port_update_baudrate(serial_port* _serial_port, const uint32_t _baudrate)
{
    if (!serial_port_is_open(_serial_port))
    {
        return false;
    }

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    DCB dcb;

    if (GetCommState(_serial_port->handle, &dcb) == 0)
    {
        MICROSTRAIN_LOG_ERROR("GetCommState() failed with error code %d\n", GetLastError());
        return false;
    }

    dcb.BaudRate = _baudrate;

    if (SetCommState(_serial_port->handle, &dcb) == 0)
    {
        MICROSTRAIN_LOG_ERROR("SetCommState() failed with error code %d\n", GetLastError());
        return false;
    }
#elif defined MICROSTRAIN_PLATFORM_LINUX
    // Get existing settings
    struct termios serial_port_settings;
    if (tcgetattr(_serial_port->handle, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to get serial port settings (%d): %s\n", errno, strerror(errno));
        return false;
    }

    if (cfsetispeed(&serial_port_settings, baud_rate_to_speed(_baudrate)) < 0 ||
        cfsetospeed(&serial_port_settings, baud_rate_to_speed(_baudrate)) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        return false;
    }

    // Persist the settings
    if (tcsetattr(_serial_port->handle, TCSANOW, &serial_port_settings) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to save serial port settings (%d): %s\n", errno, strerror(errno));
        return false;
    }
#elif defined MICROSTRAIN_PLATFORM_APPLE
    speed_t speed = _baudrate;
    if (ioctl(_serial_port->handle, IOSSIOSPEED, &speed) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Unable to set baud rate (%d): %s\n", errno, strerror(errno));
        return false;
    }
#else // Unknown
    (void)_serial_port;
    (void)_baudrate;

    return false;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    // Update the baudrate for the connection
    _serial_port->baudrate = _baudrate;

    return true;
}

uint32_t serial_port_read_count(serial_port* _serial_port)
{
    if (!_serial_port)
    {
        MICROSTRAIN_LOG_ERROR("No serial port to check read count from\n");

        assert(_serial_port);

        return false;
    }

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    // Clear the last error, if any
    SetLastError(0);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    // Check for a valid port handle
    if (!serial_port_is_open(_serial_port))
    {
        return 0;
    }

#if defined MICROSTRAIN_PLATFORM_WINDOWS
    COMSTAT com_status;
    DWORD   errors;

    // This function gets the current com status
    if (ClearCommError(_serial_port->handle, &errors, &com_status))
    {
        return com_status.cbInQue;
    }
#else // MICROSTRAIN_PLATFORM_UNIX
    int bytes_available;
    ioctl(_serial_port->handle, FIONREAD, &bytes_available);

    return bytes_available;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    return 0;
}

void serial_port_init_receive_recording_stream(const serial_port* _serial_port, FILE* _receive_stream)
{
    if (_serial_port)
    {
        recording_connection_init_receive_stream(_serial_port->recording_connection, _receive_stream);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for receive bytes recording stream initialization\n");

        assert(false);
    }
}

void serial_port_open_receive_recording_file(const serial_port* _serial_port, const char* _receive_file_name)
{
    if (_serial_port)
    {
        recording_connection_open_receive_file(_serial_port->recording_connection, _receive_file_name);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for receive bytes recording file initialization\n");

        assert(false);
    }
}

void serial_port_close_receive_recording_stream(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        recording_connection_close_receive_stream(_serial_port->recording_connection);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for receive bytes recording stream closing\n");

        assert(false);
    }
}

bool serial_port_receive_recording_enabled(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        return recording_connection_receive_recording_enabled(_serial_port->recording_connection);
    }

    MICROSTRAIN_LOG_ERROR("Null serial port for receive bytes recording stream checking\n");

    assert(false);

    return false;
}

void serial_port_init_send_recording_stream(const serial_port* _serial_port, FILE* _send_stream)
{
    if (_serial_port)
    {
        recording_connection_init_send_stream(_serial_port->recording_connection, _send_stream);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for send bytes recording stream initialization\n");

        assert(false);
    }
}

void serial_port_open_send_recording_file(const serial_port* _serial_port, const char* _send_file_name)
{
    if (_serial_port)
    {
        recording_connection_open_send_file(_serial_port->recording_connection, _send_file_name);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for send bytes recording file initialization\n");

        assert(false);
    }
}

void serial_port_close_send_recording_stream(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        recording_connection_close_send_stream(_serial_port->recording_connection);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for send bytes recording stream closing\n");

        assert(false);
    }
}

bool serial_port_send_recording_enabled(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        return recording_connection_send_recording_enabled(_serial_port->recording_connection);
    }

    MICROSTRAIN_LOG_ERROR("Null serial port for send bytes recording stream checking\n");

    assert(false);

    return false;
}

void serial_port_init_recording_streams(const serial_port* _serial_port, FILE* _receive_stream, FILE* _send_stream)
{
    if (_serial_port)
    {
        serial_port_init_receive_recording_stream(_serial_port, _receive_stream);
        serial_port_init_send_recording_stream(_serial_port, _send_stream);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for bytes recording stream initialization\n");

        assert(false);
    }
}

void serial_port_open_recording_files(const serial_port* _serial_port, const char* _receive_file_name,
    const char* _send_file_name)
{
    if (_serial_port)
    {
        serial_port_open_receive_recording_file(_serial_port, _receive_file_name);
        serial_port_open_send_recording_file(_serial_port, _send_file_name);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for bytes recording file initialization\n");

        assert(false);
    }
}

void serial_port_close_recording_streams(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        serial_port_close_receive_recording_stream(_serial_port);
        serial_port_close_send_recording_stream(_serial_port);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null serial port for bytes recording stream closing\n");

        assert(false);
    }
}

bool serial_port_recording_enabled(const serial_port* _serial_port)
{
    if (_serial_port)
    {
        return serial_port_receive_recording_enabled(_serial_port) || serial_port_send_recording_enabled(_serial_port);
    }

    MICROSTRAIN_LOG_ERROR("Null serial port for bytes recording stream checking\n");

    assert(false);

    return false;
}

#ifdef __cplusplus
} // namespace C
} // namespace microstrain
#endif // __cplusplus
