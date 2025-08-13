#include "microstrain/connections/tcp/tcp_socket.h"

#include <microstrain/logging.h>

#include <assert.h>
#include <stdio.h>

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>

#ifdef MICROSTRAIN_PLATFORM_MSVC
typedef int ssize_t;
#endif //MICROSTRAIN_PLATFORM_MSVC

static const int SEND_FLAGS = 0;
#else // MICROSTRAIN_PLATFORM_UNIX
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <string.h>

static const int INVALID_SOCKET = -1;
static const int SEND_FLAGS     = MSG_NOSIGNAL;
#endif // MICROSTRAIN_PLATFORM_WINDOWS

#ifdef __cplusplus
namespace microstrain {
namespace C {
#endif // __cplusplus

void tcp_client_init(tcp_client* _tcp_client, const char* _hostname, const uint16_t _port,
    recording_connection* _recording_connection)
{
    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to initialize\n");

        assert(_tcp_client);

        return;
    }

    _tcp_client->handle               = INVALID_SOCKET;
    _tcp_client->hostname             = _hostname;
    _tcp_client->port                 = _port;
    _tcp_client->recording_connection = _recording_connection;
}

bool tcp_client_open(tcp_client* _tcp_client, const uint32_t _timeout_ms)
{
    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to open\n");

        assert(_tcp_client);

        return false;
    }

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    // Initialize winsock for each connection since there's no global init function.
    // This is safe to do multiple times, as long as it's shutdown the same number of times.
    struct WSAData wsaData;
    const int startup_result = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if (startup_result != 0)
    {
        MICROSTRAIN_LOG_ERROR("WSAStartup() failed: %d\n", startup_result);

        return false;
    }
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    typedef struct addrinfo addrinfo;

    // https://man7.org/linux/man-pages/man3/getaddrinfo.3.html
    addrinfo hints;
    addrinfo *info;

    memset(&hints, 0, sizeof(hints));

    hints.ai_family   = AF_INET; // AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags    = 0;

    char port_str[6]; // Maximum 5 digits
    snprintf(port_str, sizeof(port_str), "%d", _tcp_client->port);

    const int result = getaddrinfo(_tcp_client->hostname, port_str, &hints, &info);

    if (result != 0)
    {
        return false;
    }

    for (const addrinfo* addr = info; addr != NULL; addr=addr->ai_next)
    {
        _tcp_client->handle = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        if (_tcp_client->handle == INVALID_SOCKET)
        {
            continue;
        }

        const int success = connect(_tcp_client->handle, addr->ai_addr, (int)addr->ai_addrlen);

        if (success == 0)
        {
            break;
        }

        tcp_client_close(_tcp_client);
#ifdef MICROSTRAIN_PLATFORM_WINDOWS
        closesocket(_tcp_client->handle);
#else // MICROSTRAIN_PLATFORM_UNIX
        close(_tcp_client->handle);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

        _tcp_client->handle = INVALID_SOCKET;
    }

    freeaddrinfo(info);

    if (_tcp_client->handle == INVALID_SOCKET)
    {
        return false;
    }

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    const char* timeout_option      = (char*)&_timeout_ms;
    const int   timeout_option_size = sizeof(_timeout_ms);
#else // MICROSTRAIN_PLATFORM_UNIX
    struct timeval timeval;
    timeval.tv_sec  = _timeout_ms / 1000;
    timeval.tv_usec = (_timeout_ms % 1000) * 1000;

    const void*     timeout_option      = (void*)&timeval;
    const socklen_t timeout_option_size = sizeof(timeval);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    // Receive
    if (0 != setsockopt(_tcp_client->handle, SOL_SOCKET, SO_RCVTIMEO, timeout_option, timeout_option_size))
    {
        return false;
    }

    // Send
    if (0 != setsockopt(_tcp_client->handle, SOL_SOCKET, SO_SNDTIMEO, timeout_option, timeout_option_size))
    {
        return false;
    }

    // Save the timeout for reopening or updating the connection
    _tcp_client->timeout = _timeout_ms;

    return true;
}

bool tcp_client_reopen(tcp_client* _tcp_client, const char* _hostname, const uint16_t _port, const uint32_t _timeout_ms)
{
    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to reopen\n");

        assert(_tcp_client);

        return false;
    }

    // Make sure the socket is closed
    if (!tcp_client_close(_tcp_client))
    {
        MICROSTRAIN_LOG_ERROR("Failed to close the TCP client to open a new connection\n");

        return false;
    }

    // Update the connection parameters
    _tcp_client->hostname = _hostname;
    _tcp_client->port     = _port;

    // Open the socket with the new parameters
    return tcp_client_open(_tcp_client, _timeout_ms);
}

bool tcp_client_close(tcp_client* _tcp_client)
{
    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to close\n");

        assert(_tcp_client);

        return false;
    }

    if (_tcp_client->handle == INVALID_SOCKET)
    {
        return true;
    }

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    closesocket(_tcp_client->handle);
    WSACleanup(); // See tcp_socket_open
#else // MICROSTRAIN_PLATFORM_UNIX
    close(_tcp_client->handle);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    _tcp_client->handle = INVALID_SOCKET;

    return true;
}

bool tcp_client_is_open(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        return _tcp_client->handle != INVALID_SOCKET;
    }

    MICROSTRAIN_LOG_ERROR("No TCP client to check if open\n");

    assert(_tcp_client);

    return false;
}

bool tcp_client_read(tcp_client* _tcp_client, uint8_t* _buffer, const size_t _byte_count, const uint32_t _wait_time,
    size_t* _bytes_read_out, microstrain_embedded_timestamp* _timestamp_out)
{
    // Timeout is always fixed
    (void)_wait_time;

    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to read from\n");

        assert(_tcp_client);

        return false;
    }

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    const int local_bytes_read = recv(_tcp_client->handle, (char*)_buffer, (int)_byte_count, SEND_FLAGS);
#else // MICROSTRAIN_PLATFORM_UNIX
    const ssize_t local_bytes_read = recv(_tcp_client->handle, (void*)_buffer, _byte_count, SEND_FLAGS);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

    if (local_bytes_read < 0)
    {
#ifdef MICROSTRAIN_PLATFORM_WINDOWS
        return false;
#else // MICROSTRAIN_PLATFORM_UNIX
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            return false;
        }

        return true;
#endif // MICROSTRAIN_PLATFORM_WINDOWS
    }

    // Throw an error if the connection has been closed by the other side.
    if (local_bytes_read == 0)
    {
        return false;
    }

    *_bytes_read_out = (size_t)local_bytes_read;

    *_timestamp_out = microstrain_get_current_timestamp();

    // Write to a recording file for received data if it has been set
    if (tcp_client_receive_recording_enabled(_tcp_client))
    {
        recording_connection_write_received_bytes(_tcp_client->recording_connection, _buffer, _byte_count, NULL);
    }

    return true;
}

bool tcp_client_write(tcp_client* _tcp_client, const uint8_t* _buffer, const size_t _byte_count,
    size_t* _bytes_written_out)
{
    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to write to\n");

        assert(_tcp_client);

        return false;
    }

    *_bytes_written_out = 0;

    while (*_bytes_written_out < _byte_count)
    {
#ifdef MICROSTRAIN_PLATFORM_WINDOWS
        const int sent = send(_tcp_client->handle, (const char*)_buffer, (int)_byte_count, SEND_FLAGS);
#else // MICROSTRAIN_PLATFORM_UNIX
        const ssize_t sent = send(_tcp_client->handle, (const void*)_buffer, _byte_count, SEND_FLAGS);
#endif // MICROSTRAIN_PLATFORM_WINDOWS

        if (sent < 0)
        {
            return false;
        }

        *_bytes_written_out += (size_t)sent;
    }

    if (*_bytes_written_out != _byte_count)
    {
        return false;
    }

    // Write to a recording file for sent data if it has been set
    if (tcp_client_send_recording_enabled(_tcp_client))
    {
        recording_connection_write_sent_bytes(_tcp_client->recording_connection, _buffer, _byte_count, NULL);
    }

    return true;
}

bool tcp_client_update_port(tcp_client* _tcp_client, const uint16_t _port)
{
    if (!_tcp_client)
    {
        MICROSTRAIN_LOG_ERROR("No TCP client to update the port for\n");

        assert(_tcp_client);

        return false;
    }

    // Store the old port in case we need to undo the changes
    const uint16_t old_port = _tcp_client->port;

    // Update the port
    _tcp_client->port = _port;

    // No connection to update
    if (!tcp_client_is_open(_tcp_client))
    {
        return true;
    }

    // Close and reopen the connection with the new port

    // Close the current connection
    if (!tcp_client_close(_tcp_client))
    {
        MICROSTRAIN_LOG_ERROR("Failed to close the TCP client during port update\n");

        // Restore old port on failure
        _tcp_client->port = old_port;

        return false;
    }

    // Attempt to reopen with the new port
    if (!tcp_client_open(_tcp_client, _tcp_client->timeout))
    {
        MICROSTRAIN_LOG_ERROR("Failed to reopen the TCP client with the new port %d\n", _port);

        // Try to restore connection with the old port
        _tcp_client->port = old_port;

        if (!tcp_client_open(_tcp_client, _tcp_client->timeout))
        {
            MICROSTRAIN_LOG_ERROR("Failed to restore the TCP client connection with the original port %d\n", old_port);
        }

        return false;
    }

    MICROSTRAIN_LOG_INFO("Successfully updated the TCP client port from %d to %d\n", old_port, _port);

    return true;
}

void tcp_client_init_receive_recording_stream(const tcp_client* _tcp_client, FILE* _receive_stream)
{
    if (_tcp_client)
    {
        recording_connection_init_receive_stream(_tcp_client->recording_connection, _receive_stream);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for receive bytes recording stream initialization\n");

        assert(false);
    }
}

void tcp_client_init_receive_recording_file(const tcp_client* _tcp_client, const char* _receive_file_name)
{
    if (_tcp_client)
    {
        recording_connection_open_receive_file(_tcp_client->recording_connection, _receive_file_name);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for receive bytes recording file initialization\n");

        assert(false);
    }
}

void tcp_client_close_receive_recording_stream(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        recording_connection_close_receive_file(_tcp_client->recording_connection);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for receive bytes recording stream closing\n");

        assert(false);
    }
}

bool tcp_client_receive_recording_enabled(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        return recording_connection_receive_recording_enabled(_tcp_client->recording_connection);
    }

    MICROSTRAIN_LOG_ERROR("Null TCP client for receive bytes recording stream checking\n");

    assert(false);

    return false;
}

void tcp_client_init_send_recording_stream(const tcp_client* _tcp_client, FILE* _send_stream)
{
    if (_tcp_client)
    {
        recording_connection_init_send_stream(_tcp_client->recording_connection, _send_stream);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for send bytes recording stream initialization\n");

        assert(false);
    }
}

void tcp_client_init_send_recording_file(const tcp_client* _tcp_client, const char* _send_file_name)
{
    if (_tcp_client)
    {
        recording_connection_open_send_file(_tcp_client->recording_connection, _send_file_name);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for send bytes recording file initialization\n");

        assert(false);
    }
}

void tcp_client_close_send_recording_stream(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        recording_connection_close_send_file(_tcp_client->recording_connection);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for send bytes recording stream closing\n");

        assert(false);
    }
}

bool tcp_client_send_recording_enabled(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        return recording_connection_send_recording_enabled(_tcp_client->recording_connection);
    }

    MICROSTRAIN_LOG_ERROR("Null TCP client for send bytes recording stream checking\n");

    assert(false);

    return false;
}

void tcp_client_init_recording_streams(const tcp_client* _tcp_client, FILE* _receive_stream, FILE* _send_stream)
{
    if (_tcp_client)
    {
        tcp_client_init_receive_recording_stream(_tcp_client, _receive_stream);
        tcp_client_init_send_recording_stream(_tcp_client, _send_stream);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for bytes recording stream initialization\n");

        assert(false);
    }
}

void tcp_client_init_recording_files(const tcp_client* _tcp_client, const char* _receive_file_name,
    const char* _send_file_name)
{
    if (_tcp_client)
    {
        tcp_client_init_receive_recording_file(_tcp_client, _receive_file_name);
        tcp_client_init_send_recording_file(_tcp_client, _send_file_name);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for bytes recording file initialization\n");

        assert(false);
    }
}

void tcp_client_close_recording_streams(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        tcp_client_close_receive_recording_stream(_tcp_client);
        tcp_client_close_send_recording_stream(_tcp_client);
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null TCP client for bytes recording stream closing\n");

        assert(false);
    }
}

bool tcp_client_recording_enabled(const tcp_client* _tcp_client)
{
    if (_tcp_client)
    {
        return tcp_client_receive_recording_enabled(_tcp_client) || tcp_client_send_recording_enabled(_tcp_client);
    }

    MICROSTRAIN_LOG_ERROR("Null TCP client for bytes recording stream checking\n");

    assert(false);

    return false;
}

#ifdef __cplusplus
} // namespace C
} // namespace microstrain
#endif // __cplusplus
