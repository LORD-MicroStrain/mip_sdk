
#include "tcp_socket.h"

#ifdef WIN32
#else
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#endif

bool tcp_socket_open(tcp_socket* socket_ptr, const char* hostname, uint16_t port, size_t timeout_ms)
{
#ifdef WIN32
    return false;  // TODO: Windows
#else
    // https://man7.org/linux/man-pages/man3/getaddrinfo.3.html
    struct addrinfo hints, *info;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET; // AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags    = 0;

    char port_str[6];  // Maximum 5 digits
    sprintf(port_str, "%d", port);

    int result = getaddrinfo(hostname, port_str, &hints, &info);
    if( result != 0 )
        return false;

    for(struct addrinfo* addr=info; addr!=NULL; addr=addr->ai_next)
    {
        socket_ptr->handle = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if( socket_ptr->handle == -1 )
            continue;

        if( connect(socket_ptr->handle, addr->ai_addr, addr->ai_addrlen) == 0 )
            break;

        close(socket_ptr->handle);
        socket_ptr->handle = -1;
    }

    freeaddrinfo(info);

    if( socket_ptr->handle == -1 )
        return false;

    struct timeval timeout_option;
    timeout_option.tv_sec  = timeout_ms / 1000;
    timeout_option.tv_usec = (timeout_ms % 1000) * 1000;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_RCVTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        return false;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_SNDTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        return false;

    return true;
#endif
}

bool tcp_socket_close(tcp_socket* socket_ptr)
{
#ifdef WIN32
    return false;  // TODO: Windows
#else
    if( socket_ptr->handle != -1 )
    {
        close(socket_ptr->handle);
        socket_ptr->handle = -1;
        return true;
    }
    else
        return false;
#endif
}

bool tcp_socket_send(tcp_socket* socket_ptr, const void* buffer, size_t num_bytes, size_t* bytes_written)
{
#ifdef WIN32
    return false;  // TODO: Windows
#else
    for(*bytes_written = 0; *bytes_written < num_bytes; )
    {
        ssize_t sent = send(socket_ptr->handle, buffer, num_bytes, MSG_NOSIGNAL);
        if(sent < 0)
            return false;

        *bytes_written += sent;
    }
    return true;
#endif
}

bool tcp_socket_recv(tcp_socket* socket_ptr, void* buffer, size_t num_bytes, size_t* bytes_read)
{
#ifdef WIN32
    return false;  // TODO: Windows
#else
    ssize_t local_bytes_read = recv(socket_ptr->handle, buffer, num_bytes, MSG_NOSIGNAL);

    if( local_bytes_read == -1 )
    {
        if(errno != EAGAIN && errno != EWOULDBLOCK)
            return false;
        else
            return true;
    }
    // Throw an error if the connection has been closed by the other side.
    else if( local_bytes_read == 0 )
        return false;

    *bytes_read = local_bytes_read;
    return true;
#endif
}
