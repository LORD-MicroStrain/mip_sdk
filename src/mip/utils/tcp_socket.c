
#include "tcp_socket.h"

#ifdef WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>

#include "../mip_loggging.h"

#else

#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>

#define INVALID_SOCKET -1

#endif

static bool tcp_socket_open_common(tcp_socket* socket_ptr, const char* hostname, uint16_t port, unsigned int timeout_ms)
{
    socket_ptr->handle = INVALID_SOCKET;

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
        if( socket_ptr->handle == INVALID_SOCKET )
            continue;

        if( connect(socket_ptr->handle, addr->ai_addr, addr->ai_addrlen) == 0 )
            break;

        close(socket_ptr->handle);
        socket_ptr->handle = INVALID_SOCKET;
    }

    freeaddrinfo(info);

    if( socket_ptr->handle == INVALID_SOCKET )
        return false;

    struct timeval timeout_option;
    timeout_option.tv_sec  = timeout_ms / 1000;
    timeout_option.tv_usec = (timeout_ms % 1000) * 1000;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_RCVTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        return false;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_SNDTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        return false;

    return true;
}

bool tcp_socket_open(tcp_socket* socket_ptr, const char* hostname, uint16_t port, unsigned int timeout_ms)
{
#ifdef WIN32

    // Initialize winsock for each connection since there's no global init function.
    // This is safe to do multiple times, as long as it's shutdown the same number of times.
    WSAData wsaData;
    int result = WSAStartup(MAKEWORD(2,2), &wsaData);
    if(result != 0)
    {
        MIP_LOG_ERROR("WSAStartup() failed: %d\n", result);
        return false;
    }

//    struct addrinfo* address = NULL;
//    struct addrinfo* ptr = NULL;
//    struct addrinfo hints;
//
//    ZeroMemory(&hints, sizeof(hints));
//    hints.ai_family = AF_UNSPEC;
//    hints.ai_socktype = SOCK_STREAM;
//    hints.ai_protocol = IPPROTO_TCP;
//
//    result = getaddrinfo(hostname, port, &hints, &address);
//    if(result != 0)
//    {
//        MIP_LOG_WARNING("getaddrinfo() failed for hostname=%s, port=%d: %d\n", hostname, port, result);
//        WSACleanup();
//        return false;
//    }
//
//    *socket_ptr = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
//    if(*socket_ptr == INVALID_SOCKET)
//    {
//        MIP_LOG_WARNING("socket() failed for hostname=%s, port=%d: %d\n", hostname, port, WSAGetLastError());
//        freeaddrinfo(address);
//        WSACleanup();
//        return false;
//    }
//
//
//
//    result = connect(*socket_ptr, address->ai_addr, (int)address->ai_addrlen);
//
//    freeaddrinfo(address);
//
//    if(result == SOCKET_ERROR)
//    {
//        closesocket(*socket_ptr);
//        *socket_ptr = INVALID_SOCKET;
//        WSACleanup();
//        return false;
//    }
//
//    return true;

#endif

    return tcp_socket_open_common(socket_ptr, hostname, port ,timeout_ms);
}

bool tcp_socket_close(tcp_socket* socket_ptr)
{
    if( socket_ptr->handle == INVALID_SOCKET )
        return false;

#ifdef WIN32
    closesocket(socket_ptr->handle);
    WSACleanup(); // See tcp_socket_open
#else
    close(socket_ptr->handle);
#endif

    socket_ptr->handle = INVALID_SOCKET;
    return true;
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
