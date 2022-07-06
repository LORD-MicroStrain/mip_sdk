
#include "tcp.hpp"

#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netdb.h>

#include <cstring>
#include <cstdio>


namespace mscl
{


SocketError::SocketError(int code) : mMessage(strerror(code))
{
}

SocketError::SocketError(const char* msg) : mMessage(msg)
{
}



TcpSocket::TcpSocket(Timeout timeout) : mTimeout(timeout)
{
}

TcpSocket::TcpSocket(const std::string& hostname, uint16_t port, Timeout timeout) : TcpSocket(timeout)
{
    connect(hostname, port);
}

TcpSocket::~TcpSocket()
{
    reset();
}

void TcpSocket::reset()
{
    if(mSocket != -1)
    {
        close(mSocket);
        mSocket = -1;
    }
}

void TcpSocket::disconnect()
{
    if(mSocket != -1)
        shutdown(mSocket, SHUT_RDWR);
}

void TcpSocket::connect(const std::string& hostname, uint16_t port)
{
    disconnect();

    // https://man7.org/linux/man-pages/man3/getaddrinfo.3.html

    addrinfo hints, *info;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET; // AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags    = 0;

    char port_str[6];  // Maximum 5 digits
    std::sprintf(port_str, "%d", port);

    if( int result = getaddrinfo(hostname.c_str(), port_str, &hints, &info) != 0 )
        throw SocketError(gai_strerror(result));

    for(addrinfo* addr=info; addr!=nullptr; addr=addr->ai_next)
    {
        mSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if( mSocket == -1 )
            continue;

        if( ::connect(mSocket, addr->ai_addr, addr->ai_addrlen) == 0 )
            break;

        close(mSocket);
        mSocket = -1;
    }

    int code = errno;
    freeaddrinfo(info);

    if( mSocket == -1 )
        throw SocketError(code);

    timeval timeout_option;
    timeout_option.tv_sec  = mTimeout.count() / 1000;
    timeout_option.tv_usec = (mTimeout.count() % 1000) * 1000;

    if( setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        throw SocketError(errno);

    if( setsockopt(mSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        throw SocketError(errno);

}

void TcpSocket::send(const uint8_t* data, size_t length)
{
    for(ssize_t total = 0; total<length; )
    {
        ssize_t sent = ::send(mSocket, data, length, MSG_NOSIGNAL);
        if(sent < 0)
            throw SocketError(errno);

        total += sent;
    }
}

int TcpSocket::recv(uint8_t* buffer, size_t maxLength)
{
    ssize_t count = ::recv(mSocket, buffer, maxLength, MSG_NOSIGNAL);

    if( count < 0 )
    {
        if(errno != EAGAIN && errno != EWOULDBLOCK)
            throw SocketError(errno);
        else
            return 0;
    }
    // Throw an error if the connection has been closed by the other side.
    else if( count == 0 )
        throw SocketError("The remote device has closed the connection");

    return count;
}

} // namespace mscl
