#pragma once

#include <exception>
#include <chrono>
#include <string>


namespace mip
{
using namespace std::literals;

class SocketError : public std::exception
{
public:
    SocketError(int number);
    SocketError(const char* msg);

    const char* what() const noexcept { return mMessage.c_str(); }

private:
    std::string mMessage;
};


class TcpSocket
{
public:
    using Timeout = std::chrono::milliseconds;

public:
    TcpSocket(Timeout timeout=3s);
    TcpSocket(const std::string& hostname, uint16_t port, Timeout timeout=3s);
    ~TcpSocket();

    void connect(const std::string& hostname, uint16_t port);
    void disconnect();
    void reset();

    void send(const uint8_t* data, size_t length);
    int  recv(uint8_t* buffer, size_t maxLength);

protected:
    Timeout mTimeout;
    int mSocket = -1;
};


} // namespace mip
