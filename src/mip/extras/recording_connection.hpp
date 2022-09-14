#pragma once

#include <mip/mip_device.hpp>

#include <memory>
#include <ostream>
#include <iostream>

namespace mip
{
namespace extras
{

class RecordingConnection : public Connection
{
public:
    RecordingConnection(std::ostream* recvFile, std::ostream* sendFile, Connection* connection);

    bool sendToDevice(const uint8_t* data, size_t length) final;
    bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* count_out, Timestamp* timestamp_out) final;

protected:
    // Files may be NULL to not record one direction or the other
    std::ostream* mRecvFile;
    std::ostream* mSendFile;

    Connection* mConnection;
};

////////////////////////////////////////////////////////////////////////////////
///@brief Template wrapper for a recording connection.
///
///@param ConnectionType The type of connection used to actually communicate.
template<typename ConnectionType>
class RecordingConnectionWrapper : public RecordingConnection
{
public:
    ///@brief Creates a RecordingConnectionWrapper that will write received bytes to recvFile, and sent bytes to sendFile, and construct a connection object from args
    ///
    ///@param recvFile The file to write to when bytes are received
    ///@param sendFile The file to write to when bytes are sent
    ///@param args     Arguments required to construct the ConnectionType
    template<typename... Args>
    RecordingConnectionWrapper(std::ostream* recvFile, std::ostream* sendFile, Args&&... args) :
        mConnectionPtr(std::unique_ptr<ConnectionType>(new ConnectionType(std::forward<Args>(args)...))), RecordingConnection(recvFile, sendFile, nullptr)
    {
        mConnection = mConnectionPtr.get();
    }
private:
    std::unique_ptr<ConnectionType> mConnectionPtr;
};

}  // namespace extras
}  // namespace mip
