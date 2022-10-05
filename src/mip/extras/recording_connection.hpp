#pragma once

#include <mip/mip_device.hpp>

#include <memory>
#include <ostream>
#include <iostream>

namespace mip {
namespace extras {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_extras Extra utilities
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Can be used with another connection to communicate with a device, and record the data at the same time
///
class RecordingConnection : public Connection
{
public:
    RecordingConnection(Connection* connection, std::ostream* recvStream = nullptr, std::ostream* sendStream = nullptr);

    bool sendToDevice(const uint8_t* data, size_t length) final;
    bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* count_out, Timestamp* timestamp_out) final;

protected:
    Connection* mConnection;

    // Files may be NULL to not record one direction or the other
    std::ostream* mRecvFile;
    std::ostream* mSendFile;
};

////////////////////////////////////////////////////////////////////////////////
///@brief Template wrapper for a recording connection.
///
///@param ConnectionType The type of connection used to actually communicate.
///
template<typename ConnectionType>
class RecordingConnectionWrapper : public RecordingConnection
{
public:
    ///@brief Creates a RecordingConnectionWrapper that will write received bytes to recvStream, sent bytes to sendStream, and construct a connection object from args
    ///
    ///@param recvStream The stream to write to when bytes are received. Null if received bytes should not be written to a stream
    ///@param sendStream The stream to write to when bytes are sent. Null if sent bytes should not be written to a stream
    ///@param args       Arguments required to construct the ConnectionType
    template<class... Args>
    RecordingConnectionWrapper(std::ostream* recvStream, std::ostream* sendStream, Args&&... args) :
        RecordingConnection(new ConnectionType(std::forward<Args>(args)...), recvStream, sendStream) {}

    ///@brief Deconstructs the RecordingConnectionWrapper as well as the underlying connection object made in the constructor
    ~RecordingConnectionWrapper() { delete mConnection; }
};

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace extras
} // namespace mip
