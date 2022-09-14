#include "recording_connection.hpp"

namespace mip
{
namespace extras
{

///@brief Creates a RecordingConnection that will write received bytes to recvFile, and sent bytes to sendFile
///
///@param recvFile  The file to write to when bytes are received. Null if received bytes should not be written to a file
///@param sendFile  The file to write to when bytes are sent. Null if sent bytes should not be written to a file
///@param connection A connection class that will actually communicate with the device
RecordingConnection::RecordingConnection(std::ostream* recvFile, std::ostream* sendFile, Connection* connection) :
    mRecvFile(recvFile), mSendFile(sendFile), mConnection(connection)
{
}

bool RecordingConnection::sendToDevice(const uint8_t* data, size_t length) {
    const bool ok = mConnection->sendToDevice(data, length);
    if( ok && mSendFile != nullptr )
        mSendFile->write(reinterpret_cast<const char*>(data), length);
    return ok;
}

bool RecordingConnection::recvFromDevice(uint8_t* buffer, size_t max_length, size_t* count_out, Timestamp* timestamp_out) {
    const bool ok = mConnection->recvFromDevice(buffer, max_length, count_out, timestamp_out);
    if( ok && mRecvFile != nullptr )
        mRecvFile->write(reinterpret_cast<char*>(buffer), *count_out);
    return ok;
}

}  // namespace extras
}  // namespace mip