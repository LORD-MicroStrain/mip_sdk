#include "recording_connection.hpp"

namespace mip
{
namespace extras
{

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