#pragma once

#include "microstrain/connections/connection.hpp"

#include <iostream>
#include <memory>
#include <ostream>

namespace microstrain
{
    namespace connections
    {
        ////////////////////////////////////////////////////////////////////////////////
        ///@addtogroup microstrain_extras Extra utilities
        ///@{

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Can be used with another connection to communicate with a device, and
        ///       record the data at the same time
        ///
        class RecordingConnection : public Connection
        {
        public:
            static constexpr const char* TYPE = "Recording";

            RecordingConnection(Connection* connection, std::ostream* recvStream = nullptr, std::ostream* sendStream = nullptr);
            ~RecordingConnection() override = default;

            bool sendToDevice(const uint8_t* data, size_t length) override;
            bool recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out) override;

            bool isConnected() const override
            {
                if (mConnection)
                    return mConnection->isConnected();

                return false;
            };

            bool connect() override
            {
                if (mConnection)
                    return mConnection->connect();

                return false;
            };
            bool disconnect() override
            {
                if (mConnection)
                    return mConnection->disconnect();

                return false;
            };

            const char* interfaceName() const override { return mConnection->interfaceName(); }
            uint32_t parameter() const override { return mConnection->parameter(); }

            uint64_t recvFileBytesWritten() const
            {
                return mRecvFileWritten;
            }

            uint64_t sendFileBytesWritten() const
            {
                return mSendFileWritten;
            }

        protected:
            Connection* mConnection;

            // Files may be NULL to not record one direction or the other
            std::ostream* mRecvFile;
            std::ostream* mSendFile;

            uint64_t mRecvFileWritten = 0;
            uint64_t mSendFileWritten = 0;
        };

        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Template wrapper for a recording connection.
        ///
        ///@tparam ConnectionType The type of connection used to actually communicate.
        ///
        template<typename ConnectionType>
        class RecordingConnectionWrapper : public RecordingConnection
        {
        public:
            ////////////////////////////////////////////////////////////////////////////////
            ///@brief Creates a RecordingConnectionWrapper that will write received bytes to
            ///       recvStream, sent bytes to sendStream, and construct a connection
            ///       object from args
            ///
            ///@param recvStream The stream to write to when bytes are received. Null if
            ///                  received bytes should not be written to a stream
            ///
            ///@param sendStream The stream to write to when bytes are sent. Null if sent
            ///                  bytes should not be written to a stream
            ///
            ///@param args       Arguments required to construct the ConnectionType
            ///
            template<class... Args>
            RecordingConnectionWrapper(std::ostream* recvStream, std::ostream* sendStream, Args&&... args) :
                RecordingConnection(new ConnectionType(std::forward<Args>(args)...), recvStream, sendStream)
            {}

            ///@brief Deconstructs the RecordingConnectionWrapper as well as the underlying connection object made in the constructor
            ~RecordingConnectionWrapper() override { delete mConnection; }
        };

        ///@}
        ////////////////////////////////////////////////////////////////////////////////
    } // namespace connections
} // namespace microstrain
