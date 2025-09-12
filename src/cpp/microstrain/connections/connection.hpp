#pragma once

#include <microstrain/embedded_time.hpp>
#include <microstrain/array_view.hpp>

#include <stddef.h>
#include <stdint.h>


namespace microstrain
{
    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Represents a type of connection.
    ///
    /// The following methods are pure virtual and must be implemented by a derived
    /// class:
    ///@li `bool sendToDevice(const uint8_t* data, size_t length)`
    ///@li `bool recvFromDevice(uint8_t* buffer, size_t maxLength, unsigned int
    ///     wait_time_ms, size_t* lengthOut, EmbeddedTimestamp* timestampOut)`
    ///
    class Connection
    {
    public:
        static constexpr const char* TYPE = "None";

        Connection() { mType = TYPE; }
        virtual ~Connection() {}

        virtual bool sendToDeviceRaw(const uint8_t* data, size_t length) = 0;
        virtual bool recvFromDeviceRaw(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out) = 0;

        bool sendToDevice(microstrain::ConstBufferView data) { return sendToDeviceRaw(data.data(), data.size()); }
        bool recvFromDevice(microstrain::BufferView buffer, unsigned int wait_time_ms, size_t* length_out, EmbeddedTimestamp* timestamp_out)
        {
            if (!recvFromDeviceRaw(buffer.data(), buffer.size(), wait_time_ms, length_out, timestamp_out))
                return false;

            return true;
        }
        bool recvFromDeviceAndUpdateBufferView(microstrain::BufferView& buffer, unsigned int wait_time_ms, EmbeddedTimestamp* timestamp_out)
        {
            size_t length = 0;
            if (!recvFromDeviceRaw(buffer.data(), buffer.size(), wait_time_ms, &length, timestamp_out))
                return false;

            buffer = buffer.first(length);
            return true;
        }

        virtual bool isConnected() const = 0;
        virtual bool connect() = 0;
        virtual bool disconnect() = 0;

        const char* type() const { return mType; }

        virtual const char* interfaceName() const = 0;
        virtual uint32_t    parameter()     const = 0;

    protected:
        const char* mType;
    };
} // namespace microstrain
