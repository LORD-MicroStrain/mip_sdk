#pragma once

#include "microstrain/common/platform.h"

#include <stdint.h>
#include <stddef.h>

#if __cpp_lib_span >= 202002L
#include <span>
#endif


namespace microstrain
{

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a type of connection to a MIP device.
///
/// The following methods are pure virtual and must be implemented by a derived
/// class. These functions map to the corresponding C functions.
///@li `bool sendToDevice(const uint8_t* data, size_t length)` - corresponds to mip_interface_user_send_to_device.
///@li `bool recvFromDevice(uint8_t* buffer, size_t maxLength, size_t* lengthOut, Timestamp* timestampOut)` - corresponds to mip_interface_user_recv_from_device.
///
class Connection
{
public:

    static constexpr auto TYPE = "None";

    Connection() { mType = TYPE; };
    virtual ~Connection() {}

    virtual bool sendToDevice(const uint8_t* data, size_t length) = 0;
    virtual bool recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out) = 0;

#if __cpp_lib_span >= 202002L
    bool sendToDevice(std::span<const uint8_t> data) { return sendToDevice(data.data(), data.size()); }
    bool recvFromDevice(std::span<uint8_t*>& buffer, unsigned int wait_time_ms) {
        size_t length = 0;
        if(!recvFromDevice(buffer.data(), buffer.size(), wait_time_ms, &length))
            return false;
        buffer = buffer.first(length);
        return true;
    }
#endif

    virtual bool isConnected() const = 0;
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;

    const char* type() const { return mType; };

    virtual const char* interfaceName() const = 0;
    virtual uint32_t parameter() const = 0;

protected:

    const char *mType;

};


} // namespace microstrain
