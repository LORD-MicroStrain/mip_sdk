
#include "mip_interface.hpp"

#include <microstrain/connections/connection.hpp>


namespace mip {

////////////////////////////////////////////////////////////////////////////////
///@fn Connection::sendToDevice
///
///@brief Sends bytes to the device
///
///@param data   The data to send to the device
///@param length Length of data in bytes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@fn Connection::recvFromDevice
///
///@brief Receives bytes from the device
///
///@param buffer        Buffer to store the received data in
///@param max_length    Max number of bytes that can be read. Should be at most the length of buffer.
///@param wait_time_ms  Time to wait for data in milliseconds.
///@param length_out    Number of bytes actually read.
///@param timestamp_out Timestamp of when the data was received
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@brief Sets up the mip interface callbacks to point at this object.
///
///@note This sets the interface's user pointer to this connection object.
///
///@param device Device to configure.
///@param conn   The connection to set
///
void connect_interface(mip::Interface& device, microstrain::Connection& conn)
{
    using microstrain::Connection;

    auto send = [](C::mip_interface* device, const uint8_t* data, size_t length)->bool
    {
        return static_cast<Connection*>(C::mip_interface_user_pointer(device))->sendToDevice(data, length);
    };
    auto recv = [](C::mip_interface* device, uint8_t* buffer, size_t max_length, C::mip_timeout wait_time, size_t* length_out, C::mip_timestamp* timestamp_out)->bool
    {
        return static_cast<Connection*>(C::mip_interface_user_pointer(device))->recvFromDevice(buffer, max_length, static_cast<unsigned int>(wait_time), length_out, timestamp_out);
    };

    C::mip_interface_set_user_pointer(&device, &conn);

    C::mip_interface_set_send_function(&device, send);
    C::mip_interface_set_recv_function(&device, recv);
}

} // namespace mip
