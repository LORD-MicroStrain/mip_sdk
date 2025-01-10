#include "mip_interface.hpp"

#include <microstrain/connections/connection.hpp>

using microstrain::Connection;
using microstrain::Span;


namespace mip
{
    ////////////////////////////////////////////////////////////////////////////////
    ///@fn Connection::sendToDevice
    ///
    ///@brief Sends bytes to the device
    ///
    ///@param data   The data to send to the device
    ///
    ///@param length Length of data in bytes
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    ///@fn Connection::recvFromDevice
    ///
    ///@brief Receives bytes from the device
    ///
    ///@param buffer        Buffer to store the received data in
    ///
    ///@param max_length    Max number of bytes that can be read. Should be at most
    ///                     the length of buffer.
    ///
    ///@param wait_time_ms  Time to wait for data in milliseconds.
    ///
    ///@param length_out    Number of bytes actually read.
    ///
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
    void connect_interface(mip::Interface& device, Connection& conn)
    {
        device.setUserPointer(&conn);

        device.setSendFunctionUserPointer<Connection, &Connection::sendToDevice>();
        device.setRecvFunctionUserPointer<Connection, &recv_from_connection>();
        device.setUpdateFunction(C::mip_interface_default_update);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Adapts microstrain::Connection::recvFromDeviceSpan to a signature
    ///       compatible with mip interface receive callbacks.
    ///
    bool recv_from_connection(Connection* conn, Span<uint8_t> buffer, Timeout timeout, bool /*from_cmd*/, size_t* length_out, Timestamp* timestamp_out)
    {
        return conn->recvFromDeviceSpan(buffer, (unsigned int)timeout, length_out, timestamp_out);
    }

} // namespace mip
