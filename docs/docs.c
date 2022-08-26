////////////////////////////////////////////////////////////////////////////////
///@mainpage MIP SDK
///
///@section Intro
///
///
///@ref mip_interface - A high-level interface to an abstract MIP device.
///
///@section quickref_cpp Quick Reference [C++]
///
///@li @ref mip::DeviceInterface Top-level MIP interface class.
///@li @ref mip::Packet          A class representing a MIP packet for either transmission or reception.
///@li @ref mip::Field           A class representing a MIP field within a packet.
///@li @ref mip::Parser          MIP parser class for converting received bytes into packets.
///
///@section quickref_c Quick Reference [C]
///
///@ref mip_interface Mip Interface
///
///
///
////////////////////////////////////////////////////////////////////////////////
///@page mip_interface Mip Interface
////////////////////////////////////////////////////////////////////////////////
///
/// The MIP interface is a high-level abstraction of a physical device which
/// communicates using the MIP protocol. It provides both data callbacks and
/// command functions for controlling and configuring the device:
///
///@image html mip_interface.svg
///
/// When an application calls one of the command functions, the MIP interface
/// creates a packet, sends it to the device, and waits for a reply. When the
/// reply is received, the command function returns the reply code to the
/// application. If no reply is received, or if an error occurs, the function
/// will return a status code.
///
/// Sending and receiving to or from the device occurs via two functions:
///@li mip_interface_user_send_to_device() for transmission and
///@li mip_interface_user_recv_from_device() for reception.
///
/// The application must define these two C functions, or subclass
/// mip::DeviceInterface and implement the pure virtual functions.
/// This should be straightforward; simply pass the bytes between the MIP
/// interface and the connection.
///
/// Because the device transmits continuously when streaming data, the
/// application must poll the connection for new data frequently. This is
/// done via the @ref update "update function".
///
/// An application obtains sensor data via the
/// @ref mip_dispatch "dispatch subsystem". There are 3 ways to do so:
///@li Packet callbacks, which call a function when a packet matching the
///    MIP descriptor set is received,
///@li Field callbacks, which call a function when a MIP field matching the
///    descriptor set and field descriptor is called, and
///@li Data pointers, which point to a data structure in memory and update it
///    when the associated data is received.
///
///
////////////////////////////////////////////////////////////////////////////////
///@section mip_commands Sending Commands
///
/// Typically an application will configure the device, initialize some
/// settings, and start streaming. To do so, it must send commands. In most
/// cases, applications will call a single function for each needed command.
/// These functions take the command parameters as arguments, send the packet,
/// wait for the reply, and return a result code. When reading from the device,
/// these commands will also report the device response information, assuming
/// the command was successful. The command functions are blocking, that is,
/// they halt execution until the device responds or the command times out.
///
///
////////////////////////////////////////////////////////////////////////////////
///@section mip_dispatch The Dispatch System
///
/// Because of the limited resources on embedded platforms, the MIP SDK will not
/// buffer received data, and instead requires the application to process data
/// as it arrives. The MIP interface will dispatch callbacks to the application
/// when the requested data is received.
///
/// The MIP interface can dispatch data in 3 ways:
///@li Packet callbacks, which call a function with a mip packet,
///@li Field callbacks, which call a function with a single mip field, and
///@li Data pointers, which are updated with data from a single mip field.
///
/// With the first two options, the callback function will receive a handle to
/// the MIP interface, the associated MIP packet or field, and the reception
/// timestamp.
///
/// An application must register callbacks with the system during
/// initialization. Each method requires a pointer and the MIP descriptor
/// associated with the data of interest. There is no limit on the number
/// of registered dispatchers, though performance may be affected by using
/// too many. Multiple dispatchers may be registered to the same data.
///
///@par Packet callbacks
/// Packet callbacks are invoked when a packet is received which matches the
/// registered descriptor set. The descriptor set may also be a wildcard,
/// allowing the application to process any type of packet.
///
/// An application can register a packet callback to occur either before or
/// after the field callbacks for the data in the same packet. For example,
/// to print a summary of the packet before displaying information about each
/// field, an application would set the callback to occur first. Usually
/// applications will set a packet callback to occur last, so that they can
/// determine if all of the fields have been processed.
///
///@par Field callbacks
/// Similar to packet callbacks, field callbacks are invoked when a MIP
/// field is received which matches the specified descriptor set and
/// field descriptor. Either descriptor may be a wildcard.
///
///@par Data pointers
/// Data pointer dispatchers can alleviate a lot of boilerplate code having to
/// do with deserializing a MIP field and storing it somewhere. An application
/// can register a pointer one of the MIP data structures, along with the
/// associated descriptors, and have it automatically updated. The descriptors
/// cannot be wildcards because the type of the data structure is fixed.
///
///@par Data callbacks
/// Thanks to the power of templates, one additional dispatch mechanism is
/// available for C++ applications. A data callback is similar to a field
/// callback except that instead of getting the raw MIP field data, the function
/// is passed the fully-deserialized data structure.
///
/// Typically an application will register a series of data or field callbacks
/// and write the data to some kind of data structure. Because the order of
/// these callbacks depends on the device configuration, it can be difficult
/// to know which fields belong together in one sample. The solution is to use
/// a packet callback after all of the fields are received. In the case of
/// wraparound "overflow" MIP packets (see the MIP documentation), packets
/// containing a shared timestamp or event source field at the beginning can
/// be used to group data together.
///
///
////////////////////////////////////////////////////////////////////////////////
///@section update The update function
///
/// The application should call mip_interface_update() periodically to process
/// data sent by the device. This update function will call
/// mip_interface_user_recv_from_device() to parse packets. When a data packet is
/// received, the list of packet and data callbacks is checked, and any
/// matching callbacks are invoked. The update function should be called at
/// a high enough rate to avoid overflowing the connection buffers. The
/// precision of the reception timestamp is dependent on the update rate.
///
/// The command functions in @ref MipCommands (e.g. mip_write_message_format() / writeMessageFormat())
/// will block execution until the command completes. Either the device will
/// respond with an ack/nack code, or the command will time out. During this
/// time, the system must be able to receive data from the device in order for
/// command replies to be detected. This occurs via the mip_interface_update()
/// function as well.
///
///@par Single-threaded applications
///
/// For single-threaded applications, data can be read from the port directly
/// from within the command function. While the command is waiting (status code
/// MIP_STATUS_WAITING / CmdResult::STATUS_WAITING), repeated calls to the
/// update function will be made. By default, the update function calls
/// mip_interface_user_recv_from_device(). Because the function is called from
/// within a loop, it should sleep for a short time to wait for data if none
/// has been received yet. Doing so prevents excessive CPU usage and lowers
/// power consumption.
///
/// The following diagram shows the typical control flow for a single-threaded
/// application. First, the device is configured by setting the message format.
/// Execution flows down into the command processing functions until
/// mip_interface_wait_for_reply() is called. This will repeatedly call
/// mip_interface_update() to pump packets from the device through the system,
/// until either an ack/nack is received or the command times out.
/// Once the device acknowledges the command, control is returned to the
/// application which then registers some data or packet callbacks. It finally
/// goes into a loop in collect_data(). Inside this loop, the update function
/// is called to process data packets.
///
/// Notice that the same update function is called from both the command
/// function and the data collection loop. If any data packets are received
/// while waiting for a command reply, associated callbacks may be executed.
/// This is why this example application registers its callbacks after the
/// format is configured properly.
///
///@image html device_update.svg
///
///@par Multi-threaded applications
///
/// For some applications, it may be desirable to run all of the data collection
/// from a separate thread. In this case, the command functions must not
/// call the update function as that would cause a race condition between the
/// command thread and the data thread. Instead, the command thread should
/// simply sleep or yield and let the data thread process the ack/nack packet.
///
/// To allow this behavior, the update function takes a boolean parameter which
/// is true when waiting on a command and false when processing data. The
/// default update function, mip_interface_default_update(), ignores this flag,
/// but applications may override it via mip_interface_set_update_function(). In
/// this case, a wrapper function can be created which implements the above
/// behavior:
///@code{.c}
/// bool user_update_function(struct mip_device* device, bool blocking)
/// {
///     // If called from the data thread, do the normal processing.
///     if( !blocking )
///         return mip_interface_default_update(device, blocking);
///
///     // Otherwise, sleep and let the data thread process the reply.
///     std::this_thread::sleep_for(std::chrono::milliseconds(10));
///     return true;
/// }
///
/// mip_interface_set_update_function(device, &user_update_function);
///@endcode
///
///@image html device_update_threaded.svg
///
/// See the threading demo for an example application.
///
///@par Other thread-safety concerns
///
///@li Data transmission to the device (for sending commands) is thread-safe
///    within the MIP SDK. If multiple threads will send to the device, the
///    application should ensure that mip_interface_user_send_to_device() is
///    thread-safe (e.g. by using a mutex).
///
///@li It is up to the application to ensure that sending and receiving from
///    separate threads is safe. This is true for the built-in serial and TCP
///    connections on most operating systems.
///
///@par Using a custom update function for other purposes
///
/// An alternate update function may be used for single-threaded
/// applications, too:
///@li To update a progress bar while waiting for commands to complete
///@li To process data from other devices
///@li To avoid blocking inside mip_interface_user_recv_from_device() when
///    called from a data processing loop.
///@li To push data through the system in a different way (e.g. without using
///    mip_interface_user_recv_from_device())
///
/// Data may be pushed into the system by calling any of these functions:
///@li mip_interface_default_update() - this is the default behavior.
///@li mip_interface_receive_bytes() - process bytes, given a buffer.
///@li mip_interface_receive_packet() - process pre-parsed packets.
///@li mip_interface_process_unparsed_packets() - continue parsing buffered data.
///