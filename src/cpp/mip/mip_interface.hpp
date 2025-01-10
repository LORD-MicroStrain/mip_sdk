#pragma once

#include "mip_cmdqueue.hpp"
#include "mip_parser.hpp"

#include <mip/mip_descriptors.h>
#include <mip/mip_interface.h>

#include <assert.h>
#include <string>

namespace microstrain
{
    class Connection;
} // namespace microstrain

namespace mip
{
    ////////////////////////////////////////////////////////////////////////////////
    ///@addtogroup mip_cpp
    ///@{

    class Interface;

    void connect_interface(Interface& dev, microstrain::Connection& conn);

    using DispatchHandler = C::mip_dispatch_handler;

    struct Dispatcher : public C::mip_dispatcher
    {
        enum : uint8_t {
            ANY_DATA_SET = C::MIP_DISPATCH_ANY_DATA_SET,
            ANY_DESCRIPTOR = C::MIP_DISPATCH_ANY_DESCRIPTOR,
        };

        void addHandler(DispatchHandler& handler) { C::mip_dispatcher_add_handler(this, &handler); }
        void removeHandler(DispatchHandler& handler) { C::mip_dispatcher_remove_handler(this, &handler); }

        void removeAllHandlers() { C::mip_dispatcher_remove_all_handlers(this); }
    };

    template<class Cmd> CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, Timeout additionalTime=0);
    template<class Cmd> CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, typename Cmd::Response& response, Timeout additionalTime=0);
    template<class Cmd, class... Args> CmdResult runCommand(C::mip_interface& device, const Args&&... args, Timeout additionalTime);
    template<class Cmd> bool startCommand(C::mip_interface& device, C::mip_pending_cmd& pending, const Cmd& cmd, Timeout additionalTime);

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Represents a connected MIP device.
    ///
    class Interface : public C::mip_interface
    {
    public:
        //
        // Constructors
        //

        ////////////////////////////////////////////////////////////////////////////////
        ///@copydoc mip::C::mip_interface_init
        ///
        /// The interface callbacks must be assigned separately (e.g. with
        /// Connection::connect_interface())
        Interface(Timeout parseTimeout, Timeout baseReplyTimeout)
        {
            C::mip_interface_init(this, parseTimeout, baseReplyTimeout, nullptr, nullptr, &C::mip_interface_default_update, nullptr);
        }

        ////////////////////////////////////////////////////////////////////////////////
        ///@copydoc mip::C::mip_interface_init
        ///
        ///@param connection The connection object used to communicate with the device.
        ///                  This object must exist for the life of the DeviceInterface
        ///                  object
        Interface(microstrain::Connection* connection, Timeout parseTimeout, Timeout baseReplyTimeout) :
            Interface(parseTimeout, baseReplyTimeout)
        {
            if (connection)
                connect_interface(*this, *connection);
        }

        Interface(const Interface&) = delete;
        Interface& operator=(const Interface&) = delete;

        ~Interface() = default;

        //
        // Callback functions
        //

        // C function callbacks

        C::mip_send_callback   sendFunction()   const { return C::mip_interface_send_function(this);   }
        C::mip_recv_callback   recvFunction()   const { return C::mip_interface_recv_function(this);   }
        C::mip_update_callback updateFunction() const { return C::mip_interface_update_function(this); }

        void setSendFunction  (C::mip_send_callback   callback) { C::mip_interface_set_send_function  (this, callback); }
        void setRecvFunction  (C::mip_recv_callback   callback) { C::mip_interface_set_recv_function  (this, callback); }
        void setUpdateFunction(C::mip_update_callback function) { C::mip_interface_set_update_function(this, function); }

        // free/nonmember function callbacks

        template<bool (*Send)(Interface&, const uint8_t*, size_t)>
        void setSendFunction();
        template<bool (*Send)(Interface&, microstrain::Span<const uint8_t>)>
        void setSendFunction();

        template<bool (*Recv)(Interface&, Timeout, bool, Timestamp*)>
        void setRecvFunction();

        template<bool (*Update)(Interface&, Timeout, bool)>
        void setUpdateFunction();

        // derived member function callbacks

        template<class Derived, bool (Derived::*Send)(const uint8_t*, size_t)>
        void setSendFunction();
        template<class Derived, bool (Derived::*Send)(microstrain::Span<const uint8_t*>)>
        void setSendFunction();

        template<class Derived, bool (Derived::*Recv)(Timeout, bool, Timestamp*)>
        void setRecvFunction();

        template<class Derived, bool (Derived::*Update)(Timeout, bool)>
        void setUpdateFunction();

        // Separate class object callbacks

        template<
            class T,
            bool (T::*Send)(const uint8_t*, size_t),
            bool (T::*Recv)(Timeout, bool, Timestamp*),
            bool (T::*Update)(Timeout, bool) = nullptr
        >
        void setCallbacks(T* object);

        //
        // General accessors
        //

        ///@brief Sets an optional user data pointer which can be retrieved later.
        ///
        ///@warning If using connect_interface(), setCallbacks(), or constructing using a microstrain::Connection object,
        ///         the user pointer must point at the connection instance and must not be changed.
        ///
        void setUserPointer(void* ptr) { C::mip_interface_set_user_pointer(this, ptr); }
        void* userPointer() { return C::mip_interface_user_pointer(this); }

        Timeout baseReplyTimeout() const          { return C::mip_cmd_queue_base_reply_timeout(&cmdQueue()); }
        void setBaseReplyTimeout(Timeout timeout) { C::mip_cmd_queue_set_base_reply_timeout(&cmdQueue(), timeout); }

        Parser&   parser()   { return *static_cast<Parser*>(C::mip_interface_parser(this)); }
        CmdQueue& cmdQueue() { return *static_cast<CmdQueue*>(C::mip_interface_cmd_queue(this)); }

        const Parser&   parser()   const   { return const_cast<Interface*>(this)->parser(); }
        const CmdQueue& cmdQueue() const { return const_cast<Interface*>(this)->cmdQueue(); }

        //
        // Communications
        //

        bool sendToDevice(const uint8_t* data, size_t length) { return C::mip_interface_send_to_device(this, data, length); }
        bool sendToDevice(const C::mip_packet_view& packet) { return sendToDevice(C::mip_packet_pointer(&packet), C::mip_packet_total_length(&packet)); }
        bool recvFromDevice(Timeout wait_time, size_t* length_out, Timestamp* timestamp_out) { return C::mip_interface_recv_from_device(this, wait_time, length_out, timestamp_out); }
        bool update(Timeout wait_time = 0, bool from_cmd = false) { return C::mip_interface_update(this, wait_time, from_cmd); }

        bool defaultUpdate(Timeout wait_time = 0, bool from_cmd = false) { return C::mip_interface_default_update(this, wait_time, from_cmd); }
        void inputBytes(const uint8_t* data, size_t length, Timestamp timestamp) { C::mip_interface_input_bytes(this, data, length, timestamp); }
        void inputPacket(const C::mip_packet_view& packet, Timestamp timestamp) { C::mip_interface_input_packet(this, &packet, timestamp); }


        CmdResult waitForReply(C::mip_pending_cmd& cmd) { return C::mip_interface_wait_for_reply(this, &cmd); }

        //
        // Data Callbacks
        //

        void registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, C::mip_dispatch_packet_callback callback, void* userData) { C::mip_interface_register_packet_callback(this, &handler, descriptorSet, afterFields, callback, userData); }
        void registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, C::mip_dispatch_field_callback callback, void* userData) { C::mip_interface_register_field_callback(this, &handler, descriptorSet, fieldDescriptor, callback, userData); }

        template<void (*Callback)(void*, const PacketView&, Timestamp)>
        void registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, void* userData=nullptr);

        template<class Object, void (Object::*Callback)(const PacketView&, Timestamp)>
        void registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, Object* object);

        template<void (*Callback)(void*, const FieldView&, Timestamp)>
        void registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, void* userData=nullptr);

        template<class Object, void (Object::*Callback)(const FieldView& field, Timestamp)>
        void registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, Object* object);

        template<class DataField, void (*Callback)(void*, const DataField&, Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, void* userData=nullptr, uint8_t descriptorSet=DataField::DESCRIPTOR_SET);

        template<class DataField, void (*Callback)(void*, const DataField&, uint8_t, Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, void* userData=nullptr, uint8_t descriptorSet=DataField::DESCRIPTOR_SET);

        template<class DataField, class Object, void (Object::*Callback)(const DataField&, Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, Object* object, uint8_t descriptorSet=DataField::DESCRIPTOR_SET);

        template<class DataField, class Object, void (Object::*Callback)(const DataField&, uint8_t, Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, Object* object, uint8_t descriptorSet=DataField::DESCRIPTOR_SET);

        template<class DataField>
        void registerExtractor(C::mip_dispatch_handler& handler, DataField* field, uint8_t descriptorSet=DataField::DESCRIPTOR_SET);

        //
        // Run function templates
        //

        template<class Cmd>
        CmdResult runCommand(const Cmd& cmd, Timeout additionalTime=0) { return mip::runCommand(*this, cmd, additionalTime); }

        template<class Cmd, class... Args>
        CmdResult runCommand(Args&&... args, Timeout additionalTime=0) { return mip::runCommand(*this, std::forward<Args>(args)..., additionalTime); }

        template<class Cmd>
        CmdResult runCommand(const Cmd& cmd, typename Cmd::Response& response, Timeout additionalTime=0) { return mip::runCommand(*this, cmd, response, additionalTime); }

        template<class Cmd>
        bool startCommand(PendingCmd& pending, const Cmd& cmd, Timeout additionalTime=0) { return mip::startCommand(*this, pending, cmd, additionalTime); }

    //    template<class Cmd>
    //    bool startCommand(PendingCmd& pending, const Cmd& cmd, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime=0) { return startCommand(pending, cmd, responseBuffer, responseBufferSize, additionalTime); }
    };

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (free function version).
    ///
    ///@tparam Send A compile-time pointer to the callback function.
    ///
    template<bool (*Send)(Interface&, const uint8_t*, size_t)>
    void Interface::setSendFunction()
    {
        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)->bool
        {
            return (*Send)(*static_cast<Interface*>(device), data, length);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (free function /w span version).
    ///
    ///@tparam Send A compile-time pointer to the callback function.
    ///
    template<bool (*Send)(Interface&, microstrain::Span<const uint8_t>)>
    void Interface::setSendFunction()
    {
        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)->bool
        {
            return (*Send)(*static_cast<Interface*>(device), {data, length});
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (free function version).
    ///
    ///@tparam Recv A compile-time pointer to the callback function.
    ///
    template<bool (*Recv)(Interface&, Timeout, bool, Timestamp*)>
    void Interface::setRecvFunction()
    {
        setRecvFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd, C::mip_timestamp* timestamp_out)->bool
        {
            return (*Recv)(*static_cast<Interface*>(device), wait_time, from_cmd, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the update callback function (free function version).
    ///
    ///@tparam Update A compile-time pointer to the callback function.
    ///
    template<bool (*Update)(Interface&, Timeout, bool)>
    void Interface::setUpdateFunction()
    {
        setUpdateFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)->bool
        {
            return (*Update)(*static_cast<Interface*>(device), wait_time, from_cmd);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (derived member function version).
    ///
    ///@tparam Derived Derived class type. Must inherit from Interface.
    ///
    ///@tparam Send    Compile-time pointer to member function of Derived.
    ///
    ///@code{.cpp}
    /// class MyClass : public mip::Interface
    /// {
    ///     bool sendToDevice(const uint8_t* data, size_t size);
    ///     bool recvFromDevice(uint8_t* data, size_t max_length, Timeout wait_time, size_t* length_out, Timestamp* timestamp_out);
    /// };
    ///
    /// MyClass instance;
    /// instance.setSendFunction<MyClass, &MyClass::sendToDevice>();
    /// instance.setRecvFunction<MyClass, &MyClass::recvFromDevice>();
    ///
    /// instance.setCallbacks<Connection, &Connection::sendToDevice, &Connection::recvFromDevice>(connection);
    ///@endcode
    ///
    template<class Derived, bool (Derived::*Send)(const uint8_t*, size_t)>
    void Interface::setSendFunction()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must be derived from C::mip_interface.");

        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)
        {
            return (static_cast<Derived*>(device)->*Send)(data, length);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (derived member function version).
    ///
    ///@tparam Derived Derived class type. Must inherit from Interface.
    ///
    ///@tparam Recv    Compile-time pointer to member function of Derived.
    ///
    ///@see Interface::setSendFunction()
    ///
    template<class Derived, bool (Derived::*Recv)(Timeout, bool, Timestamp*)>
    void Interface::setRecvFunction()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must be derived from C::mip_interface.");

        setRecvFunction([](C::mip_interface* device, Timeout wait_time, bool from_cmd, Timestamp* timestamp_out)
        {
            return (static_cast<Derived*>(device)->*Recv)(wait_time, from_cmd, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the update callback function (derived member function version).
    ///
    ///@tparam Derived Derived class type. Must inherit from Interface.
    ///
    ///@tparam Update  Compile-time pointer to member function of Derived.
    ///
    ///@see Interface::setSendFunction()
    ///
    template<class Derived, bool (Derived::*Update)(Timeout, bool)>
    void Interface::setUpdateFunction()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must be derived from C::mip_interface.");

        setUpdateFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)->bool
        {
            return (static_cast<Derived*>(device)->*Update)(wait_time, from_cmd);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the callback functions to a common class object.
    ///
    ///@tparam T
    ///        A class type.
    ///
    ///@tparam Send
    ///        A member function pointer for sending bytes to the device.
    ///        May be NULL.
    ///
    ///@tparam Recv
    ///        A member function pointer for receiving bytes from the device.
    ///        May be NULL.
    ///
    ///@tparam Update
    ///        A member function pointer for updating the device.
    ///        If both this and Recv are NULL, no update function is set.
    ///        If Update is NULL but Recv is not, the default update function
    ///        is used instead.
    ///
    ///@param object
    ///       An instance of T. The interface's user pointer will be set to this
    ///       value. All the callbacks will be invoked on this instance.
    ///
    ///@code{.cpp}
    /// class DeviceConnection
    /// {
    ///     bool send(const uint8_t* data, size_t length);
    ///     bool recv(uint8_t* buffer, size_t max_length, Timeout wait_time, size_t* length_out, Timestamp* timestamp_out);
    /// };
    ///
    /// DeviceConnection connection;
    /// mip::Interface interface;
    ///
    /// interface.setCallbacks<DeviceConnection, &DeviceConnection::send, &DeviceConnection::recv, nullptr>(&connection);
    ///@endcode
    ///
    template<
        class T,
        bool (T::*Send)(const uint8_t*, size_t),
        bool (T::*Recv)(Timeout, bool, Timestamp*),
        bool (T::*Update)(Timeout, bool)
    >
    void Interface::setCallbacks(T* object)
    {
        C::mip_send_callback send = [](C::mip_interface* device, const uint8_t* data, size_t size)
        {
            return (static_cast<T*>(mip_interface_user_pointer(device))->*Send)(data, size);
        };

        C::mip_recv_callback recv = [](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd, C::mip_timestamp* timestamp_out)
        {
            return (static_cast<T*>(mip_interface_user_pointer(device))->*Recv)(wait_time, from_cmd, timestamp_out);
        };

        C::mip_update_callback update = [](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)
        {
            return (static_cast<T*>(mip_interface_user_pointer(device))->*Update)(wait_time, from_cmd);
        };

        C::mip_interface_set_user_pointer(this, object);
        C::mip_interface_set_send_function(this, Send != nullptr ? send : nullptr);
        C::mip_interface_set_recv_function(this, Recv != nullptr ? recv : nullptr);

        if (Update != nullptr)
            C::mip_interface_set_update_function(this, update);
        else if (Recv != nullptr)
            C::mip_interface_set_update_function(this, &C::mip_interface_default_update);
        else
            C::mip_interface_set_update_function(this, nullptr);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a packet callback (free function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        function pointer.
    ///
    ///@param handler
    ///
    ///@param descriptorSet
    ///
    ///@param afterFields
    ///
    ///@param userData
    ///
    /// Example usage:
    ///@code{.cpp}
    /// void handle_packet(void* context, const PacketRef& packet, Timestamp timestamp)
    /// {
    ///   // Use the packet data
    /// }
    ///
    /// Interface device;
    /// DispatchHandler handler;
    ///
    /// void setup()
    /// {
    ///   // Set up device...
    ///
    ///   device.registerPacketCallback<&handle_packet>(handler, descriptorSet, nullptr);
    /// }
    ///
    ///@endcode
    ///
    template<void (*Callback)(void*, const PacketView&, Timestamp)>
    void Interface::registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, void* userData)
    {
        C::mip_dispatch_packet_callback callback = [](void* context, const C::mip_packet_view* packet, Timestamp timestamp)
        {
            PacketView packetView(*packet);
            Callback(context, packetView, timestamp);
        };

        registerPacketCallback(handler, descriptorSet, afterFields, callback, userData);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a packet callback (member function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        member function pointer.
    ///
    ///@param handler
    ///
    ///@param descriptorSet
    ///
    ///@param afterFields
    ///
    ///@param object
    ///
    /// Example usage:
    ///@code{.cpp}
    /// class MySystem
    /// {
    ///   void handlePacket(const PacketRef& packet, Timestamp timestamp)
    ///   {
    ///   }
    ///
    ///   void setup()
    ///   {
    ///     // setup device...
    ///     device.registerPacketHandler<MySystem, &MySystem::handlePacket>(packetHandler, descriptorSet, this);
    ///   }
    ///
    ///   Interface device;
    ///   DispatchHandler packetHandler;
    /// };
    ///@endcode
    ///
    template<class Object, void (Object::*Callback)(const PacketView&, Timestamp)>
    void Interface::registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, Object* object)
    {
        C::mip_dispatch_packet_callback callback = [](void* pointer, const C::mip_packet_view* packet, Timestamp timestamp)->void
        {
            PacketView packetView(*packet);
            Object* obj = static_cast<Object*>(pointer);
            (obj->*Callback)(packetView, timestamp);
        };

        registerPacketCallback(handler, descriptorSet, afterFields, callback, object);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a field callback (free function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        function pointer.
    ///
    ///@param handler
    ///
    ///@param descriptorSet
    ///
    ///@param fieldDescriptor
    ///
    ///@param userData
    ///
    /// Example usage:
    ///@code{.cpp}
    /// void handle_field(void* context, const Field& field, Timestamp timestamp)
    /// {
    ///   // Use the field data
    /// }
    ///
    /// Interface device;
    /// DispatchHandler handler;
    ///
    /// void setup()
    /// {
    ///   // Set up device...
    ///
    ///   device.registerFieldCallback<&handle_field>(handler, descriptorSet, nullptr);
    /// }
    ///
    ///@endcode
    ///
    template<void (*Callback)(void*, const FieldView&, Timestamp)>
    void Interface::registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, void* userData)
    {
        C::mip_dispatch_field_callback callback = [](void* context, const C::mip_field_view* field, Timestamp timestamp)
        {
            FieldView fieldView(*field);
            Callback(context, fieldView, timestamp);
        };

        registerFieldCallback(handler, descriptorSet, fieldDescriptor, callback, userData);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a field callback (member function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        member function pointer.
    ///
    ///@param handler
    ///
    ///@param descriptorSet
    ///
    ///@param fieldDescriptor
    ///
    ///@param object
    ///
    /// Example usage:
    ///@code{.cpp}
    /// class MySystem
    /// {
    ///   void handleField(const Field& field, Timestamp timestamp)
    ///   {
    ///   }
    ///
    ///   void setup()
    ///   {
    ///     // setup device...
    ///     device.registerFieldHandler<MySystem, &MySystem::handleField>(fieldHandler, descriptorSet, fieldDescriptor, this);
    ///   }
    ///
    ///   Interface device;
    ///   DispatchHandler fieldHandler;
    /// };
    ///@endcode
    ///
    template<class Object, void (Object::*Callback)(const FieldView&, Timestamp)>
    void Interface::registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, Object* object)
    {
        C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field, Timestamp timestamp)
        {
            FieldView fieldView(*field);
            Object* obj = static_cast<Object*>(pointer);
            (obj->*Callback)(fieldView, timestamp);
        };

        registerFieldCallback(handler, descriptorSet, fieldDescriptor, callback, object);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a data callback (free function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        function pointer.
    ///
    ///@param handler
    ///       This must exist as long as the handler remains registered.
    ///
    ///@param userData
    ///       Optional data to pass to the callback function.
    ///
    ///@param descriptorSet
    ///       If specified, overrides the descriptor set. Intended to be used with
    ///       shared data quantities.
    ///
    /// Example usage:
    ///@code{.cpp}
    /// void handle_packet(void* context, const PacketRef& packet, Timestamp timestamp)
    /// {
    ///   // Use the packet data
    /// }
    ///
    /// Interface device;
    /// DispatchHandler handler;
    ///
    /// void setup()
    /// {
    ///   // Set up device...
    ///
    ///   device.registerDataCallback<&handle_packet>(handler, descriptorSet, nullptr);
    /// }
    ///
    ///@endcode
    ///
    template<class DataField, void (*Callback)(void*, const DataField&, Timestamp)>
    void Interface::registerDataCallback(C::mip_dispatch_handler& handler, void* userData, uint8_t descriptorSet)
    {
        assert(descriptorSet != 0x00);
        if (descriptorSet == 0x00)
            return;

        assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
        if (descriptorSet == 0xFF)
            return;

        C::mip_dispatch_field_callback callback = [](void* context, const C::mip_field_view* field, Timestamp timestamp)
        {
            DataField data;

            bool ok = FieldView(*field).extract(data);
            assert(ok); (void)ok;

            Callback(context, data, timestamp);
        };

        registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, userData);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a data callback (free function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        function pointer.
    ///
    ///@param handler
    ///       This must exist as long as the handler remains registered.
    ///
    ///@param userData
    ///       Optional data to pass to the callback function.
    ///
    ///@param descriptorSet
    ///       If specified, overrides the descriptor set. Intended to be used with
    ///       shared data quantities.
    ///
    /// Example usage:
    ///@code{.cpp}
    /// void handle_packet(void* context, uint8_t descriptorSet, const PacketRef& packet, Timestamp timestamp)
    /// {
    ///   // Use the packet data
    /// }
    ///
    /// Interface device;
    /// DispatchHandler handler;
    ///
    /// void setup()
    /// {
    ///   // Set up device...
    ///
    ///   device.registerDataCallback<&handle_packet>(handler, descriptorSet, nullptr);
    /// }
    ///
    ///@endcode
    ///
    template<class DataField, void (*Callback)(void*, const DataField&, uint8_t, Timestamp)>
    void Interface::registerDataCallback(C::mip_dispatch_handler& handler, void* userData, uint8_t descriptorSet)
    {
        assert(descriptorSet != 0x00);
        if (descriptorSet == 0x00)
            return;

        assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
        if (descriptorSet == 0xFF)
            return;

        C::mip_dispatch_field_callback callback = [](void* context, const C::mip_field_view* field, Timestamp timestamp)
        {
            DataField data;

            bool ok = FieldView(*field).extract(data);
            assert(ok); (void)ok;

            Callback(context, data, mip_field_descriptor_set(field), timestamp);
        };

        registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, userData);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a data callback (member function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        member function pointer.
    ///
    ///@param handler
    ///       This must exist as long as the handler remains registered.
    ///
    ///@param object
    ///       A pointer to the object. The object must exist while the handler
    ///       remains registered.
    ///
    ///@param descriptorSet
    ///       If specified, overrides the descriptor set. Intended to be used with
    ///       shared data quantities.
    ///
    /// Example usage:
    ///@code{.cpp}
    /// class MySystem
    /// {
    ///   void handleAccel(const ScaledAccel& accel, Timestamp timestamp)
    ///   {
    ///   }
    ///
    ///   void setup()
    ///   {
    ///     // setup device...
    ///     device.registerDataHandler<ScaledAccel, MySystem, &MySystem::handleAccel>(accelHandler, this);
    ///   }
    ///
    ///   Interface device;
    ///   DispatchHandler accelHandler;
    /// };
    ///@endcode
    ///
    template<class DataField, class Object, void (Object::*Callback)(const DataField&, Timestamp)>
    void Interface::registerDataCallback(C::mip_dispatch_handler& handler, Object* object, uint8_t descriptorSet)
    {
        assert(descriptorSet != 0x00);
        if (descriptorSet == 0x00)
            return;

        assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
        if (descriptorSet == 0xFF)
            return;

        C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field, Timestamp timestamp)
        {
            DataField data;

            bool ok = FieldView(*field).extract(data);
            assert(ok); (void)ok;

            Object* obj = static_cast<Object*>(pointer);
            (obj->*Callback)(data, timestamp);
        };

        registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, object);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Registers a data callback (member function version).
    ///
    ///@tparam Callback A pointer to the function to call. This must be a constant
    ///        member function pointer.
    ///
    ///@param handler
    ///       This must exist as long as the handler remains registered.
    ///
    ///@param object
    ///       A pointer to the object. The object must exist while the handler
    ///       remains registered.
    ///
    ///@param descriptorSet
    ///       If specified, overrides the descriptor set. Intended to be used with
    ///       shared data quantities.
    ///
    /// Example usage:
    ///@code{.cpp}
    /// class MySystem
    /// {
    ///   void handleAccel(const ScaledAccel& accel, uint8_t descriptorSet, Timestamp timestamp)
    ///   {
    ///   }
    ///
    ///   void setup()
    ///   {
    ///     // setup device...
    ///     device.registerDataHandler<ScaledAccel, MySystem, &MySystem::handleAccel>(accelHandler, this);
    ///   }
    ///
    ///   Interface device;
    ///   DispatchHandler accelHandler;
    /// };
    ///@endcode
    ///
    template<class DataField, class Object, void (Object::*Callback)(const DataField&, uint8_t, Timestamp)>
    void Interface::registerDataCallback(C::mip_dispatch_handler& handler, Object* object, uint8_t descriptorSet)
    {
        assert(descriptorSet != 0x00);
        if (descriptorSet == 0x00)
            return;

        assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
        if (descriptorSet == 0xFF)
            return;

        C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field, Timestamp timestamp)
        {
            DataField data;

            bool ok = FieldView(*field).extract(data);
            assert(ok); (void)ok;

            Object* obj = static_cast<Object*>(pointer);
            (obj->*Callback)(data, mip_field_descriptor_set(field), timestamp);
        };

        registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, object);
    }

    template<class DataField>
    void Interface::registerExtractor(C::mip_dispatch_handler& handler, DataField* field, uint8_t descriptorSet)
    {
        C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field, Timestamp /*timestamp*/)
        {
            FieldView(*field).extract(*static_cast<DataField*>(pointer));
        };

        registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, field);
    }

    template<class Cmd>
    CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, Timeout additionalTime)
    {
        PacketBuf packet(cmd);

        C::mip_pending_cmd pending;
        C::mip_pending_cmd_init_with_timeout(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, additionalTime);

        return C::mip_interface_run_command_packet(&device, &packet, &pending);
    }

    template<class Cmd, class... Args>
    CmdResult runCommand(C::mip_interface& device, const Args&&... args, Timeout additionalTime)
    {
        Cmd cmd{std::forward<Args>(args)...};
        return runCommand(device, cmd, additionalTime);
    }

    template<class Cmd>
    CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, typename Cmd::Response& response, Timeout additionalTime)
    {
        PacketBuf packet(cmd);

        C::mip_pending_cmd pending;
        C::mip_pending_cmd_init_full(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, Cmd::Response::FIELD_DESCRIPTOR, packet.buffer(), FIELD_PAYLOAD_LENGTH_MAX, additionalTime);

        CmdResult result = C::mip_interface_run_command_packet(&device, &packet, &pending);
        if (result != C::MIP_ACK_OK)
            return result;

        size_t responseLength = C::mip_pending_cmd_response_length(&pending);

        return extract(response, packet.buffer(), responseLength, 0) ? CmdResult::ACK_OK : CmdResult::STATUS_ERROR;
    }

    template<class Cmd>
    bool startCommand(C::mip_interface& device, C::mip_pending_cmd& pending, const Cmd& cmd, Timeout additionalTime)
    {
        PacketBuf packet(cmd);

        C::mip_pending_cmd_init_with_timeout(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, additionalTime);

        return C::mip_interface_start_command_packet(&device, &packet, &pending);
    }

    //template<class Cmd>
    //bool startCommand(C::mip_interface& device, C::mip_pending_cmd& pending, const Cmd& cmd, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime)
    //{
    //    PacketBuf packet(cmd);
    //
    //    C::mip_pending_cmd_init_full(&pending, Cmd::descriptorSet, Cmd::fieldDescriptor, Cmd::Response::fieldDescriptor, responseBuffer, responseBufferSize, additionalTime);
    //
    //    return C::mip_interface_start_command_packet(&device, &packet, &pending);
    //}

    ///@}
    ////////////////////////////////////////////////////////////////////////////////
} // namespace mip
