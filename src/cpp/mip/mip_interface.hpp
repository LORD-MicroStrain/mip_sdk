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
    bool recv_from_connection(microstrain::Connection* conn, microstrain::Span<uint8_t> buffer, Timeout timeout, bool from_cmd, size_t* length_out, Timestamp* timestamp_out);

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
        // Connection-related callback functions
        //

        // C function callbacks

        C::mip_send_callback   sendFunction()   const { return C::mip_interface_send_function(this);   }
        C::mip_recv_callback   recvFunction()   const { return C::mip_interface_recv_function(this);   }
        C::mip_update_callback updateFunction() const { return C::mip_interface_update_function(this); }

        void setSendFunction  (C::mip_send_callback   callback) { C::mip_interface_set_send_function  (this, callback); }
        void setRecvFunction  (C::mip_recv_callback   callback) { C::mip_interface_set_recv_function  (this, callback); }
        void setUpdateFunction(C::mip_update_callback function) { C::mip_interface_set_update_function(this, function); }

        // Free/nonmember function callbacks

        template<bool (*Send)(Interface&, const uint8_t*, size_t)>
        void setSendFunctionFree();
        template<bool (*Send)(Interface&, microstrain::Span<const uint8_t>)>
        void setSendFunctionFree();

        template<bool (*Recv)(Interface&, uint8_t*, size_t, Timeout, bool, size_t*, Timestamp*)>
        void setRecvFunctionFree();
        template<bool (*Recv)(Interface&, microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
        void setRecvFunctionFree();

        template<bool (*Update)(Interface&, Timeout, bool)>
        void setUpdateFunctionFree();

        // Class member function callbacks - user pointer must be valid!

        template<class UserClass, bool (UserClass::*Send)(const uint8_t*, size_t)>
        void setSendFunctionUserPointer();
        template<class UserClass, bool (UserClass::*Send)(microstrain::Span<const uint8_t>)>
        void setSendFunctionUserPointer();
        template<class UserClass, bool (*Send)(UserClass*, microstrain::Span<const uint8_t>)>
        void setSendFunctionUserPointer();

        template<class UserClass, bool (UserClass::*Recv)(uint8_t*, size_t, Timeout, bool, size_t*, Timestamp*)>
        void setRecvFunctionUserPointer();
        template<class UserClass, bool (UserClass::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
        void setRecvFunctionUserPointer();
        template<class UserClass, bool (*Recv)(UserClass*, microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
        void setRecvFunctionUserPointer();

        template<class UserClass, bool (UserClass::*Update)(Timeout, bool)>
        void setUpdateFunctionUserPointer();
        template<class UserClass, bool (*Update)(UserClass*, Timeout, bool)>
        void setUpdateFunctionUserPointer();

        // All-in-one function
        template<
            class T,
            bool (T::*Send)(microstrain::Span<const uint8_t>),
            bool (T::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*),
            bool (T::*Update)(Timeout, bool) = nullptr
        >
        void setCallbacksUserPointer(T* object);

        // Derived class member function callbacks

        template<class Derived, bool (Derived::*Send)(microstrain::Span<const uint8_t>)>
        void setSendFunctionDerived();

        template<class Derived, bool (Derived::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
        void setRecvFunctionDerived();

        template<class Derived, bool (Derived::*Update)(Timeout, bool)>
        void setUpdateFunctionDerived();

        // All-in-one function
        template<
            class Derived,
            bool (Derived::*Send)(microstrain::Span<const uint8_t>),
            bool (Derived::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*),
            bool (Derived::*Update)(Timeout, bool) = nullptr
        >
        void setCallbacksDerived();

        //
        // General accessors
        //

        ///@brief Sets an optional user data pointer which can be retrieved later.
        ///
        ///@warning If using connect_interface() or constructing using a microstrain::Connection object,
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
        bool recvFromDevice(microstrain::Span<uint8_t> buffer, Timeout wait_time, bool from_cmd, size_t* length_out, Timestamp* timestamp_out) { return C::mip_interface_recv_from_device(this, buffer.data(), buffer.size(), wait_time, from_cmd, length_out, timestamp_out); }
        bool update(Timeout wait_time = 0, bool from_cmd = false) { return C::mip_interface_update(this, wait_time, from_cmd); }

        bool defaultUpdate(Timeout wait_time = 0, bool from_cmd = false) { return C::mip_interface_default_update(this, wait_time, from_cmd); }
        bool defaultUpdateExtBuffer(Timeout wait_time, bool from_cmd, microstrain::Span<uint8_t> buffer) { return C::mip_interface_default_update_ext_buffer(this, wait_time, from_cmd, buffer.data(), buffer.size()); }
        void inputBytes(const uint8_t* data, size_t length, Timestamp timestamp) { C::mip_interface_input_bytes_from_device(this, data, length, timestamp); }
        void inputPacket(const C::mip_packet_view& packet, Timestamp timestamp) { C::mip_interface_input_packet_from_device(this, &packet, timestamp); }
        void updateTime(Timestamp timestamp) { C::mip_interface_update_time(this, timestamp); }
        void inputBytesAndOrTime(microstrain::Span<const uint8_t> data, Timestamp timestamp) { C::mip_interface_input_bytes_andor_time(this, data.data(), data.size(), timestamp); }

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
    ////////////////////////////////////////////////////////////////////////////////
    //
    // Connection callback assignment functions
    //
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    //
    // Free / non-member functions
    //
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (free function version).
    ///
    ///@tparam Send A compile-time pointer to the callback function.
    ///
    template<bool (*Send)(Interface&, const uint8_t*, size_t)>
    void Interface::setSendFunctionFree()
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
    void Interface::setSendFunctionFree()
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
    template<bool (*Recv)(Interface&, uint8_t*, size_t, Timeout, bool, size_t*, Timestamp*)>
    void Interface::setRecvFunctionFree()
    {
        setRecvFunction([](C::mip_interface* device, uint8_t* buffer, size_t max_length, C::mip_timeout wait_time, bool from_cmd, size_t* length_out, C::mip_timestamp* timestamp_out)->bool
        {
            return (*Recv)(*static_cast<Interface*>(device), buffer, max_length, wait_time, from_cmd, length_out, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (free function w/ span version).
    ///
    ///@tparam Recv A compile-time pointer to the callback function.
    ///
    template<bool (*Recv)(Interface&, microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
    void Interface::setRecvFunctionFree()
    {
        setRecvFunction([](C::mip_interface* device, uint8_t* buffer, size_t max_length, C::mip_timeout wait_time, bool from_cmd, size_t* length_out, C::mip_timestamp* timestamp_out)->bool
        {
            return (*Recv)(*static_cast<Interface*>(device), {buffer, max_length}, wait_time, from_cmd, length_out, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the update callback function (free function version).
    ///
    ///@tparam Update A compile-time pointer to the callback function.
    ///
    template<bool (*Update)(Interface&, Timeout, bool)>
    void Interface::setUpdateFunctionFree()
    {
        setUpdateFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)->bool
        {
            return (*Update)(*static_cast<Interface*>(device), wait_time, from_cmd);
        });
    }


    ////////////////////////////////////////////////////////////////////////////////
    //
    // Class member functions on user data pointer (cast user pointer)
    //
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (class member function version w/ ptr & length).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Send Compile-time pointer to member function of type UserClass.
    ///
    ///@code{.cpp}
    /// class MyClass
    /// {
    ///     bool sendToDevice(const uint8_t* data, size_t size);
    ///     bool recvFromDevice(uint8_t* data, size_t max_length, Timeout wait_time, size_t* length_out, Timestamp* timestamp_out);
    /// };
    ///
    /// MyClass instance;
    ///
    /// instance.setUserPointer(&instance);
    /// instance.setSendFunction<MyClass, &MyClass::sendToDevice>();
    /// instance.setRecvFunction<MyClass, &MyClass::recvFromDevice>();
    ///
    ///@endcode
    ///
    template<class UserClass, bool (UserClass::*Send)(const uint8_t*, size_t)>
    void Interface::setSendFunctionUserPointer()
    {
        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (object->*Send)(data, length);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (class member function version).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Send Compile-time pointer to member function of type UserClass.
    ///
    ///@code{.cpp}
    /// class MyClass
    /// {
    ///     bool sendToDevice(const uint8_t* data, size_t size);
    ///     bool recvFromDevice(microstrain::Span<uint8_t>, Timeout wait_time, size_t* length_out, Timestamp* timestamp_out);
    /// };
    ///
    /// MyClass instance;
    ///
    /// instance.setUserPointer(&instance);
    /// instance.setSendFunction<MyClass, &MyClass::sendToDevice>();
    /// instance.setRecvFunction<MyClass, &MyClass::recvFromDevice>();
    ///
    ///@endcode
    ///
    template<class UserClass, bool (UserClass::*Send)(microstrain::Span<const uint8_t>)>
    void Interface::setSendFunctionUserPointer()
    {
        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (object->*Send)({data, length});
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the send callback function (nonmember member function version).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Send Compile-time pointer to function taking an object of type UserClass.
    ///
    ///@code{.cpp}
    /// class MyClass
    /// {
    /// };
    /// bool sendToDevice(MyClass* mc, const uint8_t* data, size_t size);
    /// bool recvFromDevice(MyClass* mc, microstrain::Span<uint8_t>, Timeout wait_time, size_t* length_out, Timestamp* timestamp_out);
    ///
    /// MyClass instance;
    ///
    /// instance.setUserPointer(&instance);
    /// instance.setSendFunction<MyClass, &MyClass::sendToDevice>();
    /// instance.setRecvFunction<MyClass, &MyClass::recvFromDevice>();
    ///
    ///@endcode
    ///
    template<class UserClass, bool (*Send)(UserClass*, microstrain::Span<const uint8_t>)>
    void Interface::setSendFunctionUserPointer()
    {
        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (*Send)(object, {data, length});
        });
    }


    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (class member function version w/ ptr & length).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Recv Compile-time pointer to member function of UserClass.
    ///
    ///@see Interface::setSendFunctionUserPointer()
    ///
    template<class UserClass, bool (UserClass::*Recv)(uint8_t*, size_t, Timeout, bool, size_t*, Timestamp*)>
    void Interface::setRecvFunctionUserPointer()
    {
        setRecvFunction([](C::mip_interface* device, uint8_t* buffer, size_t max_length, Timeout wait_time, bool from_cmd, size_t* length_out, Timestamp* timestamp_out)
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (object->*Recv)(buffer, max_length, wait_time, from_cmd, length_out, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (class member function version).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Recv Compile-time pointer to member function of UserClass.
    ///
    ///@see Interface::setSendFunctionUserPointer()
    ///
    template<class UserClass, bool (UserClass::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
    void Interface::setRecvFunctionUserPointer()
    {
        setRecvFunction([](C::mip_interface* device, uint8_t* buffer, size_t max_length, Timeout wait_time, bool from_cmd, size_t* length_out, Timestamp* timestamp_out)
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (object->*Recv)({buffer, max_length}, wait_time, from_cmd, length_out, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (nonmember function version).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Recv Compile-time pointer to function taking an object of type C.
    ///
    ///@see Interface::setSendFunctionUserPointer()
    ///
    template<class UserClass, bool (*Recv)(UserClass*, microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
    void Interface::setRecvFunctionUserPointer()
    {
        setRecvFunction([](C::mip_interface* device, uint8_t* buffer, size_t max_length, Timeout wait_time, bool from_cmd, size_t* length_out, Timestamp* timestamp_out)
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (*Recv)(object, {buffer, max_length}, wait_time, from_cmd, length_out, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the update callback function (class member function version).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Update  Compile-time pointer to member function of Derived.
    ///
    ///@see Interface::setSendFunction()
    ///
    template<class UserClass, bool (UserClass::*Update)(Timeout, bool)>
    void Interface::setUpdateFunctionUserPointer()
    {
        setUpdateFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)->bool
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (object->*Update)(wait_time, from_cmd);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the update callback function (nonmember function version).
    ///
    ///@tparam UserClass Class type of the user pointer value.
    ///
    ///@tparam Update  Compile-time pointer to member function of Derived.
    ///
    ///@see Interface::setSendFunction()
    ///
    template<class UserClass, bool (*Update)(UserClass*, Timeout, bool)>
    void Interface::setUpdateFunctionUserPointer()
    {
        setUpdateFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)->bool
        {
            auto* object = static_cast<UserClass*>(C::mip_interface_user_pointer(device));
            return (*Update)(object, wait_time, from_cmd);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the callback functions to a common class object.
    ///
    ///@tparam UserClass
    ///        Class type of the user pointer value.
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
    ///       An instance of UserClass. The interface's user pointer will be set to this
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
    /// interface.setCallbacksUserPointer<DeviceConnection, &DeviceConnection::send, &DeviceConnection::recv, nullptr>(&connection);
    ///@endcode
    ///
    template<
        class UserClass,
        bool (UserClass::*Send)(microstrain::Span<const uint8_t>),
        bool (UserClass::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*),
        bool (UserClass::*Update)(Timeout, bool)
    >
    void Interface::setCallbacksUserPointer(UserClass* object)
    {
        setUserPointer(object);

        setSendFunctionUserPointer<UserClass, Send>();
        setRecvFunctionUserPointer<UserClass, Recv>();

        // Use default update function if an update function is not provided.
        if(Update == nullptr && Recv != nullptr)
            setUpdateFunction(&C::mip_interface_default_update);
        else
            setUpdateFunctionUserPointer<UserClass, Update>();
    }


    ////////////////////////////////////////////////////////////////////////////////
    //
    // Derived member functions (cast this to derived type)
    //
    ////////////////////////////////////////////////////////////////////////////////

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
    ///
    /// instance.setSendFunction<MyClass, &MyClass::sendToDevice>();
    /// instance.setRecvFunction<MyClass, &MyClass::recvFromDevice>();
    ///@endcode
    ///
    template<class Derived, bool (Derived::*Send)(microstrain::Span<const uint8_t>)>
    void Interface::setSendFunctionDerived()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must inherit C::mip_interface.");

        setSendFunction([](C::mip_interface* device, const uint8_t* data, size_t length)
        {
            return (static_cast<Derived*>(device)->*Send)({data, length});
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the receive callback function (derived member function version).
    ///
    ///@tparam Derived Derived class type. Must inherit from Interface.
    ///
    ///@tparam Recv    Compile-time pointer to member function of Derived.
    ///
    ///@see Interface::setSendFunctionDerived()
    ///
    template<class Derived, bool (Derived::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*)>
    void Interface::setRecvFunctionDerived()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must inherit C::mip_interface.");

        setRecvFunction([](C::mip_interface* device, uint8_t* buffer, size_t max_length, Timeout wait_time, bool from_cmd, size_t* length_out, Timestamp* timestamp_out)
        {
            return (static_cast<Derived*>(device)->*Recv)({buffer, max_length}, wait_time, from_cmd, length_out, timestamp_out);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the update callback function (derived member function version).
    ///
    ///@tparam Derived Derived class type. Must inherit from Interface.
    ///
    ///@tparam Update  Compile-time pointer to member function of Derived.
    ///
    ///@see Interface::setSendFunctionDerived()
    ///
    template<class Derived, bool (Derived::*Update)(Timeout, bool)>
    void Interface::setUpdateFunctionDerived()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must inherit C::mip_interface.");

        setUpdateFunction([](C::mip_interface* device, C::mip_timeout wait_time, bool from_cmd)->bool
        {
            return (static_cast<Derived*>(device)->*Update)(wait_time, from_cmd);
        });
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Sets the callback functions to a derived class object.
    ///
    ///@tparam Derived
    ///        A class type which derives from mip::Interface or mip::C::mip_interface.
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
    ///@code{.cpp}
    /// class Device : public mip::Interface
    /// {
    ///     bool send(microstrain::Span<const uint8_t>);
    ///     bool recv(microstrain::Span<uint8_t>, Timeout wait_time, size_t* length_out, Timestamp* timestamp_out);
    /// };
    ///
    /// DeviceConnection connection;
    /// mip::Interface interface;
    ///
    /// interface.setCallbacksDerived<Device, &Device::send, &Device::recv, nullptr>(&connection);
    ///@endcode
    ///
    template<
        class Derived,
        bool (Derived::*Send)(microstrain::Span<const uint8_t>),
        bool (Derived::*Recv)(microstrain::Span<uint8_t>, Timeout, bool, size_t*, Timestamp*),
        bool (Derived::*Update)(Timeout, bool)
    >
    void Interface::setCallbacksDerived()
    {
        static_assert(std::is_base_of<C::mip_interface, Derived>::value, "Derived must inherit C::mip_interface.");

        setSendFunctionDerived<Derived, Send>();
        setRecvFunctionDerived<Derived, Recv>();

        // Use default update function if an update function is not provided.
        if(Update == nullptr && Recv != nullptr)
            setUpdateFunction(&C::mip_interface_default_update);
        else
            setRecvFunctionDerived<Derived, Update>();
    }


    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    //
    // Data callback assignment functions
    //
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////


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


    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    //
    // Run command functions
    //
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////


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
