#pragma once

#include "mip.hpp"
#include "mip_interface.h"

#include "definitions/descriptors.h"


namespace mip
{

using DispatchHandler = C::mip_dispatch_handler;

struct Dispatcher : public C::mip_dispatcher
{
    enum : uint8_t {
        ANY_DATA_SET = C::MIP_DISPATCH_ANY_DATA_SET,
        ANY_DESCRIPTOR = C::MIP_DISPATCH_ANY_DESCRIPTOR,
    };
};


struct CmdQueue : public C::mip_cmd_queue
{
    CmdQueue(Timeout baseReplyTimeout=1000) { C::mip_cmd_queue_init(this, baseReplyTimeout); }
    ~CmdQueue() { assert(_first_pending_cmd==nullptr); }

    CmdQueue(const CmdQueue&) = delete;
    CmdQueue& operator=(const CmdQueue&) = delete;

    void enqueue(C::mip_pending_cmd& cmd) { C::mip_cmd_queue_enqueue(this, &cmd); }
    void dequeue(C::mip_pending_cmd& cmd) { C::mip_cmd_queue_dequeue(this, &cmd); }

    void update(Timestamp now) { C::mip_cmd_queue_update(this, now); }

    void setBaseReplyTimeout(Timeout timeout) { C::mip_cmd_queue_set_base_reply_timeout(this, timeout); }
    Timeout baseReplyTimeout() const { return C::mip_cmd_queue_base_reply_timeout(this); }

    void processPacket(const C::mip_packet& packet, Timestamp timestamp) { C::mip_cmd_queue_process_packet(this, &packet, timestamp); }
};

struct PendingCmd : public C::mip_pending_cmd
{
    PendingCmd() { std::memset(static_cast<C::mip_pending_cmd*>(this), 0, sizeof(C::mip_pending_cmd)); }
    PendingCmd(uint8_t descriptorSet, uint8_t fieldDescriptor, Timeout additionalTime=0) { C::mip_pending_cmd_init_with_timeout(this, descriptorSet, fieldDescriptor, additionalTime); }
    PendingCmd(uint8_t descriptorSet, uint8_t fieldDescriptor, uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime) { C::mip_pending_cmd_init_full(this, descriptorSet, fieldDescriptor, responseDescriptor, responseBuffer, responseBufferSize, additionalTime); }

    template<class Cmd>
    PendingCmd(const Cmd& cmd, Timeout additionalTime=0) : PendingCmd(cmd.descriptorSet, cmd.fieldDescriptor, additionalTime) {}

    template<class Cmd>
    PendingCmd(const Cmd& cmd, const typename Cmd::Response& response, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime=0) : PendingCmd(cmd.descriptorSet, cmd.fieldDescriptor, response.fieldDescriptor, responseBuffer, responseBufferSize, additionalTime) {}

    PendingCmd(const PendingCmd&) = delete;
    PendingCmd& operator=(const PendingCmd&) = delete;

    ~PendingCmd() { CmdResult tmp = status(); assert(tmp.isFinished() || tmp==CmdResult::STATUS_NONE); (void)tmp; }

    CmdResult status() const { return C::mip_pending_cmd_status(this); }

    const uint8_t* response() const { return C::mip_pending_cmd_response(this); }
    uint8_t responseLength() const { return C::mip_pending_cmd_response_length(this); }
};



template<class Cmd> CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, Timeout additionalTime=0);
template<class Cmd> CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, typename Cmd::Response& response, Timeout additionalTime=0);
template<class Cmd, class... Args> CmdResult runCommand(C::mip_interface& device, const Args&&... args, Timeout additionalTime);
template<class Cmd> bool startCommand(C::mip_interface& device, C::mip_pending_cmd& pending, const Cmd& cmd, Timeout additionalTime);

class DeviceInterface : public C::mip_interface
{
public:
    //
    // Constructors
    //

    ///@copydoc mip_interface_init
    DeviceInterface(uint8_t* parseBuffer, size_t parseBufferSize, Timeout parseTimeout, Timeout baseReplyTimeout) { C::mip_interface_init(this, parseBuffer, parseBufferSize, parseTimeout, baseReplyTimeout); }

    DeviceInterface(const DeviceInterface&) = delete;
    DeviceInterface& operator=(const DeviceInterface&) = delete;

    ~DeviceInterface() = default;

    //
    // Accessors
    //

    void setMaxPacketsPerPoll(unsigned int maxPackets) { C::mip_interface_set_max_packets_per_update(this, maxPackets); }
    unsigned int maxPacketsPerPoll() const             { return C::mip_interface_max_packets_per_update(this); }

    Timeout baseReplyTimeout() const          { return C::mip_cmd_queue_base_reply_timeout(&cmdQueue()); }
    void setBaseReplyTimeout(Timeout timeout) { C::mip_cmd_queue_set_base_reply_timeout(&cmdQueue(), timeout); }

    Parser&   parser()   { return *static_cast<Parser*>(C::mip_interface_parser(this)); }
    CmdQueue& cmdQueue() { return *static_cast<CmdQueue*>(C::mip_interface_cmd_queue(this)); }

    const Parser&   parser()   const   { return const_cast<DeviceInterface*>(this)->parser(); }
    const CmdQueue& cmdQueue() const { return const_cast<DeviceInterface*>(this)->cmdQueue(); }

    //
    // Communications
    //

    RemainingCount receiveBytes(const uint8_t* data, size_t length, Timestamp timestamp) { return C::mip_interface_receive_bytes(this, data, length, timestamp); }

    void           receivePacket(const C::mip_packet& packet, Timestamp timestamp) { C::mip_interface_receive_packet(this, &packet, timestamp); }

    virtual bool   sendToDevice(const uint8_t* data, size_t length) = 0;  // Must be implemented by a derived class.
    bool           sendToDevice(const C::mip_packet& packet) { return sendToDevice(C::mip_packet_pointer(&packet), C::mip_packet_total_length(&packet)); }

    bool           update() { return C::mip_interface_update(this); }
    virtual bool   recvFromDevice(uint8_t* buffer, size_t max_length, size_t* length_out, Timestamp* timestamp) = 0;  // Must be implemented by a derived class.

    void           processUnparsedPackets() { C::mip_interface_process_unparsed_packets(this); }

    CmdResult      waitForReply(const C::mip_pending_cmd& cmd) { return C::mip_interface_wait_for_reply(this, &cmd); }


    template<class Function>
    bool parseFromSource(Function function, unsigned int maxPackets=MIPPARSER_UNLIMITED_PACKETS) { return parseMipDataFromSource(parser(), function, maxPackets); }

    //
    // These must be implemented by a derived class
    //

    // virtual bool update() = 0;
    // virtual bool sendToDevice(const uint8_t* data, size_t length) = 0;

    //
    // Data Callbacks
    //

    void registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, C::mip_dispatch_packet_callback callback, void* userData) { C::mip_interface_register_packet_callback(this, &handler, descriptorSet, afterFields, callback, userData); }
    void registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, C::mip_dispatch_field_callback callback, void* userData) { C::mip_interface_register_field_callback(this, &handler, descriptorSet, fieldDescriptor, callback, userData); }


    template<void (*Callback)(void*, const Packet&, Timestamp)>
    void registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, void* userData=nullptr);

    template<class Object, void (Object::*Callback)(const Packet&, Timestamp)>
    void registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, Object* object);


    template<void (*Callback)(void*, const Field&, Timestamp)>
    void registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, void* userData=nullptr);

    template<class Object, void (Object::*Callback)(const Field& field, Timestamp)>
    void registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, Object* object);


    template<class DataField, void (*Callback)(void*, const DataField&, Timestamp)>
    void registerDataCallback(C::mip_dispatch_handler& handler, void* userData=nullptr, uint8_t descriptorSet=DataField::DESCRIPTOR_SET);

    template<class DataField, class Object, void (Object::*Callback)(const DataField&, Timestamp)>
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
//    bool startCommand(PendingCmd& pending, const Cmd& cmd, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime=0) { return mscl::startCommand(pending, cmd, responseBuffer, responseBufferSize, additionalTime); }
};


////////////////////////////////////////////////////////////////////////////////
///@brief Registers a packet callback (free function version).
///
///@tparam Callback A pointer to the function to call. This must be a constant
///        function pointer.
///
///@param handler
///@param descriptorSet
///@param userData
///
/// Example usage:
///@code{.cpp}
/// void handle_packet(void* context, const Packet& packet, Timestamp timestamp)
/// {
///   // Use the packet data
/// }
///
/// DeviceInterface device;
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
template<void (*Callback)(void*, const Packet&, Timestamp)>
void DeviceInterface::registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, void* userData)
{
    auto callback = [](void* context, const C::mip_packet* packet, Timestamp timestamp)
    {
        Callback(context, Packet(*packet), timestamp);
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
///@param descriptorSet
///@param object
///
/// Example usage:
///@code{.cpp}
/// class MySystem
/// {
///   void handlePacket(const Packet& packet, Timestamp timestamp)
///   {
///   }
///
///   void setup()
///   {
///     // setup device...
///     device.registerPacketHandler<MySystem, &MySystem::handlePacket>(packetHandler, descriptorSet, this);
///   }
///
///   DeviceInterface device;
///   DispatchHandler packetHandler;
/// };
///@endcode
///
template<class Object, void (Object::*Callback)(const Packet&, Timestamp)>
void DeviceInterface::registerPacketCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, bool afterFields, Object* object)
{
    auto callback = [](void* pointer, const Packet& packet, Timestamp timestamp)
    {
        Object* obj = static_cast<Object*>(pointer);
        (obj->*Callback)(Packet(packet), timestamp);
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
///@param descriptorSet
///@param fieldDescriptor
///@param userData
///
/// Example usage:
///@code{.cpp}
/// void handle_field(void* context, const Field& field, Timestamp timestamp)
/// {
///   // Use the field data
/// }
///
/// DeviceInterface device;
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
template<void (*Callback)(void*, const Field&, Timestamp)>
void DeviceInterface::registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, void* userData)
{
    auto callback = [](void* context, const C::mip_field* field, Timestamp timestamp)
    {
        Callback(context, Field(*field), timestamp);
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
///@param descriptorSet
///@param fieldDescriptor
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
///   DeviceInterface device;
///   DispatchHandler fieldHandler;
/// };
///@endcode
///
template<class Object, void (Object::*Callback)(const Field&, Timestamp)>
void DeviceInterface::registerFieldCallback(C::mip_dispatch_handler& handler, uint8_t descriptorSet, uint8_t fieldDescriptor, Object* object)
{
    auto callback = [](void* pointer, const C::mip_field* field, Timestamp timestamp)
    {
        Object* obj = static_cast<Object*>(pointer);
        (obj->*Callback)(Field(*field), timestamp);
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
///       This must exist as long as the hander remains registered.
///
///@param userData
///       Optional data to pass to the callback function.
///
///@param descriptorSet
///       If specified, overrides the descriptor set. Intended to be used with
///       with shared data quantities.
///
/// Example usage:
///@code{.cpp}
/// void handle_packet(void* context, const Packet& packet, Timestamp timestamp)
/// {
///   // Use the packet data
/// }
///
/// DeviceInterface device;
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
void DeviceInterface::registerDataCallback(C::mip_dispatch_handler& handler, void* userData, uint8_t descriptorSet)
{
    assert(descriptorSet != 0x00);
    if(descriptorSet == 0x00)
        return;

    assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
    if(descriptorSet == 0xFF)
        return;

    auto callback = [](void* context, const C::mip_field* field, Timestamp timestamp)
    {
        DataField data;

        bool ok = Field(*field).extract(data);
        assert(ok); (void)ok;

        Callback(context, data, timestamp);
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
///       This must exist as long as the hander remains registered.
///
///@param object
///       A pointer to the object. The object must exist while the handler
///       remains registered.
///
///@param descriptorSet
///       If specified, overrides the descriptor set. Intended to be used with
///       with shared data quantities.
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
///   DeviceInterface device;
///   DispatchHandler accelHandler;
/// };
///@endcode
///
template<class DataField, class Object, void (Object::*Callback)(const DataField&, Timestamp)>
void DeviceInterface::registerDataCallback(C::mip_dispatch_handler& handler, Object* object, uint8_t descriptorSet)
{
    assert(descriptorSet != 0x00);
    if(descriptorSet == 0x00)
        return;

    assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
    if(descriptorSet == 0xFF)
        return;

    auto callback = [](void* pointer, const C::mip_field* field, Timestamp timestamp)
    {
        DataField data;

        bool ok = Field(*field).extract(data);
        assert(ok); (void)ok;

        Object* obj = static_cast<Object*>(pointer);
        (obj->*Callback)(data, timestamp);
    };

    registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, object);
}


template<class DataField>
void DeviceInterface::registerExtractor(C::mip_dispatch_handler& handler, DataField* field, uint8_t descriptorSet)
{
    auto callback = [](void* pointer, const C::mip_field* field, Timestamp timestamp)
    {
        Field(*field).extract( *static_cast<DataField*>(pointer) );
    };

    registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, field);
}


template<class Cmd>
CmdResult runCommand(C::mip_interface& device, const Cmd& cmd, Timeout additionalTime)
{
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    Packet packet = Packet::createFromField(buffer, sizeof(buffer), cmd);

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
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    Packet packet = Packet::createFromField(buffer, sizeof(buffer), cmd);

    C::mip_pending_cmd pending;
    C::mip_pending_cmd_init_full(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, Cmd::Response::FIELD_DESCRIPTOR, buffer, MIP_FIELD_PAYLOAD_LENGTH_MAX, additionalTime);

    CmdResult result = C::mip_interface_run_command_packet(&device, &packet, &pending);
    if( result != C::MIP_ACK_OK )
        return result;

    size_t responseLength = C::mip_pending_cmd_response_length(&pending);

    return extract(response, buffer, responseLength, 0) ? CmdResult::ACK_OK : CmdResult::STATUS_ERROR;
}


template<class Cmd>
bool startCommand(C::mip_interface& device, C::mip_pending_cmd& pending, const Cmd& cmd, Timeout additionalTime)
{
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    Packet packet = Packet::createFromField(buffer, sizeof(buffer), cmd);

    C::mip_pending_cmd_init_with_timeout(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, additionalTime);

    return C::mip_interface_start_command_packet(&device, &packet, &pending);
}

//template<class Cmd>
//bool startCommand(C::mip_interface& device, C::mip_pending_cmd& pending, const Cmd& cmd, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime)
//{
//    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
//    Packet packet = Packet::createFromField(buffer, sizeof(buffer), cmd);
//
//    C::mip_pending_cmd_init_full(&pending, Cmd::descriptorSet, Cmd::fieldDescriptor, Cmd::Response::fieldDescriptor, responseBuffer, responseBufferSize, additionalTime);
//
//    return C::mip_interface_start_command_packet(&device, &packet, &pending);
//}


} // namespace mscl
