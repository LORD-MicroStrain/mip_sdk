#include "mip/mip_interface.hpp"

#include <microstrain/logging.hpp>
#include <microstrain/connections/connection.hpp>

namespace mip
{
    void Dispatcher::addHandler(DispatchHandler& _handler)
    {
        C::mip_dispatcher_add_handler(this, &_handler);
    }

    void Dispatcher::removeHandler(DispatchHandler& _handler)
    {
        C::mip_dispatcher_remove_handler(this, &_handler);
    }

    void Dispatcher::removeAllHandlers()
    {
        C::mip_dispatcher_remove_all_handlers(this);
    }

    bool Interface::sendCallback(const C::mip_interface* _device, const uint8_t* _buffer, const size_t _byteCount,
        size_t* _bytesWrittenOut)
    {
        if (!_device)
        {
            assert(false);

            return false;
        }

        const Interface& device = static_cast<const Interface&>(*_device);

        return device.mSendCallback &&
            device.mSendCallback(device, _buffer, _byteCount, *_bytesWrittenOut);
    }

    bool Interface::connectionSendCallback(const C::mip_interface* _device, const uint8_t* _buffer,
        const size_t _byteCount, size_t* _bytesWrittenOut)
    {
        if (!_device)
        {
            assert(false);

            return false;
        }

        microstrain::connections::Connection* connection =
            static_cast<microstrain::connections::Connection*>(_device->connection);

        return connection &&
            connection->write(_buffer, _byteCount, *_bytesWrittenOut);
    }

    bool Interface::receiveCallback(const mip_interface* _device, uint8_t* _buffer, const size_t _byteCount,
        const uint32_t _waitTime, size_t* _bytesReadOut, microstrain::EmbeddedTimestamp* _timestampOut,
        const bool _fromCommand)
    {
        if (!_device)
        {
            assert(false);

            return false;
        }

        const Interface& device = static_cast<const Interface&>(*_device);

        return device.mReceiveCallback &&
            device.mReceiveCallback(device, _buffer, _byteCount, _waitTime, _bytesReadOut, *_timestampOut,
                _fromCommand);
    }

    bool Interface::connectionReceiveCallback(const mip_interface* _device, uint8_t* _buffer, const size_t _byteCount,
        const uint32_t _waitTime, size_t* _bytesReadOut, microstrain::EmbeddedTimestamp* _timestampOut,
        const bool _fromCommand)
    {
        (void)_fromCommand;

        if (!_device)
        {
            assert(false);

            return false;
        }

        microstrain::connections::Connection* connection =
            static_cast<microstrain::connections::Connection*>(_device->connection);

        return connection &&
            connection->read(_buffer, _byteCount, _waitTime, *_bytesReadOut, *_timestampOut);
    }

    bool Interface::updateCallback(mip_interface* _device, const Timeout _waitTime, const bool _fromCommand)
    {
        if (!_device)
        {
            assert(false);

            return false;
        }

        Interface& device = static_cast<Interface&>(*_device);

        return device.mUpdateCallback &&
            device.mUpdateCallback(device, _waitTime, _fromCommand);
    }

    Interface::Interface(const Timeout _parseTimeout, const Timeout _baseReplyTimeout)
    {
        mip_interface_init(this, _parseTimeout, _baseReplyTimeout, nullptr, nullptr, &C::mip_interface_default_update,
            nullptr, nullptr);
    }

    Interface::Interface(microstrain::connections::Connection& _connection, const Timeout _parseTimeout,
        const Timeout _baseReplyTimeout)
    {
        mip_interface_init(this, _parseTimeout, _baseReplyTimeout, connectionSendCallback, connectionReceiveCallback,
            C::mip_interface_default_update, &_connection, nullptr);
    }

    Interface::~Interface()
    {
        if (connection())
        {
            if (connection()->isConnected())
            {
                connection()->disconnect();
            }

            mip_interface::connection = nullptr;
        }
    }

    C::mip_send_callback Interface::sendFunction() const
    {
        return mip_interface_send_function(this);
    }

    C::mip_recv_callback Interface::recvFunction() const
    {
        return mip_interface_recv_function(this);
    }

    C::mip_update_callback Interface::updateFunction() const
    {
        return mip_interface_update_function(this);
    }

    void Interface::setSendFunction(const SendCallback _sendCallback)
    {
        mSendCallback = std::move(_sendCallback);

        mip_interface_set_send_function(this, sendCallback);
    }

    void Interface::setRecvFunction(const ReceiveCallback _receiveCallback)
    {
        mReceiveCallback = std::move(_receiveCallback);

        mip_interface_set_recv_function(this, receiveCallback);
    }

    void Interface::setUpdateFunction(const UpdateCallback _updateCallback)
    {
        mUpdateCallback = std::move(_updateCallback);

        mip_interface_set_update_function(this, updateCallback);
    }

    void Interface::setUserPointer(void* _userData)
    {
        mip_interface_set_user_pointer(this, _userData);
    }

    void* Interface::userPointer() const
    {
        return mip_interface_user_pointer(this);
    }

    microstrain::connections::Connection* Interface::connection() const
    {
        return static_cast<microstrain::connections::Connection*>(mip_interface_connection_pointer(this));
    }

    Timeout Interface::baseReplyTimeout() const
    {
        return mip_cmd_queue_base_reply_timeout(&cmdQueue());
    }

    void Interface::setBaseReplyTimeout(const Timeout _timeout)
    {
        mip_cmd_queue_set_base_reply_timeout(&cmdQueue(), _timeout);
    }

    Parser& Interface::parser()
    {
        return *static_cast<Parser*>(mip_interface_parser(this));
    }

    const Parser& Interface::parser() const
    {
        return const_cast<Interface*>(this)->parser();
    }

    CmdQueue& Interface::cmdQueue()
    {
        return *static_cast<CmdQueue*>(mip_interface_cmd_queue(this));
    }

    const CmdQueue& Interface::cmdQueue() const
    {
        return const_cast<Interface*>(this)->cmdQueue();
    }

    bool Interface::sendToDevice(const uint8_t* _data, const size_t _byteCount, size_t& _bytesWrittenOut) const
    {
        return mip_interface_send_to_device(this, _data, _byteCount, &_bytesWrittenOut);
    }

    bool Interface::sendToDevice(const PacketView& _packet, size_t& _bytesWrittenOut) const
    {
        return sendToDevice(_packet.pointer(), _packet.totalLength(), _bytesWrittenOut);
    }

    bool Interface::update(const Timeout _waitTime, const bool _fromCommand)
    {
        return mip_interface_update(this, _waitTime, _fromCommand);
    }

    bool Interface::defaultUpdate(const Timeout _waitTime, const bool _fromCommand)
    {
        return mip_interface_default_update(this, _waitTime, _fromCommand);
    }

    bool Interface::defaultUpdate(uint8_t* _data, const size_t _bufferSize, const Timeout _waitTime,
        const bool _fromCommand)
    {
        return mip_interface_default_update_ext_buffer(this, _data, _bufferSize, _waitTime, _fromCommand);
    }

    void Interface::inputBytes(const uint8_t* _data, const size_t _bufferSize, const Timestamp _timestamp)
    {
        mip_interface_input_bytes_from_device(this, _data, _bufferSize, _timestamp);
    }

    void Interface::inputPacket(const PacketView& _packet, const Timestamp _timestamp)
    {
        mip_interface_input_packet_from_device(this, &_packet, _timestamp);
    }

    void Interface::updateTime(const Timestamp _timestamp)
    {
        mip_interface_update_time(this, _timestamp);
    }

    CmdResult Interface::waitForReply(PendingCmd& _pendingCommand)
    {
        return mip_interface_wait_for_reply(this, &_pendingCommand);
    }

    void Interface::registerPacketCallback(DispatchHandler& _dispatchHandler, const uint8_t _descriptorSet,
        const bool _afterFields, const DispatchPacketCallback _dispatchPacketCallback, void* _userData)
    {
        mip_interface_register_packet_callback(this, &_dispatchHandler, _descriptorSet, _afterFields,
            _dispatchPacketCallback, _userData);
    }

    void Interface::registerFieldCallback(DispatchHandler& _dispatchHandler, const uint8_t _descriptorSet,
        const uint8_t _fieldDescriptor, const DispatchFieldCallback _dispatchFieldCallback, void* _userData)
    {
        mip_interface_register_field_callback(this, &_dispatchHandler, _descriptorSet, _fieldDescriptor,
            _dispatchFieldCallback, _userData);
    }
} // namespace mip
