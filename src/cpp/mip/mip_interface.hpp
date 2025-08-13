#pragma once

#include "mip/mip_cmdqueue.hpp"
#include "mip/mip_descriptors.h"
#include "mip/mip_interface.h"
#include "mip/mip_parser.hpp"

#include <cassert>

namespace microstrain
{
    namespace connections
    {
        class Connection;
    } // namespace connections
} // namespace microstrain

namespace mip
{
    ////////////////////////////////////////////////////////////////////////////////
    /// @addtogroup mip_cpp
    /// @{
    ///

    using DispatchPacketCallback = C::mip_dispatch_packet_callback;
    using DispatchFieldCallback  = C::mip_dispatch_field_callback;
    using DispatchHandler        = C::mip_dispatch_handler;

    struct Dispatcher : public C::mip_dispatcher
    {
        enum : uint8_t
        {
            ANY_DATA_SET   = C::MIP_DISPATCH_ANY_DATA_SET,
            ANY_DESCRIPTOR = C::MIP_DISPATCH_ANY_DESCRIPTOR,
        };

        void addHandler(DispatchHandler& _handler);
        void removeHandler(DispatchHandler& _handler);

        void removeAllHandlers();
    };

    class Interface;

    typedef bool (*SendCallback)(const Interface& _device, const uint8_t* _buffer, const size_t _byteCount,
        size_t& _bytesWrittenOut);

    typedef bool (*ReceiveCallback)(const Interface& _device, uint8_t* _buffer, const size_t _byteCount,
        const Timeout _waitTime, size_t* _bytesReadOut, microstrain::EmbeddedTimestamp& _timestampOut,
        const bool _fromCommand);

    typedef bool (*UpdateCallback)(Interface& _device, const Timeout _waitTime, const bool _fromCommand);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Represents a connected MIP device.
    ///
    class Interface : public C::mip_interface
    {
        SendCallback    mSendCallback    = nullptr;
        ReceiveCallback mReceiveCallback = nullptr;
        UpdateCallback  mUpdateCallback  = nullptr;

        static bool sendCallback(const mip_interface* _device, const uint8_t* _buffer, const size_t _byteCount,
            size_t* _bytesWrittenOut);

        static bool connectionSendCallback(const mip_interface* _device, const uint8_t* _buffer,
            const size_t _byteCount, size_t* _bytesWrittenOut);

        static bool receiveCallback(const mip_interface* _device, uint8_t* _buffer, const size_t _byteCount,
            const uint32_t _waitTime, size_t* _bytesReadOut, microstrain::EmbeddedTimestamp* _timestampOut,
            const bool _fromCommand);

        static bool connectionReceiveCallback(const mip_interface* _device, uint8_t* _buffer, const size_t _byteCount,
            const uint32_t _waitTime, size_t* _bytesReadOut, microstrain::EmbeddedTimestamp* _timestampOut,
            const bool _fromCommand);

        static bool updateCallback(mip_interface* _device, const Timeout _waitTime, const bool _fromCommand);

    public:
        ////////////////////////////////////////////////////////////////////////////////
        /// @copybrief mip_interface_init
        ///
        /// The interface callbacks must be assigned separately
        Interface(const Timeout _parseTimeout, const Timeout _baseReplyTimeout);

        ////////////////////////////////////////////////////////////////////////////////
        /// @copybrief mip_interface_init
        ///
        /// @param _connection The connection object used to communicate with the
        ///                    device. This object must exist for the life of the
        ///                    Interface object
        Interface(microstrain::connections::Connection& _connection, const Timeout _parseTimeout,
            const Timeout _baseReplyTimeout);

        Interface(const Interface&)            = delete;
        Interface& operator=(const Interface&) = delete;
        virtual ~Interface();

        C::mip_send_callback sendFunction()     const;
        C::mip_recv_callback recvFunction()     const;
        C::mip_update_callback updateFunction() const;

        void setSendFunction(const SendCallback _sendCallback);
        void setRecvFunction(const ReceiveCallback _receiveCallback);
        void setUpdateFunction(const UpdateCallback _updateCallback);

        ////////////////////////////////////////////////////////////////////////////////
        /// @brief Sets an optional user data pointer which can be retrieved later
        ///
        void setUserPointer(void* _userData);
        void* userPointer() const;
        microstrain::connections::Connection* connection() const;

        Timeout baseReplyTimeout() const;
        void setBaseReplyTimeout(const Timeout _timeout);

        Parser& parser();
        const Parser& parser() const;

        CmdQueue& cmdQueue();
        const CmdQueue& cmdQueue() const;

        bool sendToDevice(const uint8_t* _data, const size_t _byteCount, size_t& _bytesWrittenOut) const;
        bool sendToDevice(const PacketView& _packet, size_t& _bytesWrittenOut) const;
        bool update(const Timeout _waitTime = 0, const bool _fromCommand = false);

        bool defaultUpdate(const Timeout _waitTime = 0, const bool _fromCommand = false);
        bool defaultUpdate(uint8_t* _data, const size_t _bufferSize, const Timeout _waitTime = 0,
            const bool _fromCommand = false);
        void inputBytes(const uint8_t* _data, const size_t _bufferSize, const Timestamp _timestamp);
        void inputPacket(const PacketView& _packet, const Timestamp _timestamp);
        void updateTime(const Timestamp _timestamp);

        CmdResult waitForReply(PendingCmd& _pendingCommand);

        void registerPacketCallback(DispatchHandler& _dispatchHandler, const uint8_t _descriptorSet,
            const bool _afterFields, const DispatchPacketCallback _dispatchPacketCallback, void* _userData);

        void registerFieldCallback(DispatchHandler& _dispatchHandler, const uint8_t _descriptorSet,
            const uint8_t _fieldDescriptor, const DispatchFieldCallback _dispatchFieldCallback,
            void* _userData);

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
        template<void (*Callback)(void*, const PacketView&, const Timestamp)>
        void registerPacketCallback(C::mip_dispatch_handler& handler, const uint8_t descriptorSet,
            const bool afterFields, void* userData = nullptr)
        {
            const C::mip_dispatch_packet_callback callback = [](void* context, const C::mip_packet_view* packet,
                const Timestamp timestamp)->void
            {
                const PacketView packetView(*packet);
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
        template<class Object, void (Object::*Callback)(const PacketView&, const Timestamp)>
        void registerPacketCallback(C::mip_dispatch_handler& handler, const uint8_t descriptorSet,
            const bool afterFields, Object* object)
        {
            const C::mip_dispatch_packet_callback callback = [](void* pointer, const C::mip_packet_view* packet,
                Timestamp timestamp)->void
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
        template<void (*Callback)(void*, const FieldView&, const Timestamp)>
        void registerFieldCallback(C::mip_dispatch_handler& handler, const uint8_t descriptorSet,
            const uint8_t fieldDescriptor, void* userData = nullptr)
        {
            const C::mip_dispatch_field_callback callback = [](void* context, const C::mip_field_view* field,
                const Timestamp timestamp)->void
            {
                const FieldView fieldView(*field);
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
        template<class Object, void (Object::*Callback)(const FieldView& field, const Timestamp)>
        void registerFieldCallback(C::mip_dispatch_handler& handler, const uint8_t descriptorSet,
            const uint8_t fieldDescriptor, Object* object)
        {
            const C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field,
                const Timestamp timestamp)->void
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
        template<class DataField, void (*Callback)(void*, const DataField&, const Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, void* userData = nullptr,
            const uint8_t descriptorSet = DataField::DESCRIPTOR_SET)
        {
            assert(descriptorSet != 0x00);
            if (descriptorSet == 0x00)
            {
                return;
            }

            assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
            if (descriptorSet == 0xFF)
            {
                return;
            }

            const C::mip_dispatch_field_callback callback = [](void* context, const C::mip_field_view* field,
                const Timestamp timestamp)->void
            {
                DataField data;

                const bool ok = FieldView(*field).extract(data);
                assert(ok);
                (void)ok;

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
        template<class DataField, void (*Callback)(void*, const DataField&, const uint8_t, const Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, void* userData = nullptr,
            const uint8_t descriptorSet = DataField::DESCRIPTOR_SET)
        {
            assert(descriptorSet != 0x00);
            if (descriptorSet == 0x00)
            {
                return;
            }

            assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
            if (descriptorSet == 0xFF)
            {
                return;
            }

            const C::mip_dispatch_field_callback callback = [](void* context, const C::mip_field_view* field,
                const Timestamp timestamp)->void
            {
                DataField data;

                const bool ok = FieldView(*field).extract(data);
                assert(ok);
                (void)ok;

                Callback(context, data, C::mip_field_descriptor_set(field), timestamp);
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
        template<class DataField, class Object, void (Object::*Callback)(const DataField&, const Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, Object* object,
            const uint8_t descriptorSet = DataField::DESCRIPTOR_SET)
        {
            assert(descriptorSet != 0x00);
            if (descriptorSet == 0x00)
            {
                return;
            }

            assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
            if (descriptorSet == 0xFF)
            {
                return;
            }

            const C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field,
                const Timestamp timestamp)->void
            {
                DataField data;

                const bool ok = FieldView(*field).extract(data);
                assert(ok);
                (void)ok;

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
        template<class DataField, class Object, void (Object::*Callback)(const DataField&, const uint8_t, const Timestamp)>
        void registerDataCallback(C::mip_dispatch_handler& handler, Object* object,
            const uint8_t descriptorSet = DataField::DESCRIPTOR_SET)
        {
            assert(descriptorSet != 0x00);
            if (descriptorSet == 0x00)
            {
                return;
            }

            assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
            if (descriptorSet == 0xFF)
            {
                return;
            }

            const C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field,
                const Timestamp timestamp)->void
            {
                DataField data;

                const bool ok = FieldView(*field).extract(data);
                assert(ok);
                (void)ok;

                Object* obj = static_cast<Object*>(pointer);
                (obj->*Callback)(data, C::mip_field_descriptor_set(field), timestamp);
            };

            registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, object);
        }

        template<class DataField>
        void registerExtractor(C::mip_dispatch_handler& handler, DataField* field,
            const uint8_t descriptorSet = DataField::DESCRIPTOR_SET)
        {
            assert(descriptorSet != 0x00);
            if (descriptorSet == 0x00)
            {
                return;
            }

            assert(descriptorSet != 0xFF);  // Descriptor set must be specified for shared data.
            if (descriptorSet == 0xFF)
            {
                return;
            }

            const C::mip_dispatch_field_callback callback = [](void* pointer, const C::mip_field_view* field,
                const Timestamp /*timestamp*/)->void
            {
                FieldView(*field).extract(*static_cast<DataField*>(pointer));
            };

            registerFieldCallback(handler, descriptorSet, DataField::FIELD_DESCRIPTOR, callback, field);
        }

        template<class Cmd>
        CmdResult runCommand(const Cmd& _cmd, const Timeout _additionalTime = 0)
        {
            const PacketBuf packet(_cmd);

            C::mip_pending_cmd pending;
            mip_pending_cmd_init_with_timeout(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, _additionalTime);

            return mip_interface_run_command_packet(this, &packet, &pending);
        }

        template<class Cmd, class... Args>
        CmdResult runCommand(Args&&... _args, const Timeout _additionalTime = 0)
        {
            Cmd cmd{std::forward<Args>(_args)...};
            return runCommand(cmd, _additionalTime);
        }

        template<class Cmd>
        CmdResult runCommand(const Cmd& _cmd, typename Cmd::Response& _response, const Timeout _additionalTime = 0)
        {
            PacketBuf packet(_cmd);

            C::mip_pending_cmd pending;
            mip_pending_cmd_init_full(&pending, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, Cmd::Response::FIELD_DESCRIPTOR,
                packet.buffer(), FIELD_PAYLOAD_LENGTH_MAX, _additionalTime);

            const CmdResult result = mip_interface_run_command_packet(this, &packet, &pending);
            if (!result.isAck())
            {
                return result;
            }

            size_t responseLength = mip_pending_cmd_response_length(&pending);

            return extract(_response, packet.buffer(), responseLength, 0) ? CmdResult::ACK_OK : CmdResult::STATUS_ERROR;
        }

        template<class Cmd>
        bool startCommand(PendingCmd& _pendingCmd, const Cmd& _cmd, const Timeout _additionalTime = 0)
        {
            const PacketBuf packet(_cmd);

            mip_pending_cmd_init_with_timeout(&_pendingCmd, Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, _additionalTime);

            return mip_interface_start_command_packet(this, &packet, &_pendingCmd);
        }
    };
    ///
    /// @}
    ////////////////////////////////////////////////////////////////////////////////
} // namespace mip
