#pragma once

#include "mip_packet.hpp"

#include <mip/mip_parser.h>


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a MIP parser.
///
/// See @ref parsing_packets
///
class Parser : public C::mip_parser
{
public:
    ///@copydoc mip::C::mip_parser_init
    Parser(C::mip_packet_callback callback, void* callbackObject, Timeout timeout) { C::mip_parser_init(this, callback, callbackObject, timeout); }
    ///@copydoc mip::C::mip_parser_init
    Parser(bool (*callback)(void*,const PacketView*,Timestamp), void* callbackObject, Timeout timeout) { C::mip_parser_init(this, (C::mip_packet_callback)callback, callbackObject, timeout); }
    /// Construct without a callback.
    explicit Parser(Timeout timeout) { C::mip_parser_init(this, nullptr, nullptr, timeout); }

    void setCallback(C::mip_packet_callback callback, void* callbackObject) { C::mip_parser_set_callback(this, callback, callbackObject); }

    template<class Lambda>
    void setCallback(Lambda lambda);

    template<void (*Callback)(const PacketView&, Timestamp)>
    void setCallback();

    template<void (*Callback)(void*, const PacketView&, Timestamp)>
    void setCallback(void* user=nullptr);

    template<class T, void (*Callback)(T&, const PacketView&, Timestamp)>
    void setCallback(T& object);

    template<class T, void (T::*Callback)(const PacketView&, Timestamp)>
    void setCallback(T& object);

    template<class T, bool (*Callback)(T&, const PacketView&, Timestamp)>
    void setCallback(T& object);

    ///@copydoc mip::C::mip_parser_reset
    void reset() { C::mip_parser_reset(this); }

    ///@copydoc mip::C::mip_parser_parse
    size_t parse(const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp) { return C::mip_parser_parse(this, inputBuffer, inputCount, timestamp); }

    ///@brief Parse packets from a buffer (span version).
    ///@copydetails mip::C::mip_parser_parse
    size_t parse(microstrain::Span<const uint8_t> data, Timestamp timestamp) { return parse(data.data(), data.size(), timestamp); }

    ///@copydoc mip::C::mip_parser_flush
    void flush() { C::mip_parser_flush(this); }

    ///@copybrief mip::C::mip_parser_get_write_ptr
    ///@returns a buffer into which data can be written.
    microstrain::Span<uint8_t> getWritePtr() { uint8_t* ptr; size_t length = C::mip_parser_get_write_ptr(this, &ptr); return {ptr, length}; }

    ///@copydoc mip::C::mip_parser_timeout
    Timeout timeout() const { return C::mip_parser_timeout(this); }
    ///@copydoc mip::C::mip_parser_set_timeout
    void setTimeout(Timeout timeout) { return C::mip_parser_set_timeout(this, timeout); }
};


template<class FunctionOrLambda>
void Parser::setCallback(FunctionOrLambda function)
{
    C::mip_packet_callback callback = [](void* obj, const C::mip_packet_view* packet, Timestamp timestamp)->bool
    {
        return *(static_cast<FunctionOrLambda*>(obj))( mip::PacketView(*packet), timestamp );
    };

    C::mip_parser_set_callback(this, callback, &function);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the packet callback function.
///
/// This version allows binding a free function without any context.
/// Example:
///@code{.cpp}
/// void handlePacket(const PacketRef& packet, Timeout timeout);
/// Parser parser;
/// parser.setCallback<&handlePacket>();
///@endcode
///
///@tparam Callback A pointer to a function to be called when a
///        packet is parsed.
///
template<void (*Callback)(const PacketView&, Timestamp)>
void Parser::setCallback()
{
    C::mip_packet_callback callback = [](void*, const C::mip_packet_view* pkt, Timestamp timestamp)
    {
        (*Callback)(PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, nullptr);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the packet callback function.
///
/// This version allows binding a free function with a void pointer.
/// Example:
///@code{.cpp}
/// void handlePacket(void* user, const PacketRef& packet, Timeout timeout);
/// Parser parser;
/// parser.setCallback<&handlePacket>(&userdata);
///@endcode
///
///@tparam Callback A pointer to a function to be called when a
///        packet is parsed.
///
///@param user
///       This will be passed to the callback function for context.
///
template<void (*Callback)(void*, const PacketView&, Timestamp)>
void Parser::setCallback(void* user)
{
    C::mip_packet_callback callback = [](void* user, const C::mip_packet_view* pkt, Timestamp timestamp)
    {
        (*Callback)(user, PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, user);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the packet callback function.
///
/// This version allows binding a free function with context instead of a C-style callback.
/// Example:
///@code{.cpp}
/// struct Context { /* ... */ };
/// void handlePacket(Context& context, const PacketRef& packet, Timeout timeout);
/// Context context;
/// Parser parser;
/// parser.setCallback<Context, &handlePacket>(context);
///@endcode
///
///@tparam T        Type of object.
///@tparam Callback A pointer to a function to be called when a
///        packet is parsed.
///
///@param object
///       This will be passed to the callback function for context.
///
template<class T, void (*Callback)(T& object, const PacketView&, Timestamp)>
void Parser::setCallback(T& object)
{
    C::mip_packet_callback callback = [](void* obj, const C::mip_packet_view* pkt, Timestamp timestamp)->void
    {
        (*Callback)(*static_cast<T*>(obj), PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, &object);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the packet callback function.
///
/// This version allows binding a member function instead of a C-style callback.
/// Example:
///@code{.cpp}
/// struct MyClass
/// {
///     bool handlePacket(const PacketRef& packet, Timeout timeout);
/// };
/// MyClass myInstance;
/// Parser parser;
/// parser.setCallback<MyClass, &MyClass::handlePacket>(myInstance);
///@endcode
///
///@tparam T Class type containing the member function to be called.
///@tparam Callback A pointer to a member function on a T to be called when a
///        packet is parsed.
///
///@param object
///       Instance of T to call the callback.
///
template<class T, void (T::*Callback)(const PacketView&, Timestamp)>
void Parser::setCallback(T& object)
{
    C::mip_packet_callback callback = +[](void* obj, const C::mip_packet_view* pkt, Timestamp timestamp)->bool
    {
        return (static_cast<T*>(obj)->*Callback)(PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, &object);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MIP Parser
///
/// This version allows binding a nonmember function instead of a C-style callback.
/// Example:
///@code{.cpp}
/// struct MyData {};
/// bool handlePacket(MyData& self, const PacketRef& packet, Timeout timeout);
/// MyData myInstance;
/// Parser parser<MyData, &handlePacket>(myInstance);
///@endcode
///
///@tparam T Class type containing the member function to be called.
///@tparam Callback A pointer to a nonmember function taking a T to be called when a
///        packet is parsed.
///
///@param object
///       Instance of T to call the callback.
///
template<class T, bool (*Callback)(T&, const PacketView&, Timestamp)>
void Parser::setCallback(T& object)
{
    C::mip_packet_callback callback = +[](void* obj, const C::mip_packet_view* pkt, Timestamp timestamp)->bool
    {
        return (*Callback)(*static_cast<T*>(obj), PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, &object);
}

///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
