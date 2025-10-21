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

    template<class T, bool (T::*Callback)(const PacketView&, Timestamp)>
    void setCallback(T& object);

    template<class T, bool (*Callback)(T&, const PacketView&, Timestamp)>
    void setCallback(T& object);

    ///@copydoc mip::C::mip_parser_reset
    void reset() { C::mip_parser_reset(this); }

    ///@copydoc mip::C::mip_parser_parse
    size_t parse(const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp) { return C::mip_parser_parse(this, inputBuffer, inputCount, timestamp); }

    ///@brief Parse packets from a buffer (U8ArrayView version).
    ///@copydetails mip::C::mip_parser_parse
    size_t parse(microstrain::ConstU8ArrayView data, Timestamp timestamp) { return parse(data.data(), data.size(), timestamp); }

    ///@copydoc mip::C::mip_parser_flush
    void flush() { C::mip_parser_flush(this); }

    ///@copybrief mip::C::mip_parser_get_write_ptr
    ///@returns a buffer into which data can be written.
    microstrain::U8ArrayView getWritePtr() { uint8_t* ptr; size_t length = C::mip_parser_get_write_ptr(this, &ptr); return {ptr, length}; }

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
        FunctionOrLambda& func = *(static_cast<FunctionOrLambda*>(obj));
        return func( mip::PacketView(*packet), timestamp );
    };

    C::mip_parser_set_callback(this, callback, &function);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MIP Parser
///
/// This version allows binding a member function instead of a C-style callback.
/// Example:
///@code{.cpp}
/// struct MyClass
/// {
///     bool handlePacket(const PacketRef& packet, Timeout timeout);
/// };
/// MyClass myInstance;
/// Parser parser<MyClass, &MyClass::handlePacket>(myInstance);
///@endcode
///
///@tparam T Class type containing the member function to be called.
///@tparam Callback A pointer to a member function on a T to be called when a
///        packet is parsed.
///
///@param object
///       Instance of T to call the callback.
///
template<class T, bool (T::*Callback)(const PacketView&, Timestamp)>
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
