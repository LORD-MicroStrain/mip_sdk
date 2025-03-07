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
    Parser(void (*callback)(void*,const PacketView*,Timestamp), void* callbackObject, Timeout timeout) { C::mip_parser_init(this, (C::mip_packet_callback)callback, callbackObject, timeout); }

    Parser(Timeout timeout) { C::mip_parser_init(this, nullptr, nullptr, timeout); }

    template<class T, bool (T::*Callback)(const PacketView&, Timestamp)>
    void setCallback(T& object);

    ///@copydoc mip::C::mip_parser_reset
    void reset() { C::mip_parser_reset(this); }

    ///@copydoc mip::C::mip_parser_parse
    void parse(const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp) { return C::mip_parser_parse(this, inputBuffer, inputCount, timestamp); }

    ///@brief Parse packets from a buffer (span version).
    ///@copydetails mip::C::mip_parser_parse
    void parse(microstrain::Span<const uint8_t> data, Timestamp timestamp) { return parse(data.data(), data.size(), timestamp); }

    ///@copydoc mip::C::mip_parser_timeout
    Timeout timeout() const { return C::mip_parser_timeout(this); }
    ///@copydoc mip::C::mip_parser_set_timeout
    void setTimeout(Timeout timeout) { return C::mip_parser_set_timeout(this, timeout); }
};


////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MIP Parser
///
/// This version allows binding a member function instead of a C-style callback.
/// Example:
///@code{.cpp}
/// struct MyClass
/// {
///     void handlePacket(const PacketRef& packet, Timeout timeout);
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
    C::mip_packet_callback callback = [](void* obj, const C::mip_packet_view* pkt, Timestamp timestamp)->void
    {
        (static_cast<T*>(obj)->*Callback)(PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, &object);
}

///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
