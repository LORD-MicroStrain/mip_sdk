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
    Parser(uint8_t* buffer, size_t bufferSize, C::mip_packet_callback callback, void* callbackObject, Timeout timeout) { C::mip_parser_init(this, buffer, bufferSize, callback, callbackObject, timeout); }
    ///@copydoc mip::C::mip_parser_init
    Parser(uint8_t* buffer, size_t bufferSize, bool (*callback)(void*,const PacketView*,Timestamp), void* callbackObject, Timeout timeout) { C::mip_parser_init(this, buffer, bufferSize, (C::mip_packet_callback)callback, callbackObject, timeout); }

    Parser(uint8_t* buffer, size_t bufferSize, Timeout timeout) { C::mip_parser_init(this, buffer, bufferSize, nullptr, nullptr, timeout); }

    template<class T, bool (T::*Callback)(const PacketView&, Timestamp)>
    void setCallback(T& object);

    ///@copydoc mip::C::mip_parser_reset
    void reset() { C::mip_parser_reset(this); }

    ///@copydoc mip::C::mip_parser_parse
    size_t parse(const uint8_t* inputBuffer, size_t inputCount, Timestamp timestamp, unsigned int maxPackets=0) { return C::mip_parser_parse(this, inputBuffer, inputCount, timestamp, maxPackets); }

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
    C::mip_packet_callback callback = [](void* obj, const C::mip_packet_view* pkt, Timestamp timestamp)->bool
    {
        return (static_cast<T*>(obj)->*Callback)(PacketView(pkt), timestamp);
    };

    C::mip_parser_set_callback(this, callback, &object);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Read data from a source into the internal parsing buffer.
///
///@tparam Function
/// A function-like object with the following signature:
/// `bool read(size_t maxCount, size_t* count_out, Timestamp* timestamp_out);`
/// The parameters are as follows:
/// @li buffer - Buffer into which to write data.
/// @li maxCount - The maximum number of bytes to read.
/// @li count_out - Updated with the number of bytes actually read.
/// @li timestamp_out - Updated with the timestamp of the data.
///
///@param parser
///
///@param reader
///       A callback function, lambda, or similar which will read data into the
///       buffer and capture the timestamp. It should return true if successful
///       or false otherwise. If it returns false, parsing is skipped. The read
///       function may also throw an exception instead of returning false.
///
///@param maxPackets
///       Maximum number of packets to parse, just like for Parser::parse().
///
///@return Same as the return value of reader.
///
template<class Function>
bool parseMipDataFromSource(C::mip_parser& parser, Function reader, size_t maxPackets)
{
    uint8_t* ptr;
    size_t maxCount = C::mip_parser_get_write_ptr(&parser, &ptr);

    size_t count;
    Timestamp timestamp;
    if( !reader(ptr, maxCount, &count, &timestamp) )
        return false;

    assert(count <= maxCount);

    C::mip_parser_process_written(&parser, count, timestamp, maxPackets);

    return true;
}

///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
