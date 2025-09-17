#pragma once

#include <mip/mip_logging.h>
#include <mip/mip_packet.hpp>


namespace mip
{

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_logging_cpp
///
///@brief MIP logging in C++.
///
///@{

#ifdef MICROSTRAIN_LOGGING_ENABLED

////////////////////////////////////////////////////////////////////////////////
///@brief Format MIP packet bytes to a string.
///
/// The bytes will be grouped by header, field, and checksum.
/// For example, a reset comm speed command would print like this:
/// `75650104 04090501 f2bb`
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///@param packet
///       Packet to be printed. There are no restrictions on the packet view
///       other than it being initialized; even a view where mip_packet_is_sane
///       returns false is allowed.
///
///@returns True if successful.
///@returns False in case of insufficient buffer space.
///
inline bool formatPacketBytes(microstrain::Span<char> buffer, size_t* index, const PacketView& packet)
{
    return mip::C::mip_format_packet_bytes(buffer.data(), buffer.size(), index, &packet);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format a MIP packet to a human-readable string.
///
/// The string will identify the packet descriptor set and the field descriptor
/// and payload for each field in the packet.
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///@param packet
///       Packet to be printed. There are no restrictions on the packet view
///       other than it being initialized; even a view where mip_packet_is_sane
///       returns false is allowed.
///
///@returns True if successful.
///@returns False in case of insufficient buffer space.
///
inline bool formatPacket(microstrain::Span<char> buffer, size_t* index, const PacketView& packet)
{
    return mip::C::mip_format_packet(buffer.data(), buffer.size(), index, &packet);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format a MIP field to a human-readable string.
///
/// The string will identify the field descriptor and payload bytes.
///
///@param buffer
///       Buffer of characters. The size should be the number of characters,
///       including the NULL terminator, that will fit in the buffer. If the
///       pointer is NULL and size is 0, this function will just compute the
///       required buffer size and not write any characters.
///@param buffer_size
///       Number of characters the buffer can hold, including the NULL
///       terminator. Must be 0 if buffer is NULL.
///@param[in,out] index
///       Position in buffer where string data will be written. It will be
///       updated with the new index and will point to the new NULL terminator
///       position. If insufficient space is available in buffer, index will
///       still be updated even if it exceeds buffer_size.
///@param field
///       Field to be printed. There are no restrictions on the field view
///       other than it being initialized; even a view where mip_field_is_valid
///       returns false is allowed.
///
///@returns True if successful.
///@returns False in case of insufficient buffer space.
///
inline bool formatField(microstrain::Span<char> buffer, size_t* index, const FieldView& field)
{
    return mip::C::mip_format_field(buffer.data(), buffer.size(), index, &field);
}



///@copydoc mip::C::mip_log_packet
///
inline void logPacket(const PacketView& packet, microstrain_log_level level)
{
    return mip::C::mip_log_packet(&packet, level);
}

///@copydoc mip::C::mip_log_field
///
inline void logField(const FieldView& field, microstrain_log_level level)
{
    return mip::C::mip_log_field(&field, level);
}

///@copydoc mip::C::mip_log_packet_verbose
///
inline void logPacketVerbose(const PacketView& packet, microstrain_log_level level)
{
    return mip::C::mip_log_packet_verbose(&packet, level);
}

#endif // MICROSTRAIN_LOGGING_ENABLED

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace mip
