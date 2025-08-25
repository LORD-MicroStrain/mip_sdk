
#include "mip_logging.h"

#include <microstrain/strings.h>
#include <mip/mip_offsets.h>


#ifdef MICROSTRAIN_ENABLE_LOGGING

//
// Formatting
//

////////////////////////////////////////////////////////////////////////////////
///@brief Format MIP packet bytes to a string.
///
/// The bytes will be grouped by header, field, and checksum.
/// For example, a reset comm speed command would print like this:
/// `75650104 04090501 f2bb`
///
///@param buffer
///       Pointer to character buffer where string data will be stored.
///       If this is NULL, this function will only compute the required buffer
///       size (set buffer_size = 0 in this case).
///@param buffer_size
///       Size of the buffer. Up to buffer_size-1 chars will be written, plus
///       a NULL terminator. Must be 0 if buffer is NULL.
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
bool mip_format_packet_bytes(char* buffer, size_t buffer_size, size_t* index, const mip_packet_view* packet)
{
    if(!mip_packet_is_sane(packet))
        return microstrain_strfmt_bytes(buffer, buffer_size, index, mip_packet_pointer(packet), mip_packet_buffer_size(packet), 0);

    bool ok = true;
    // ok &= microstrain_strcat_l(buffer, buffer_size, index, "[");
    ok &= microstrain_strfmt(
        buffer, buffer_size, index, "%02X%02X%02X%02X",
        mip_packet_pointer(packet)[MIP_INDEX_SYNC1],
        mip_packet_pointer(packet)[MIP_INDEX_SYNC2],
        mip_packet_pointer(packet)[MIP_INDEX_DESCSET],
        mip_packet_pointer(packet)[MIP_INDEX_LENGTH]
    );

    for(mip_field_view field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field))
    {
        microstrain_strfmt(
            buffer, buffer_size, index, " %02X%02X",
            MIP_FIELD_HEADER_LENGTH+ mip_field_payload_length(&field),
            mip_field_field_descriptor(&field)
        );

        microstrain_strfmt_bytes(buffer, buffer_size, index, mip_field_payload(&field), mip_field_payload_length(&field), 0);
    }

    ok &= microstrain_strfmt(buffer, buffer_size, index, " %04X", mip_packet_checksum_value(packet));

    return ok;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Format a MIP packet to a human-readable string.
///
/// The string will identify the packet descriptor set and the field descriptor
/// and payload for each field in the packet.
///
///@param buffer
///       Pointer to character buffer where string data will be stored.
///       If this is NULL, this function will only compute the required buffer
///       size (set buffer_size = 0 in this case).
///@param buffer_size
///       Size of the buffer. Up to buffer_size-1 chars will be written, plus
///       a NULL terminator. Must be 0 if buffer is NULL.
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
bool mip_format_packet(char* buffer, size_t buffer_size, size_t* index, const mip_packet_view* packet)
{
    bool ok = true;

    if(!mip_packet_is_sane(packet))
    {
        ok &= microstrain_strfmt(buffer, buffer_size, index, "Invalid Packet: [");
        ok &= mip_format_packet_bytes(buffer, buffer_size, index, packet);
        ok &= microstrain_strcat_l(buffer, buffer_size, index, "]");
        return ok;
    }

    ok &= microstrain_strfmt(
        buffer, buffer_size, index, "Packet(DS=0x%02X){",
        mip_packet_descriptor_set(packet)
    );

    for(mip_field_view field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field))
    {
        ok &= microstrain_strcat_l(buffer, buffer_size, index, " ");
        ok &= mip_format_field(buffer, buffer_size, index, &field);
    }

    const uint16_t checksum_value = mip_packet_checksum_value(packet);
    const uint16_t checksum_check = mip_packet_compute_checksum(packet);

    if(checksum_value == checksum_check)
        ok &= microstrain_strfmt(buffer, buffer_size, index, " }", checksum_value);
    else
        ok &= microstrain_strfmt(buffer, buffer_size, index, " BAD_CHECKSUM(%04X!=%04X) }", checksum_value, checksum_check);

    return ok;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Format a MIP field to a human-readable string.
///
/// The string will identify the field descriptor and payload bytes.
///
///@param buffer
///       Pointer to character buffer where string data will be stored.
///       If this is NULL, this function will only compute the required buffer
///       size (set buffer_size = 0 in this case).
///@param buffer_size
///       Size of the buffer. Up to buffer_size-1 chars will be written, plus
///       a NULL terminator. Must be 0 if buffer is NULL.
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
bool mip_format_field(char* buffer, size_t buffer_size, size_t* index, const mip_field_view* field)
{
    bool ok = true;

    if(!mip_field_is_valid(field))
    {
        ok &= microstrain_strcat_l(buffer, buffer_size, index, "Field(INVALID)");
    }
    else
    {
        ok &= microstrain_strfmt(buffer, buffer_size, index, "Field(FD=0x%02X)[", mip_field_field_descriptor(field));
        ok &= microstrain_strfmt_bytes(buffer, buffer_size, index, mip_field_payload(field), mip_field_payload_length(field), 0);
        ok &= microstrain_strcat_l(buffer, buffer_size, index, "]");
    }

    return ok;
}

//
// Logging
//

////////////////////////////////////////////////////////////////////////////////
///@brief Print a MIP packet to the microstrain logging system.
///
///@see mip::C::mip_format_packet
///
///@param packet
///       Packet to be printed. There are no restrictions on the packet view
///       other than it being initialized; even a view where mip_packet_is_sane
///       returns false is allowed.
///@param level
///       Logging level passed to the microstrain log callback. If the current
///       log level is less than this value, nothing is printed.
///
void mip_log_packet(const mip_packet_view* packet, microstrain_log_level level)
{
    if(microstrain_logging_level() < level)
        return;

    // Buffer must be large enough to hold 3 digits per payload byte,
    // plus extra if there are more fields, due to formatting around
    // the field header.
    char buffer[2048];
    size_t index = 0;

    mip_format_packet(buffer, sizeof(buffer), &index, packet);

    microstrain_logging_log(level, "%s\n", buffer);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Print a MIP field to the microstrain logging system.
///
///@see mip::C::mip_format_field
///
///@param field
///       Field to be printed. There are no restrictions on the field view
///       other than it being initialized; even a view where mip_field_is_valid
///       returns false is allowed.
///@param level
///       Logging level passed to the microstrain log callback. If the current
///       log level is less than this value, nothing is printed.
///
void mip_log_field(const mip_field_view* field, microstrain_log_level level)
{
    if(microstrain_logging_level() < level)
        return;

    // Buffer must be large enough to hold 3 digits per payload byte,
    // plus a little extra for the header.
    char buffer[1024];
    size_t index = 0;

    mip_format_field(buffer, sizeof(buffer), &index, field);

    microstrain_logging_log(level, "%s\n", buffer);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Prints the details of a MIP packet to the microstrain logging system.
///
///@param packet
///       Packet to be printed. There are no restrictions on the packet view
///       other than it being initialized; even a view where mip_packet_is_sane
///       returns false is allowed.
///@param level
///       Logging level passed to the microstrain log callback. If the current
///       log level is less than this value, nothing is printed.
///
void mip_log_packet_verbose(const mip_packet_view* packet, microstrain_log_level level)
{
    if(microstrain_logging_level() < level)
        return;

    char byte_buffer[MIP_PACKET_LENGTH_MAX*2];
    size_t index = 0;
    mip_format_packet_bytes(byte_buffer, sizeof(byte_buffer), &index, packet);

    if(!mip_packet_is_sane(packet))
    {
        microstrain_logging_log(level, "Invalid Packet: [%s]\n", byte_buffer);

        return;
    }

    const bool valid = mip_packet_compute_checksum(packet) == mip_packet_checksum_value(packet);

    microstrain_logging_log(level, "Packet: [%s]\n", byte_buffer);

    // Print the packet details.
    microstrain_logging_log(level, "%4s%-20s = %u\n",          " ", "Total Length", mip_packet_total_length(packet));
    // microstrain_logging_log(level, "%4s%-20s = [%s]\n",        " ", "Raw Packet", byte_buffer);
    microstrain_logging_log(level, "%4s%-20s = 0x%02X\n",      " ", "MIP SYNC1", mip_packet_pointer(packet)[0]);
    microstrain_logging_log(level, "%4s%-20s = 0x%02X\n",      " ", "MIP SYNC2", mip_packet_pointer(packet)[1]);
    microstrain_logging_log(level, "%4s%-20s = 0x%02X\n",      " ", "Descriptor Set", mip_packet_descriptor_set(packet));
    microstrain_logging_log(level, "%4s%-20s = %u (0x%02X)\n", " ", "Payload Length", mip_packet_payload_length(packet), mip_packet_payload_length(packet));
    microstrain_logging_log(level, "%4s%-20s = 0x%04X (%s)\n", " ", "Checksum", mip_packet_checksum_value(packet), valid ? "valid" : "INVALID");

    mip_field_view field = mip_field_first_from_packet(packet);
    if(!mip_field_is_valid(&field))
    {
        microstrain_logging_log(level, "%4sNo Fields\n", " ");

        return;
    }

    unsigned int i=0;
    do
    {
        ++i;

        const uint16_t field_length = MIP_FIELD_HEADER_LENGTH+mip_field_payload_length(&field);

        index = 0;
        microstrain_strfmt_bytes(byte_buffer, sizeof(byte_buffer), &index, mip_field_payload(&field), mip_field_payload_length(&field), 0);

        microstrain_logging_log(
            level, "%4sField %u: [%02X%02X %s]\n", " ",
            i,
            field_length,
            mip_field_field_descriptor(&field),
            byte_buffer
        );

        // Print field info.
        microstrain_logging_log(level, "%8s%-16s = %u (0x%02X)\n", " ", "Field Length", field_length, field_length);
        microstrain_logging_log(level, "%8s%-16s = 0x%02X\n",      " ", "Field Descriptor", mip_field_field_descriptor(&field));
        // microstrain_logging_log(level, "%8s%-16s = [%s]\n",        " ", "Raw Payload", byte_buffer);

    } while(mip_field_next(&field));

    microstrain_logging_log(level, "\n");
}


#endif // MICROSTRAIN_ENABLE_LOGGING
