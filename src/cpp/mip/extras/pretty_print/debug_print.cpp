
#include "debug_print.hpp"

#include "basic_formatter.hpp"
#include "packet_printer.hpp"

#include <microstrain/logging.hpp>

#include <sstream>


#if MICROSTRAIN_LOGGING_MAX_LEVEL > MICROSTRAIN_LOG_LEVEL_OFF
#define ENABLE_DEBUG_PRINTING 1
#endif


namespace mip
{

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_pretty_print
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Prints a packet's contents to the log in human-readable format.
///
/// Adds a trailing newline.
///
/// If metadata is available, the names of the descriptor set, fields, and
/// members are printed, along with the values of each member.
/// Otherwise, just the values of descriptors and field payload bytes are
/// printed.
///
///@param packet Packet data to be printed.
///@param level  Logging level (defaults to DEBUG).
///
void logPrintPretty(const mip::PacketView& packet, microstrain_log_level level)
{
#if ENABLE_DEBUG_PRINTING
    if(microstrain_logging_level() >= level)
    {
#if MIP_ENABLE_METADATA
        std::ostringstream           stream;
        mip::printer::BasicFormatter formatter(stream);
        mip::printer::PacketPrinter  printer(formatter);

        printer.formatPacket(packet);

        MICROSTRAIN_LOG_LOG(level, "%s\n", stream.str().c_str());
#else
        MICROSTRAIN_LOG_LOG(level, "Packet(DS=0x%02X){ ", packet.descriptorSet());
        for(FieldView field : packet)
        {
            logPrintPretty(field, level);
            MICROSTRAIN_LOG_LOG(level, " ");
        }
        MICROSTRAIN_LOG_LOG(level, "}\n");
#endif
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prints a field's members to the log in human-readable format.
///
/// Does not add a trailing newline.
///
/// If metadata is available, the names of the field and each parameter are
/// printed, along with the values of each parameter.
/// Otherwise, just the descriptor value and payload bytes are printed.
///
///@param field  Field data to be printed.
///@param level  Logging level (defaults to DEBUG).
///
void logPrintPretty(const mip::FieldView& field, microstrain_log_level level)
{
#if ENABLE_DEBUG_PRINTING
    if(microstrain_logging_level() >= level)
    {
#if MIP_ENABLE_METADATA
        std::ostringstream           stream;
        mip::printer::BasicFormatter formatter(stream);
        mip::printer::PacketPrinter  printer(formatter);

        printer.formatField(field);

        MICROSTRAIN_LOG_LOG(level, "%s\n", stream.str().c_str());
#else
        MICROSTRAIN_LOG_LOG(level, "Field(FD=0x%02X)[", field.fieldDescriptor());
        logPrintBytes(field.payloadSpan(), 0, level);
        MICROSTRAIN_LOG_LOG(level, "]");
#endif
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prints an array of bytes to the log. Does not add a trailing newline.
///
///@param data     Data array to be printed. Nothing printed if empty.
///@param grouping Add a space every N bytes (0 = no spaces).
///@param level    Logging level (defaults to DEBUG).
///
void logPrintBytes(microstrain::Span<const uint8_t> data, unsigned int grouping, microstrain_log_level level)
{
#if ENABLE_DEBUG_PRINTING
    if(microstrain_logging_level() >= level)
    {
        for(unsigned int i=0; i<data.size(); i++)
        {
            bool pad = (grouping>0) && ((i % grouping) != (grouping-1));
            const char* format = pad ? "%02X " : "%02X";
            MICROSTRAIN_LOG_LOG(level, format, data[i]);
        }
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prints a packet's bytes to the log. Adds a trailing newline.
///
///@param packet Packet to be printed. Safe even if packet is not sane/valid.
///@param level  Logging level (defaults to DEBUG).
///
void logPrintBytes(const mip::PacketView& packet, microstrain_log_level level)
{
#if ENABLE_DEBUG_PRINTING
    if(microstrain_logging_level() >= level)
    {
        if(!packet.isSane())
        {
            MICROSTRAIN_LOG_LOG(level, "Invalid packet ");
            logPrintBytes(packet.totalSpan(), 0, level);
            MICROSTRAIN_LOG_LOG(level, "\n");
            return;
        }

        MICROSTRAIN_LOG_LOG(level, "%02X%02X %02X %02X  ", packet.payload()[0], packet.payload()[1], packet.descriptorSet(), packet.payloadLength());
        for(FieldView field : packet)
        {
            MICROSTRAIN_LOG_LOG(level, "%02X%02X ", field.payload()[-2], field.fieldDescriptor());
            logPrintBytes(field.payloadSpan(), 0, level);
        }
        MICROSTRAIN_LOG_LOG(level, "  %04X\n", packet.checksumValue());
    }
#endif
}


} // namespace mip


namespace mip::C
{
extern "C"
{

////////////////////////////////////////////////////////////////////////////////
///@brief Prints a packet's contents to the log in human-readable format.
///
/// Adds a trailing newline.
///
/// If metadata is available, the names of the descriptor set, fields, and
/// members are printed, along with the values of each member.
/// Otherwise, just the values of descriptors and field payload bytes are
/// printed.
///
///@param packet Packet data to be printed.
///@param level  Logging level (defaults to DEBUG).
///
void mip_log_pretty_print_packet(const mip_packet_view* packet, microstrain_log_level level)
{
    assert(packet);
    logPrintPretty(PacketView(*packet), level);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prints a field's members to the log in human-readable format.
///
/// Does not add a trailing newline.
///
/// If metadata is available, the names of the field and each parameter are
/// printed, along with the values of each parameter.
/// Otherwise, just the descriptor value and payload bytes are printed.
///
///@param field  Field data to be printed.
///@param level  Logging level (defaults to DEBUG).
///
void mip_log_pretty_print_field(const mip_field_view* field, microstrain_log_level level)
{
    assert(field);
    logPrintPretty(FieldView(*field), level);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prints an array of bytes to the log. Does not add a trailing newline.
///
///@param data     Data array to be printed. Must not be NULL unless length == 0.
///@param length   Number of bytes in data to be printed. Must be 0 if data is NULL.
///@param grouping Add a space every N bytes (0 = no spaces).
///@param level    Logging level (defaults to DEBUG).
///
void mip_log_print_bytes(const uint8_t* data, size_t length, unsigned int grouping, microstrain_log_level level)
{
    logPrintBytes({data, length}, grouping, level);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prints a packet's bytes to the log. Adds a trailing newline.
///
///@param packet Packet to be printed. May be NULL or not sane/valid/etc.
///@param level  Logging level (defaults to DEBUG).
///
void mip_log_print_packet_bytes(const mip_packet_view* packet, microstrain_log_level level)
{
    if(!packet)
    {
        MICROSTRAIN_LOG_LOG(level, "NULL packet\n");
    }
    else
    {
        logPrintBytes(PacketView(*packet), level);
    }
}

} // extern "C"
} // namespace mip::C

///@}
////////////////////////////////////////////////////////////////////////////////
///