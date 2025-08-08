
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

void prettyPrint(const mip::PacketView& packet, microstrain_log_level level)
{
#if ENABLE_DEBUG_PRINTING
    if(microstrain_logging_level() >= level)
    {
        std::ostringstream           stream;
        mip::printer::BasicFormatter formatter(stream);
        mip::printer::PacketPrinter  printer(formatter);

        printer.formatPacket(packet);

        MICROSTRAIN_LOG_LOG(level, "%s\n", stream.str().c_str());
    }
#endif
}

void prettyPrint(const mip::FieldView& field, microstrain_log_level level)
{
#if ENABLE_DEBUG_PRINTING
    if(microstrain_logging_level() >= level)
    {
        std::ostringstream           stream;
        mip::printer::BasicFormatter formatter(stream);
        mip::printer::PacketPrinter  printer(formatter);

        printer.formatField(field);

        MICROSTRAIN_LOG_LOG(level, "%s\n", stream.str().c_str());
    }
#endif
}

} // namespace mip


namespace mip::C
{
extern "C"
{

void mip_pretty_print_packet(const mip_packet_view* packet, microstrain_log_level level)
{
    assert(packet);
    prettyPrint(PacketView(*packet), level);
}
void mip_pretty_print_field(const mip_field_view* field, microstrain_log_level level)
{
    assert(field);
    prettyPrint(FieldView(*field), level);
}

} // extern "C"
} // namespace mip::C
