
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

void debugPrint(const mip::PacketView& packet, microstrain_log_level level)
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

void debugPrint(const mip::FieldView& field, microstrain_log_level level)
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

void mip_debug_print_packet(const mip::C::mip_packet_view* packet)
{
    assert(packet);
    debugPrint(PacketView(*packet));
}
void mip_debug_print_field(const mip::C::mip_field_view* field)
{
    assert(field);
    debugPrint(FieldView(*field));
}

} // extern "C"
} // namespace mip::C
