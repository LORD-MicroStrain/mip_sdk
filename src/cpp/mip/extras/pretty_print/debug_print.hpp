#pragma once

#include <microstrain/logging.hpp>
#include <mip/mip_packet.hpp>

namespace mip
{

void debugPrint(const mip::PacketView& packet, microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);
void debugPrint(const mip::FieldView& field,   microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);

}  // namespace mip


namespace mip::C
{
extern "C"
{

void mip_debug_print_packet(const mip::C::mip_packet_view* packet);
void mip_debug_print_field(const mip::C::mip_field_view* field);

} // extern "C"
} // namespace mip::C
