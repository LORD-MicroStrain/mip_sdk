#pragma once

#include <microstrain/logging.hpp>
#include <mip/mip_packet.hpp>

namespace mip
{

void prettyPrint(const mip::PacketView& packet, microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);
void prettyPrint(const mip::FieldView& field,   microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);

}  // namespace mip


namespace mip::C
{
extern "C"
{

void mip_pretty_print_packet(const mip_packet_view* packet, microstrain_log_level level);
void mip_pretty_print_field(const mip_field_view* field, microstrain_log_level level);

} // extern "C"
} // namespace mip::C
