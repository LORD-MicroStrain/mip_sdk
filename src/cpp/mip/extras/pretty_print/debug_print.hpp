#pragma once

#include <microstrain/logging.hpp>
#include <mip/mip_packet.hpp>

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip
///@{
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_pretty_print  Logging and formatting API for MIP packets.
///
///@brief This module is used for debugging and inspection of MIP packets..
///
///@note These functions do nothing if logging is disabled or set at a level
///      numerically lower than the 'level' parameter. Ensure that the following are
///      configured properly:
///@li MICROSTRAIN_ENABLE_LOGGING is ON
///@li MICROSTRAIN_LOGGING_MAX_LEVEL is greater than MICROSTRAIN_LOG_LEVEL_OFF
///@li MICROSTRAIN_LOG_INIT has been called with an appropriate log level
///@li The 'level' parameter is greater than or equal to the level given to
///    MICROSTRAIN_LOG_INIT
///@li The log callback prints somewhere visible, i.e. to stdout or a file.
///
///@}


namespace mip
{

void logPrintPretty(const mip::PacketView& packet, microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);
void logPrintPretty(const mip::FieldView& field,   microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);

void logPrintBytes(microstrain::Span<const uint8_t> data, unsigned int grouping, microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);
void logPrintBytes(const mip::PacketView& packet, microstrain_log_level level = MICROSTRAIN_LOG_LEVEL_DEBUG);

}  // namespace mip


namespace mip::C
{
extern "C"
{

void mip_log_pretty_print_packet(const mip_packet_view* packet, microstrain_log_level level);
void mip_log_pretty_print_field(const mip_field_view* field, microstrain_log_level level);

void mip_log_print_bytes(const uint8_t* data, size_t length, unsigned int grouping, microstrain_log_level level);
void mip_log_print_packet_bytes(const mip_packet_view* packet, microstrain_log_level level);

} // extern "C"
} // namespace mip::C
