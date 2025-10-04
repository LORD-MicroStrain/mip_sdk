#pragma once

#include <microstrain/logging.h>

#include <mip/mip_packet.h>
#include <mip/mip_field.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup microstrain
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_logging  MIP Logging
///
///@brief Functions for printing MIP packets.
///
///@{
///@defgroup mip_logging_c   MIP Logging [C]
///@defgroup mip_logging_cpp MIP Logging [CPP]
///@}
///

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_logging_c
///
///@brief MIP logging in C.
///
///@{


#ifdef MICROSTRAIN_LOGGING_ENABLED

// Format functions

bool mip_format_packet_bytes(char* buffer, size_t buffer_size, size_t* index, const mip_packet_view* packet);

bool mip_format_packet(char* buffer, size_t buffer_size, size_t* index, const mip_packet_view* packet);

bool mip_format_field(char* buffer, size_t buffer_size, size_t* index, const mip_field_view* field);

// Logging functions

void mip_log_packet(const mip_packet_view* packet, microstrain_log_level level);

void mip_log_field(const mip_field_view* field, microstrain_log_level level);

void mip_log_packet_verbose(const mip_packet_view* packet, microstrain_log_level level);

#define MIP_LOG_PACKET(packet, level) mip_log_packet(packet, level)
#define MIP_LOG_FIELD(field, level)   mip_log_packet(field, level)

#define MIP_LOG_PACKET_VERBOSE(packet, level) mip_log_packet_verbose(packet, level)


#else // !MICROSTRAIN_LOGGING_ENABLED


#define MIP_LOG_PACKET(packet, grouping, level)
#define MIP_LOG_FIELD(field, grouping, level)

#define MIP_LOG_PACKET_VERBOSE(packet, grouping, level)
#define MIP_LOG_FIELD_VERBOSE(field, grouping, level)


#endif // MICROSTRAIN_LOGGING_ENABLED


///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif
