#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
#endif // __cplusplus

struct mip_interface;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup SHARED_DATA  SHARED DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_shared_data_descriptors
{
    MIP_SHARED_DATA_DESC_SET            = 0xFF,
    
    MIP_DATA_DESC_SHARED_EVENT_SOURCE   = 0xD0,
    MIP_DATA_DESC_SHARED_TICKS          = 0xD1,
    MIP_DATA_DESC_SHARED_DELTA_TICKS    = 0xD2,
    MIP_DATA_DESC_SHARED_GPS_TIME       = 0xD3,
    MIP_DATA_DESC_SHARED_DELTA_TIME     = 0xD4,
    MIP_DATA_DESC_SHARED_REFERENCE_TIME = 0xD5,
    MIP_DATA_DESC_SHARED_REF_TIME_DELTA = 0xD6,
    MIP_DATA_DESC_SHARED_EXTERNAL_TIME  = 0xD7,
    MIP_DATA_DESC_SHARED_SYS_TIME_DELTA = 0xD8,
    
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIP_DATA_DESC_SHARED_START_MIP_DATA_DESC_SHARED_START 0xD0

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_event_source  None
/// Identifies which event trigger caused this packet to be emitted.
/// 
/// Generally this is used to determine whether a packet was emitted
/// due to scheduled streaming or due to an event.
///
///@{

struct mip_shared_event_source_command
{
    uint8_t                                           trigger_id;
};
size_t insert_mip_shared_event_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_event_source_command* self);
size_t extract_mip_shared_event_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_event_source_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_ticks  None
/// Time since powerup in multiples of the base rate.
/// 
/// The counter will wrap around to 0 after approximately 50 days.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct mip_shared_ticks_command
{
    uint32_t                                          ticks;
};
size_t insert_mip_shared_ticks_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_ticks_command* self);
size_t extract_mip_shared_ticks_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_ticks_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_delta_ticks  None
/// Ticks since the last output of this field.
/// 
/// This field can be used to track the amount of time passed between
/// event occurrences.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct mip_shared_delta_ticks_command
{
    uint32_t                                          ticks;
};
size_t insert_mip_shared_delta_ticks_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_delta_ticks_command* self);
size_t extract_mip_shared_delta_ticks_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_delta_ticks_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_gps_timestamp  None
/// Outputs the current GPS system time in time-of-week and week number format.
/// 
/// For events, this is the time of the event trigger.
/// In order to be valid, a PPS signal needs to be present, and both a valid GPS time-of-week and week number command (0x0C, 0x72) need to be received after PPS sync has been achieved.
///
///@{

enum mip_shared_gps_timestamp_command_valid_flags
{
    MIP_SHARED_GPS_TIMESTAMP_COMMAND_VALID_FLAGS_TOW         = 0x01,
    MIP_SHARED_GPS_TIMESTAMP_COMMAND_VALID_FLAGS_WEEK_NUMBER = 0x02,
    MIP_SHARED_GPS_TIMESTAMP_COMMAND_VALID_FLAGS_TIME_VALID  = 0x03,
};
size_t insert_mip_shared_gps_timestamp_command_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_shared_gps_timestamp_command_valid_flags self);
size_t extract_mip_shared_gps_timestamp_command_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_shared_gps_timestamp_command_valid_flags* self);

struct mip_shared_gps_timestamp_command
{
    double                                            tow;
    uint16_t                                          week_number;
    enum mip_shared_gps_timestamp_command_valid_flags valid_flags;
};
size_t insert_mip_shared_gps_timestamp_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_gps_timestamp_command* self);
size_t extract_mip_shared_gps_timestamp_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_gps_timestamp_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_delta_time  None
/// Time in the synchronized clock domain since the last output of this field within the same descriptor set and event instance.
/// 
/// This can be used to track the amount of time passed between
/// event occurrences. See the manual page on delta time quantities.
/// 
/// This field contains the same value as the delta external time field, 0xD8,
/// but is expressed in seconds. Transmission of either of these fields
/// restarts a shared counter, so only one should be streamed at a time to
/// avoid confusion. The counter is not shared across descriptors sets or
/// between event instances.
///
///@{

struct mip_shared_delta_time_command
{
    double                                            seconds;
};
size_t insert_mip_shared_delta_time_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_delta_time_command* self);
size_t extract_mip_shared_delta_time_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_delta_time_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_reference_timestamp  None
/// Internal reference timestamp.
/// 
/// This timestamp represents the time at which the corresponding
/// data was sampled, according to the internal reference clock.
/// 
/// This is a monotonic clock which never jumps. The value is always valid.
/// 
/// For events, this is the time of the event trigger.
///
///@{

struct mip_shared_reference_timestamp_command
{
    uint64_t                                          nanoseconds;
};
size_t insert_mip_shared_reference_timestamp_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_reference_timestamp_command* self);
size_t extract_mip_shared_reference_timestamp_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_reference_timestamp_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_reference_time_delta  None
/// Delta time since the last packet.
/// 
/// Difference between the time as reported by the shared reference time field, 0xD5,
/// and the previous output of this delta quantity within the same descriptor set and event instance.
/// 
/// The delta is based on the reference time which never jumps. The value
/// is always valid.
/// 
/// This can be used to track the amount of time passed between
/// event occurrences. See the manual page on delta time quantities.
///
///@{

struct mip_shared_reference_time_delta_command
{
    uint64_t                                          dt_nanos;
};
size_t insert_mip_shared_reference_time_delta_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_reference_time_delta_command* self);
size_t extract_mip_shared_reference_time_delta_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_reference_time_delta_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_external_timestamp  None
/// External timestamp in nanoseconds.
/// 
/// This timestamp represents the time at which the corresponding
/// data was sampled in the external clock domain.
/// Equivalent to the GPS Timestamp but in nanoseconds.
/// 
/// For events, this is the time of the event trigger.
/// 
/// To be valid, external clock sync must be achieved using the PPS input.
///
///@{

enum mip_shared_external_timestamp_command_valid_flags
{
    MIP_SHARED_EXTERNAL_TIMESTAMP_COMMAND_VALID_FLAGS_NANOSECONDS = 0x01,
};
size_t insert_mip_shared_external_timestamp_command_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_shared_external_timestamp_command_valid_flags self);
size_t extract_mip_shared_external_timestamp_command_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_shared_external_timestamp_command_valid_flags* self);

struct mip_shared_external_timestamp_command
{
    uint64_t                                          nanoseconds;
    enum mip_shared_external_timestamp_command_valid_flags valid_flags;
};
size_t insert_mip_shared_external_timestamp_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_external_timestamp_command* self);
size_t extract_mip_shared_external_timestamp_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_external_timestamp_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_shared_external_time_delta  None
/// Delta time since the last packet containing delta external (0xFF,0xD4) or delta gps time (0xFF,0xD8).
/// 
/// Difference between the time as reported by the shared external time field, 0xD7,
/// and the previous output of this delta quantity within the same descriptor set and event instance.
/// 
/// This can be used to track the amount of time passed between
/// event occurrences. See the manual page on delta time quantities.
/// 
/// This field contains the same value as the delta gps time field, 0xD4,
/// but is expressed in nanoseconds. Transmission of either of these fields
/// restarts a shared counter, so only one should be streamed at a time to
/// avoid confusion. The counter is not shared across descriptors sets or
/// between event instances.
///
///@{

enum mip_shared_external_time_delta_command_valid_flags
{
    MIP_SHARED_EXTERNAL_TIME_DELTA_COMMAND_VALID_FLAGS_DT_NANOS = 0x01,
};
size_t insert_mip_shared_external_time_delta_command_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_shared_external_time_delta_command_valid_flags self);
size_t extract_mip_shared_external_time_delta_command_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_shared_external_time_delta_command_valid_flags* self);

struct mip_shared_external_time_delta_command
{
    uint64_t                                          dt_nanos;
    enum mip_shared_external_time_delta_command_valid_flags valid_flags;
};
size_t insert_mip_shared_external_time_delta_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_external_time_delta_command* self);
size_t extract_mip_shared_external_time_delta_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_external_time_delta_command* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C


template<>
struct MipFieldInfo<C::mip_shared_event_source_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_EVENT_SOURCE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_event_source_command& self)
    {
        return C::insert_mip_shared_event_source_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_event_source_command& self)
    {
        return C::extract_mip_shared_event_source_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_ticks_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_TICKS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_ticks_command& self)
    {
        return C::insert_mip_shared_ticks_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_ticks_command& self)
    {
        return C::extract_mip_shared_ticks_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_delta_ticks_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_DELTA_TICKS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_delta_ticks_command& self)
    {
        return C::insert_mip_shared_delta_ticks_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_delta_ticks_command& self)
    {
        return C::extract_mip_shared_delta_ticks_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_gps_timestamp_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_GPS_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_gps_timestamp_command& self)
    {
        return C::insert_mip_shared_gps_timestamp_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_gps_timestamp_command& self)
    {
        return C::extract_mip_shared_gps_timestamp_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_delta_time_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_DELTA_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_delta_time_command& self)
    {
        return C::insert_mip_shared_delta_time_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_delta_time_command& self)
    {
        return C::extract_mip_shared_delta_time_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_reference_timestamp_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_REFERENCE_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_reference_timestamp_command& self)
    {
        return C::insert_mip_shared_reference_timestamp_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_reference_timestamp_command& self)
    {
        return C::extract_mip_shared_reference_timestamp_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_reference_time_delta_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_REF_TIME_DELTA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_reference_time_delta_command& self)
    {
        return C::insert_mip_shared_reference_time_delta_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_reference_time_delta_command& self)
    {
        return C::extract_mip_shared_reference_time_delta_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_external_timestamp_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_EXTERNAL_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_external_timestamp_command& self)
    {
        return C::insert_mip_shared_external_timestamp_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_external_timestamp_command& self)
    {
        return C::extract_mip_shared_external_timestamp_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_shared_external_time_delta_command>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_SYS_TIME_DELTA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_shared_external_time_delta_command& self)
    {
        return C::insert_mip_shared_external_time_delta_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_shared_external_time_delta_command& self)
    {
        return C::extract_mip_shared_external_time_delta_command(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
