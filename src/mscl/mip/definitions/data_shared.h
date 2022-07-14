#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

struct MipInterfaceState;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup SHARED_DATA  SHARED DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipSharedDataDescriptors
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

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIPMIPDATADESCSHAREDSTART_MIP_DATA_DESC_SHARED_START 0xD0

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_event_source  Shared Event Source
/// Identifies which event trigger caused this packet to be emitted.
/// 
/// Generally this is used to determine whether a packet was emitted
/// due to scheduled streaming or due to an event.
///
///@{

struct MipData_Shared_EventSource
{
    uint8_t                                           trigger_id;
};
size_t insert_MipData_Shared_EventSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_EventSource* self);
size_t extract_MipData_Shared_EventSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_EventSource* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_ticks  Shared Ticks
/// Time since powerup in multiples of the base rate.
/// 
/// The counter will wrap around to 0 after approximately 50 days.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct MipData_Shared_Ticks
{
    uint32_t                                          ticks;
};
size_t insert_MipData_Shared_Ticks(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_Ticks* self);
size_t extract_MipData_Shared_Ticks(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_Ticks* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_delta_ticks  Shared Delta Ticks
/// Ticks since the last output of this field.
/// 
/// This field can be used to track the amount of time passed between
/// event occurrences.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct MipData_Shared_DeltaTicks
{
    uint32_t                                          ticks;
};
size_t insert_MipData_Shared_DeltaTicks(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_DeltaTicks* self);
size_t extract_MipData_Shared_DeltaTicks(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_DeltaTicks* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_gps_timestamp  Shared Gps Timestamp
/// Outputs the current GPS system time in time-of-week and week number format.
/// 
/// For events, this is the time of the event trigger.
/// In order to be valid, a PPS signal needs to be present, and both a valid GPS time-of-week and week number command (0x0C, 0x72) need to be received after PPS sync has been achieved.
///
///@{

enum MipData_Shared_GpsTimestamp_Validflags
{
    MIPDATA_SHARED_GPSTIMESTAMP_VALIDFLAGS_TOW         = 0x01,
    MIPDATA_SHARED_GPSTIMESTAMP_VALIDFLAGS_WEEK_NUMBER = 0x02,
    MIPDATA_SHARED_GPSTIMESTAMP_VALIDFLAGS_TIME_VALID  = 0x03,
};
size_t insert_MipData_Shared_GpsTimestamp_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Shared_GpsTimestamp_Validflags self);
size_t extract_MipData_Shared_GpsTimestamp_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Shared_GpsTimestamp_Validflags* self);

struct MipData_Shared_GpsTimestamp
{
    double                                            tow;
    uint16_t                                          week_number;
    enum MipData_Shared_GpsTimestamp_Validflags       valid_flags;
};
size_t insert_MipData_Shared_GpsTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_GpsTimestamp* self);
size_t extract_MipData_Shared_GpsTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_GpsTimestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_delta_time  Shared Delta Time
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

struct MipData_Shared_DeltaTime
{
    double                                            seconds;
};
size_t insert_MipData_Shared_DeltaTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_DeltaTime* self);
size_t extract_MipData_Shared_DeltaTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_DeltaTime* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_reference_timestamp  Shared Reference Timestamp
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

struct MipData_Shared_ReferenceTimestamp
{
    uint64_t                                          nanoseconds;
};
size_t insert_MipData_Shared_ReferenceTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ReferenceTimestamp* self);
size_t extract_MipData_Shared_ReferenceTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ReferenceTimestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_reference_time_delta  Shared Reference Time Delta
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

struct MipData_Shared_ReferenceTimeDelta
{
    uint64_t                                          dt_nanos;
};
size_t insert_MipData_Shared_ReferenceTimeDelta(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ReferenceTimeDelta* self);
size_t extract_MipData_Shared_ReferenceTimeDelta(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ReferenceTimeDelta* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_external_timestamp  Shared External Timestamp
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

enum MipData_Shared_ExternalTimestamp_Validflags
{
    MIPDATA_SHARED_EXTERNALTIMESTAMP_VALIDFLAGS_NANOSECONDS = 0x01,
};
size_t insert_MipData_Shared_ExternalTimestamp_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Shared_ExternalTimestamp_Validflags self);
size_t extract_MipData_Shared_ExternalTimestamp_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Shared_ExternalTimestamp_Validflags* self);

struct MipData_Shared_ExternalTimestamp
{
    uint64_t                                          nanoseconds;
    enum MipData_Shared_ExternalTimestamp_Validflags  valid_flags;
};
size_t insert_MipData_Shared_ExternalTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ExternalTimestamp* self);
size_t extract_MipData_Shared_ExternalTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ExternalTimestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_shared_external_time_delta  Shared External Time Delta
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

enum MipData_Shared_ExternalTimeDelta_Validflags
{
    MIPDATA_SHARED_EXTERNALTIMEDELTA_VALIDFLAGS_DT_NANOS = 0x01,
};
size_t insert_MipData_Shared_ExternalTimeDelta_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Shared_ExternalTimeDelta_Validflags self);
size_t extract_MipData_Shared_ExternalTimeDelta_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Shared_ExternalTimeDelta_Validflags* self);

struct MipData_Shared_ExternalTimeDelta
{
    uint64_t                                          dt_nanos;
    enum MipData_Shared_ExternalTimeDelta_Validflags  valid_flags;
};
size_t insert_MipData_Shared_ExternalTimeDelta(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ExternalTimeDelta* self);
size_t extract_MipData_Shared_ExternalTimeDelta(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ExternalTimeDelta* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipData_Shared_EventSource>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_EVENT_SOURCE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_EventSource& self)
    {
        return insert_MipData_Shared_EventSource(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_EventSource& self)
    {
        return extract_MipData_Shared_EventSource(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_Ticks>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_TICKS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_Ticks& self)
    {
        return insert_MipData_Shared_Ticks(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_Ticks& self)
    {
        return extract_MipData_Shared_Ticks(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_DeltaTicks>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_DELTA_TICKS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_DeltaTicks& self)
    {
        return insert_MipData_Shared_DeltaTicks(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_DeltaTicks& self)
    {
        return extract_MipData_Shared_DeltaTicks(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_GpsTimestamp>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_GPS_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_GpsTimestamp& self)
    {
        return insert_MipData_Shared_GpsTimestamp(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_GpsTimestamp& self)
    {
        return extract_MipData_Shared_GpsTimestamp(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_DeltaTime>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_DELTA_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_DeltaTime& self)
    {
        return insert_MipData_Shared_DeltaTime(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_DeltaTime& self)
    {
        return extract_MipData_Shared_DeltaTime(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_ReferenceTimestamp>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_REFERENCE_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_ReferenceTimestamp& self)
    {
        return insert_MipData_Shared_ReferenceTimestamp(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_ReferenceTimestamp& self)
    {
        return extract_MipData_Shared_ReferenceTimestamp(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_ReferenceTimeDelta>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_REF_TIME_DELTA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_ReferenceTimeDelta& self)
    {
        return insert_MipData_Shared_ReferenceTimeDelta(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_ReferenceTimeDelta& self)
    {
        return extract_MipData_Shared_ReferenceTimeDelta(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_ExternalTimestamp>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_EXTERNAL_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_ExternalTimestamp& self)
    {
        return insert_MipData_Shared_ExternalTimestamp(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_ExternalTimestamp& self)
    {
        return extract_MipData_Shared_ExternalTimestamp(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Shared_ExternalTimeDelta>
{
    static const uint8_t descriptorSet = MIP_SHARED_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SHARED_SYS_TIME_DELTA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Shared_ExternalTimeDelta& self)
    {
        return insert_MipData_Shared_ExternalTimeDelta(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Shared_ExternalTimeDelta& self)
    {
        return extract_MipData_Shared_ExternalTimeDelta(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
