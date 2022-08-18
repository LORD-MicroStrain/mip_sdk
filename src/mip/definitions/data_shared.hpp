#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_shared {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup shared_data_cpp  SHAREDData
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET      = 0xFF,
    
    
    DATA_EVENT_SOURCE   = 0xD0,
    DATA_TICKS          = 0xD1,
    DATA_DELTA_TICKS    = 0xD2,
    DATA_GPS_TIME       = 0xD3,
    DATA_DELTA_TIME     = 0xD4,
    DATA_REFERENCE_TIME = 0xD5,
    DATA_REF_TIME_DELTA = 0xD6,
    DATA_EXTERNAL_TIME  = 0xD7,
    DATA_SYS_TIME_DELTA = 0xD8,
    DATA_DEBUG_TICKS    = 0xFF,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static const uint8_t MIP_DATA_DESC_SHARED_START = 0xD0;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_event_source  None
/// Identifies which event trigger caused this packet to be emitted.
/// 
/// Generally this is used to determine whether a packet was emitted
/// due to scheduled streaming or due to an event.
///
///@{

struct EventSource
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_EVENT_SOURCE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t trigger_id;
    
};
void insert(Serializer& serializer, const EventSource& self);
void extract(Serializer& serializer, EventSource& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ticks  None
/// Time since powerup in multiples of the base rate.
/// 
/// The counter will wrap around to 0 after approximately 50 days.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct Ticks
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_TICKS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint32_t ticks;
    
};
void insert(Serializer& serializer, const Ticks& self);
void extract(Serializer& serializer, Ticks& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_delta_ticks  None
/// Ticks since the last output of this field.
/// 
/// This field can be used to track the amount of time passed between
/// event occurrences.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct DeltaTicks
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_DELTA_TICKS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint32_t ticks;
    
};
void insert(Serializer& serializer, const DeltaTicks& self);
void extract(Serializer& serializer, DeltaTicks& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_timestamp  None
/// Outputs the current GPS system time in time-of-week and week number format.
/// 
/// For events, this is the time of the event trigger.
/// In order to be valid, a PPS signal needs to be present, and both a valid GPS time-of-week and week number command (0x0C, 0x72) need to be received after PPS sync has been achieved.
///
///@{

struct GpsTimestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_GPS_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            TOW         = 0x0001,
            WEEK_NUMBER = 0x0002,
            TIME_VALID  = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    double tow;
    uint16_t week_number;
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const GpsTimestamp& self);
void extract(Serializer& serializer, GpsTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_delta_time  None
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

struct DeltaTime
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_DELTA_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double seconds;
    
};
void insert(Serializer& serializer, const DeltaTime& self);
void extract(Serializer& serializer, DeltaTime& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_reference_timestamp  None
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

struct ReferenceTimestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_REFERENCE_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint64_t nanoseconds;
    
};
void insert(Serializer& serializer, const ReferenceTimestamp& self);
void extract(Serializer& serializer, ReferenceTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_reference_time_delta  None
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

struct ReferenceTimeDelta
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_REF_TIME_DELTA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint64_t dt_nanos;
    
};
void insert(Serializer& serializer, const ReferenceTimeDelta& self);
void extract(Serializer& serializer, ReferenceTimeDelta& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_external_timestamp  None
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

struct ExternalTimestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_EXTERNAL_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            NANOSECONDS = 0x0001,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    uint64_t nanoseconds;
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const ExternalTimestamp& self);
void extract(Serializer& serializer, ExternalTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_external_time_delta  None
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

struct ExternalTimeDelta
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_shared::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_shared::DATA_SYS_TIME_DELTA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE     = 0x0000,
            DT_NANOS = 0x0001,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    uint64_t dt_nanos;
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const ExternalTimeDelta& self);
void extract(Serializer& serializer, ExternalTimeDelta& self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_shared
} // namespace mip

