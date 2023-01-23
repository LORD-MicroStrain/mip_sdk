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
///@addtogroup MipData_cpp  MIP Data [CPP]
///@{
///@defgroup shared_data_cpp  Shared Data [CPP]
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
///@defgroup cpp_shared_event_source  (0xFF,0xD0) Event Source [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(trigger_id);
    }
    
    uint8_t trigger_id = 0; ///< Trigger ID number. If 0, this message was emitted due to being scheduled in the 3DM Message Format Command (0x0C,0x0F).
    
};
void insert(Serializer& serializer, const EventSource& self);
void extract(Serializer& serializer, EventSource& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_ticks  (0xFF,0xD1) Ticks [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(ticks);
    }
    
    uint32_t ticks = 0; ///< Ticks since powerup.
    
};
void insert(Serializer& serializer, const Ticks& self);
void extract(Serializer& serializer, Ticks& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_delta_ticks  (0xFF,0xD2) Delta Ticks [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(ticks);
    }
    
    uint32_t ticks = 0; ///< Ticks since last output.
    
};
void insert(Serializer& serializer, const DeltaTicks& self);
void extract(Serializer& serializer, DeltaTicks& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_gps_timestamp  (0xFF,0xD3) Gps Timestamp [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(tow,week_number,valid_flags);
    }
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            TOW         = 0x0001,  ///<  Whole number seconds TOW has been set
            WEEK_NUMBER = 0x0002,  ///<  Week number has been set
            TIME_VALID  = 0x0003,  ///<  Both TOW and Week Number have been set
            ALL         = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool tow() const { return (value & TOW) > 0; }
        void tow(bool val) { if(val) value |= TOW; else value &= ~TOW; }
        bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        void weekNumber(bool val) { if(val) value |= WEEK_NUMBER; else value &= ~WEEK_NUMBER; }
        uint16_t timeValid() const { return (value & TIME_VALID) >> 0; }
        void timeValid(uint16_t val) { value = (value & ~TIME_VALID) | (val << 0); }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    double tow = 0; ///< GPS Time of Week [seconds]
    uint16_t week_number = 0; ///< GPS Week Number since 1980 [weeks]
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const GpsTimestamp& self);
void extract(Serializer& serializer, GpsTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_delta_time  (0xFF,0xD4) Delta Time [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(seconds);
    }
    
    double seconds = 0; ///< Seconds since last output.
    
};
void insert(Serializer& serializer, const DeltaTime& self);
void extract(Serializer& serializer, DeltaTime& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_reference_timestamp  (0xFF,0xD5) Reference Timestamp [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(nanoseconds);
    }
    
    uint64_t nanoseconds = 0; ///< Nanoseconds since initialization.
    
};
void insert(Serializer& serializer, const ReferenceTimestamp& self);
void extract(Serializer& serializer, ReferenceTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_reference_time_delta  (0xFF,0xD6) Reference Time Delta [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(dt_nanos);
    }
    
    uint64_t dt_nanos = 0; ///< Nanoseconds since the last occurrence of this field in a packet of the same descriptor set and event source.
    
};
void insert(Serializer& serializer, const ReferenceTimeDelta& self);
void extract(Serializer& serializer, ReferenceTimeDelta& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_external_timestamp  (0xFF,0xD7) External Timestamp [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(nanoseconds,valid_flags);
    }
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            NANOSECONDS = 0x0001,  ///<  
            ALL         = 0x0001,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool nanoseconds() const { return (value & NANOSECONDS) > 0; }
        void nanoseconds(bool val) { if(val) value |= NANOSECONDS; else value &= ~NANOSECONDS; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    uint64_t nanoseconds = 0;
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const ExternalTimestamp& self);
void extract(Serializer& serializer, ExternalTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_shared_external_time_delta  (0xFF,0xD8) External Time Delta [CPP]
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
    
    auto as_tuple() const
    {
        return std::make_tuple(dt_nanos,valid_flags);
    }
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE     = 0x0000,
            DT_NANOS = 0x0001,  ///<  
            ALL      = 0x0001,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool dtNanos() const { return (value & DT_NANOS) > 0; }
        void dtNanos(bool val) { if(val) value |= DT_NANOS; else value &= ~DT_NANOS; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    uint64_t dt_nanos = 0; ///< Nanoseconds since the last occurrence of this field in a packet of the same descriptor set and event source.
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

