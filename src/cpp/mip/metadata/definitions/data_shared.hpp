#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/data_shared.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<data_shared::EventSource>
{
    using type = data_shared::EventSource;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "trigger_id",
            /* .docs          = */ "Trigger ID number. If 0, this message was emitted due to being\nscheduled in the 3DM Message Format Command (0x0C,0x0F).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::trigger_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::EventSource",
        /* .title       = */ "event_source",
        /* .docs        = */ "Identifies which event trigger caused this packet to be emitted.\n\nGenerally this is used to determine whether a packet was emitted\ndue to scheduled streaming or due to an event.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::Ticks>
{
    using type = data_shared::Ticks;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "ticks",
            /* .docs          = */ "Ticks since powerup.",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::ticks>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::Ticks",
        /* .title       = */ "ticks",
        /* .docs        = */ "Time since powerup in multiples of the base rate.\n\nThe counter will wrap around to 0 after approximately 50 days.\nOne tick is equivalent to one base period (reciprocal of the base rate).",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::DeltaTicks>
{
    using type = data_shared::DeltaTicks;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "ticks",
            /* .docs          = */ "Ticks since last output.",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::ticks>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::DeltaTicks",
        /* .title       = */ "delta_ticks",
        /* .docs        = */ "Ticks since the last output of this field.\n\nThis field can be used to track the amount of time passed between\nevent occurrences.\nOne tick is equivalent to one base period (reciprocal of the base rate).",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::GpsTimestamp::ValidFlags>
{
    using type = data_shared::GpsTimestamp::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "Whole number seconds TOW has been set" },
        { uint32_t(2), "week_number", "Week number has been set" },
        { uint32_t(3), "time_valid", "Both TOW and Week Number have been set" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_shared::GpsTimestamp>
{
    using type = data_shared::GpsTimestamp;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "tow",
            /* .docs          = */ "GPS Time of Week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::tow>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week Number since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_shared::GpsTimestamp::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_shared::GpsTimestamp::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::GpsTimestamp",
        /* .title       = */ "gps_timestamp",
        /* .docs        = */ "Outputs the current GPS system time in time-of-week and week number format.\n\nFor events, this is the time of the event trigger.\nIn order to be valid, a PPS signal needs to be present, and both a valid GPS time-of-week and week number command (0x0C, 0x72) need to be received after PPS sync has been achieved.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::DeltaTime>
{
    using type = data_shared::DeltaTime;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "seconds",
            /* .docs          = */ "Seconds since last output.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::seconds>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::DeltaTime",
        /* .title       = */ "delta_time",
        /* .docs        = */ "Time in the synchronized clock domain since the last output of this field within the same descriptor set and event instance.\n\nThis can be used to track the amount of time passed between\nevent occurrences. See the manual page on delta time quantities.\n\nThis field contains the same value as the delta external time field, 0xD8,\nbut is expressed in seconds. Transmission of either of these fields\nrestarts a shared counter, so only one should be streamed at a time to\navoid confusion. The counter is not shared across descriptors sets or\nbetween event instances.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::ReferenceTimestamp>
{
    using type = data_shared::ReferenceTimestamp;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "nanoseconds",
            /* .docs          = */ "Nanoseconds since initialization.",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint64_t, &type::nanoseconds>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::ReferenceTimestamp",
        /* .title       = */ "reference_timestamp",
        /* .docs        = */ "Internal reference timestamp.\n\nThis timestamp represents the time at which the corresponding\ndata was sampled, according to the internal reference clock.\n\nThis is a monotonic clock which never jumps. The value is always valid.\n\nFor events, this is the time of the event trigger.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::ReferenceTimeDelta>
{
    using type = data_shared::ReferenceTimeDelta;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "dt_nanos",
            /* .docs          = */ "Nanoseconds since the last occurrence of this field in a packet of the same descriptor set and event source.",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint64_t, &type::dt_nanos>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::ReferenceTimeDelta",
        /* .title       = */ "reference_time_delta",
        /* .docs        = */ "Delta time since the last packet.\n\nDifference between the time as reported by the shared reference time field, 0xD5,\nand the previous output of this delta quantity within the same descriptor set and event instance.\n\nThe delta is based on the reference time which never jumps. The value\nis always valid.\n\nThis can be used to track the amount of time passed between\nevent occurrences. See the manual page on delta time quantities.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::ExternalTimestamp::ValidFlags>
{
    using type = data_shared::ExternalTimestamp::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "nanoseconds", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_shared::ExternalTimestamp>
{
    using type = data_shared::ExternalTimestamp;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "nanoseconds",
            /* .docs          = */ "",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint64_t, &type::nanoseconds>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_shared::ExternalTimestamp::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_shared::ExternalTimestamp::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::ExternalTimestamp",
        /* .title       = */ "external_timestamp",
        /* .docs        = */ "External timestamp in nanoseconds.\n\nThis timestamp represents the time at which the corresponding\ndata was sampled in the external clock domain.\nEquivalent to the GPS Timestamp but in nanoseconds.\n\nFor events, this is the time of the event trigger.\n\nTo be valid, external clock sync must be achieved using the PPS input.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_shared::ExternalTimeDelta::ValidFlags>
{
    using type = data_shared::ExternalTimeDelta::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "dt_nanos", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_shared::ExternalTimeDelta>
{
    using type = data_shared::ExternalTimeDelta;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "dt_nanos",
            /* .docs          = */ "Nanoseconds since the last occurrence of this field in a packet of the same descriptor set and event source.",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint64_t, &type::dt_nanos>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_shared::ExternalTimeDelta::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_shared::ExternalTimeDelta::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_shared::ExternalTimeDelta",
        /* .title       = */ "external_time_delta",
        /* .docs        = */ "Delta time since the last packet containing delta external (0xFF,0xD4) or delta gps time (0xFF,0xD8).\n\nDifference between the time as reported by the shared external time field, 0xD7,\nand the previous output of this delta quantity within the same descriptor set and event instance.\n\nThis can be used to track the amount of time passed between\nevent occurrences. See the manual page on delta time quantities.\n\nThis field contains the same value as the delta gps time field, 0xD4,\nbut is expressed in nanoseconds. Transmission of either of these fields\nrestarts a shared counter, so only one should be streamed at a time to\navoid confusion. The counter is not shared across descriptors sets or\nbetween event instances.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* DATA_SHARED_FIELDS[] = {
    &MetadataFor<data_shared::EventSource>::value,
    &MetadataFor<data_shared::Ticks>::value,
    &MetadataFor<data_shared::DeltaTicks>::value,
    &MetadataFor<data_shared::GpsTimestamp>::value,
    &MetadataFor<data_shared::DeltaTime>::value,
    &MetadataFor<data_shared::ReferenceTimestamp>::value,
    &MetadataFor<data_shared::ReferenceTimeDelta>::value,
    &MetadataFor<data_shared::ExternalTimestamp>::value,
    &MetadataFor<data_shared::ExternalTimeDelta>::value,
};

static constexpr DescriptorSetInfo DATA_SHARED = {
    /*.descriptor =*/ mip::data_shared::DESCRIPTOR_SET,
    /*.name       =*/ "Shared Data",
    /*.fields     =*/ DATA_SHARED_FIELDS,
};

} // namespace mip::metadata

