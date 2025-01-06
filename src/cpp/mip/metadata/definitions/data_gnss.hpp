#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/data_gnss.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<data_gnss::PosLlh::ValidFlags>
{
    using type = data_gnss::PosLlh::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "lat_lon", "" },
        { uint32_t(2), "ellipsoid_height", "" },
        { uint32_t(4), "msl_height", "" },
        { uint32_t(8), "horizontal_accuracy", "" },
        { uint32_t(16), "vertical_accuracy", "" },
        { uint32_t(31), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::PosLlh>
{
    using type = data_gnss::PosLlh;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "latitude",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::latitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "longitude",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::longitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ellipsoid_height",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ellipsoid_height>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "msl_height",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::msl_height>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "horizontal_accuracy",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::horizontal_accuracy>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "vertical_accuracy",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::vertical_accuracy>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::PosLlh::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::PosLlh::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::PosLlh",
        /* .title       = */ "GNSS LLH Position",
        /* .docs        = */ "GNSS reported position in the WGS84 geodetic frame",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::PosEcef::ValidFlags>
{
    using type = data_gnss::PosEcef::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "position", "" },
        { uint32_t(2), "position_accuracy", "" },
        { uint32_t(3), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::PosEcef>
{
    using type = data_gnss::PosEcef;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "x",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::x>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "x_accuracy",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::x_accuracy>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::PosEcef::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::PosEcef::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::PosEcef",
        /* .title       = */ "GNSS ECEF Position",
        /* .docs        = */ "GNSS reported position in the Earth-centered, Earth-Fixed (ECEF) frame",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::VelNed::ValidFlags>
{
    using type = data_gnss::VelNed::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "velocity", "" },
        { uint32_t(2), "speed_3d", "" },
        { uint32_t(4), "ground_speed", "" },
        { uint32_t(8), "heading", "" },
        { uint32_t(16), "speed_accuracy", "" },
        { uint32_t(32), "heading_accuracy", "" },
        { uint32_t(63), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::VelNed>
{
    using type = data_gnss::VelNed;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "v",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::v>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "speed",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::speed>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ground_speed",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::ground_speed>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "speed_accuracy",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::speed_accuracy>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading_accuracy",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading_accuracy>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::VelNed::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::VelNed::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::VelNed",
        /* .title       = */ "NED Velocity",
        /* .docs        = */ "GNSS reported velocity in the NED frame",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::VelEcef::ValidFlags>
{
    using type = data_gnss::VelEcef::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "velocity", "" },
        { uint32_t(2), "velocity_accuracy", "" },
        { uint32_t(3), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::VelEcef>
{
    using type = data_gnss::VelEcef;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "v",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::v>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "v_accuracy",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::v_accuracy>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::VelEcef::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::VelEcef::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::VelEcef",
        /* .title       = */ "GNSS ECEF Velocity",
        /* .docs        = */ "GNSS reported velocity in the Earth-centered, Earth-Fixed (ECEF) frame",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::Dop::ValidFlags>
{
    using type = data_gnss::Dop::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "gdop", "" },
        { uint32_t(2), "pdop", "" },
        { uint32_t(4), "hdop", "" },
        { uint32_t(8), "vdop", "" },
        { uint32_t(16), "tdop", "" },
        { uint32_t(32), "ndop", "" },
        { uint32_t(64), "edop", "" },
        { uint32_t(127), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::Dop>
{
    using type = data_gnss::Dop;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gdop",
            /* .docs          = */ "Geometric DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::gdop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pdop",
            /* .docs          = */ "Position DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pdop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "hdop",
            /* .docs          = */ "Horizontal DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::hdop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "vdop",
            /* .docs          = */ "Vertical DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::vdop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tdop",
            /* .docs          = */ "Time DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::tdop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ndop",
            /* .docs          = */ "Northing DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::ndop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "edop",
            /* .docs          = */ "Easting DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::edop>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::Dop::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::Dop::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::Dop",
        /* .title       = */ "dop",
        /* .docs        = */ "GNSS reported dilution of precision information.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::UtcTime::ValidFlags>
{
    using type = data_gnss::UtcTime::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "gnss_date_time", "" },
        { uint32_t(2), "leap_seconds_known", "" },
        { uint32_t(3), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::UtcTime>
{
    using type = data_gnss::UtcTime;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "year",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::year>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "month",
            /* .docs          = */ "Month (1-12)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::month>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "day",
            /* .docs          = */ "Day (1-31)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::day>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "hour",
            /* .docs          = */ "Hour (0-23)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::hour>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "min",
            /* .docs          = */ "Minute (0-59)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::min>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sec",
            /* .docs          = */ "Second (0-59)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sec>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "msec",
            /* .docs          = */ "Millisecond(0-999)",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::msec>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::UtcTime::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::UtcTime::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::UtcTime",
        /* .title       = */ "utc_time",
        /* .docs        = */ "GNSS reported Coordinated Universal Time",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsTime::ValidFlags>
{
    using type = data_gnss::GpsTime::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(3), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GpsTime>
{
    using type = data_gnss::GpsTime;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "tow",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::tow>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GpsTime::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GpsTime::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsTime",
        /* .title       = */ "gps_time",
        /* .docs        = */ "GNSS reported GPS Time",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::ClockInfo::ValidFlags>
{
    using type = data_gnss::ClockInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "bias", "" },
        { uint32_t(2), "drift", "" },
        { uint32_t(4), "accuracy_estimate", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::ClockInfo>
{
    using type = data_gnss::ClockInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::bias>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "drift",
            /* .docs          = */ "[seconds/second]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::drift>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "accuracy_estimate",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::accuracy_estimate>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::ClockInfo::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::ClockInfo::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::ClockInfo",
        /* .title       = */ "clock_info",
        /* .docs        = */ "GNSS reported receiver clock parameters",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::FixInfo::FixType>
{
    using type = data_gnss::FixInfo::FixType;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "FIX_3D", "" },
        { uint32_t(1), "FIX_2D", "" },
        { uint32_t(2), "FIX_TIME_ONLY", "" },
        { uint32_t(3), "FIX_NONE", "" },
        { uint32_t(4), "FIX_INVALID", "" },
        { uint32_t(5), "FIX_RTK_FLOAT", "" },
        { uint32_t(6), "FIX_RTK_FIXED", "" },
        { uint32_t(7), "FIX_DIFFERENTIAL", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FixType",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::FixInfo::FixFlags>
{
    using type = data_gnss::FixInfo::FixFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "sbas_used", "" },
        { uint32_t(2), "dgnss_used", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "FixFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::FixInfo::ValidFlags>
{
    using type = data_gnss::FixInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "fix_type", "" },
        { uint32_t(2), "num_sv", "" },
        { uint32_t(4), "fix_flags", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::FixInfo>
{
    using type = data_gnss::FixInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "fix_type",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::FixInfo::FixType>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::FixInfo::FixType, &type::fix_type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "num_sv",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::num_sv>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "fix_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::FixInfo::FixFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::FixInfo::FixFlags, &type::fix_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::FixInfo::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::FixInfo::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::FixInfo",
        /* .title       = */ "fix_info",
        /* .docs        = */ "GNSS reported position fix type",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::SvInfo::SVFlags>
{
    using type = data_gnss::SvInfo::SVFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "used_for_navigation", "" },
        { uint32_t(2), "healthy", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "SVFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SvInfo::ValidFlags>
{
    using type = data_gnss::SvInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "channel", "" },
        { uint32_t(2), "sv_id", "" },
        { uint32_t(4), "carrier_noise_ratio", "" },
        { uint32_t(8), "azimuth", "" },
        { uint32_t(16), "elevation", "" },
        { uint32_t(32), "sv_flags", "" },
        { uint32_t(63), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SvInfo>
{
    using type = data_gnss::SvInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "channel",
            /* .docs          = */ "Receiver channel number",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::channel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sv_id",
            /* .docs          = */ "GNSS Satellite ID",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sv_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "carrier_noise_ratio",
            /* .docs          = */ "[dBHz]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::carrier_noise_ratio>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "azimuth",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::S16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, int16_t, &type::azimuth>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "elevation",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::S16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, int16_t, &type::elevation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sv_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::SvInfo::SVFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SvInfo::SVFlags, &type::sv_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::SvInfo::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SvInfo::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SvInfo",
        /* .title       = */ "sv_info",
        /* .docs        = */ "GNSS reported space vehicle information\n\nWhen enabled, these fields will arrive in separate MIP packets",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::HwStatus::ReceiverState>
{
    using type = data_gnss::HwStatus::ReceiverState;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "OFF", "" },
        { uint32_t(1), "ON", "" },
        { uint32_t(2), "UNKNOWN", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "ReceiverState",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::HwStatus::AntennaState>
{
    using type = data_gnss::HwStatus::AntennaState;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "INIT", "" },
        { uint32_t(2), "SHORT", "" },
        { uint32_t(3), "OPEN", "" },
        { uint32_t(4), "GOOD", "" },
        { uint32_t(5), "UNKNOWN", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "AntennaState",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::HwStatus::AntennaPower>
{
    using type = data_gnss::HwStatus::AntennaPower;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "OFF", "" },
        { uint32_t(1), "ON", "" },
        { uint32_t(2), "UNKNOWN", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "AntennaPower",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::HwStatus::ValidFlags>
{
    using type = data_gnss::HwStatus::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "sensor_state", "" },
        { uint32_t(2), "antenna_state", "" },
        { uint32_t(4), "antenna_power", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::HwStatus>
{
    using type = data_gnss::HwStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_state",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::HwStatus::ReceiverState>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::HwStatus::ReceiverState, &type::receiver_state>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "antenna_state",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::HwStatus::AntennaState>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::HwStatus::AntennaState, &type::antenna_state>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "antenna_power",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::HwStatus::AntennaPower>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::HwStatus::AntennaPower, &type::antenna_power>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::HwStatus::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::HwStatus::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::HwStatus",
        /* .title       = */ "GNSS Hardware Status",
        /* .docs        = */ "GNSS reported hardware status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::DgpsInfo::ValidFlags>
{
    using type = data_gnss::DgpsInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "age", "" },
        { uint32_t(2), "base_station_id", "" },
        { uint32_t(4), "base_station_status", "" },
        { uint32_t(8), "num_channels", "" },
        { uint32_t(15), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::DgpsInfo>
{
    using type = data_gnss::DgpsInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "sv_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sv_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "age",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::age>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_correction",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::range_correction>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_rate_correction",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::range_rate_correction>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::DgpsInfo::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::DgpsInfo::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::DgpsInfo",
        /* .title       = */ "dgps_info",
        /* .docs        = */ "GNSS reported DGNSS status\n\n<pre>Possible Base Station Status Values:</pre>\n<pre>  0 - UDRE Scale Factor = 1.0</pre>\n<pre>  1 - UDRE Scale Factor = 0.75</pre>\n<pre>  2 - UDRE Scale Factor = 0.5</pre>\n<pre>  3 - UDRE Scale Factor = 0.3</pre>\n<pre>  4 - UDRE Scale Factor = 0.2</pre>\n<pre>  5 - UDRE Scale Factor = 0.1</pre>\n<pre>  6 - Reference Station Transmission Not Monitored</pre>\n<pre>  7 - Reference Station Not Working</pre>\n\n(UDRE = User Differential Range Error)",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::DgpsChannel::ValidFlags>
{
    using type = data_gnss::DgpsChannel::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "id", "" },
        { uint32_t(2), "age", "" },
        { uint32_t(4), "range_correction", "" },
        { uint32_t(8), "range_rate_correction", "" },
        { uint32_t(15), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::DgpsChannel>
{
    using type = data_gnss::DgpsChannel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "sv_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sv_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "age",
            /* .docs          = */ "[s]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::age>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_correction",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::range_correction>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_rate_correction",
            /* .docs          = */ "[m/s]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::range_rate_correction>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::DgpsChannel::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::DgpsChannel::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::DgpsChannel",
        /* .title       = */ "dgps_channel",
        /* .docs        = */ "GNSS reported DGPS Channel Status status\n\nWhen enabled, a separate field for each active space vehicle will be sent in the packet.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::ClockInfo2::ValidFlags>
{
    using type = data_gnss::ClockInfo2::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "bias", "" },
        { uint32_t(2), "drift", "" },
        { uint32_t(4), "bias_accuracy", "" },
        { uint32_t(8), "drift_accuracy", "" },
        { uint32_t(15), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::ClockInfo2>
{
    using type = data_gnss::ClockInfo2;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::bias>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "drift",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::drift>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "bias_accuracy_estimate",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::bias_accuracy_estimate>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "drift_accuracy_estimate",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::drift_accuracy_estimate>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::ClockInfo2::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::ClockInfo2::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::ClockInfo2",
        /* .title       = */ "clock_info_2",
        /* .docs        = */ "GNSS reported receiver clock parameters\n\nThis supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsLeapSeconds::ValidFlags>
{
    using type = data_gnss::GpsLeapSeconds::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(2), "leap_seconds", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GpsLeapSeconds>
{
    using type = data_gnss::GpsLeapSeconds;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "leap_seconds",
            /* .docs          = */ "[s]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::leap_seconds>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GpsLeapSeconds::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GpsLeapSeconds::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsLeapSeconds",
        /* .title       = */ "gps_leap_seconds",
        /* .docs        = */ "GNSS reported leap seconds (difference between GPS and UTC Time)",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::SbasSystem>
{
    using type = data_gnss::SbasSystem;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "UNKNOWN", "" },
        { uint32_t(1), "WAAS", "" },
        { uint32_t(2), "EGNOS", "" },
        { uint32_t(3), "MSAS", "" },
        { uint32_t(4), "GAGAN", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "SbasSystem",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SbasInfo::SbasStatus>
{
    using type = data_gnss::SbasInfo::SbasStatus;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "range_available", "" },
        { uint32_t(2), "corrections_available", "" },
        { uint32_t(4), "integrity_available", "" },
        { uint32_t(8), "test_mode", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "SbasStatus",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SbasInfo::ValidFlags>
{
    using type = data_gnss::SbasInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "sbas_system", "" },
        { uint32_t(8), "sbas_id", "" },
        { uint32_t(16), "count", "" },
        { uint32_t(32), "sbas_status", "" },
        { uint32_t(63), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SbasInfo>
{
    using type = data_gnss::SbasInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sbas_system",
            /* .docs          = */ "SBAS system id",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::SbasSystem>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SbasSystem, &type::sbas_system>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sbas_id",
            /* .docs          = */ "SBAS satellite id.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sbas_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Number of SBAS corrections",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sbas_status",
            /* .docs          = */ "Status of the SBAS service",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::SbasInfo::SbasStatus>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SbasInfo::SbasStatus, &type::sbas_status>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::SbasInfo::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SbasInfo::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SbasInfo",
        /* .title       = */ "sbas_info",
        /* .docs        = */ "GNSS SBAS status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GnssConstellationId>
{
    using type = data_gnss::GnssConstellationId;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "UNKNOWN", "" },
        { uint32_t(1), "GPS", "" },
        { uint32_t(2), "GLONASS", "" },
        { uint32_t(3), "GALILEO", "" },
        { uint32_t(4), "BEIDOU", "" },
        { uint32_t(5), "SBAS", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "GnssConstellationId",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SbasCorrection::ValidFlags>
{
    using type = data_gnss::SbasCorrection::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "udrei", "" },
        { uint32_t(2), "pseudorange_correction", "" },
        { uint32_t(4), "iono_correction", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SbasCorrection>
{
    using type = data_gnss::SbasCorrection;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week the message was received [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gnss_id",
            /* .docs          = */ "GNSS constellation id",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssConstellationId>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GnssConstellationId, &type::gnss_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sv_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sv_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "udrei",
            /* .docs          = */ "[See above 0-13 usable, 14 not monitored, 15 - do not use]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::udrei>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pseudorange_correction",
            /* .docs          = */ "Pseudo-range correction [meters].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pseudorange_correction>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iono_correction",
            /* .docs          = */ "Ionospheric correction [meters].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::iono_correction>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::SbasCorrection::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SbasCorrection::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SbasCorrection",
        /* .title       = */ "sbas_correction",
        /* .docs        = */ "GNSS calculated SBAS Correction\n\nUDREI - the variance of a normal distribution associated with the user differential range errors for a\nsatellite after application of fast and long-term corrections, excluding atmospheric effects\n\n<pre>UDREI  Variance</pre>\n<pre>-----------------------</pre>\n<pre>0      0.0520 m^2</pre>\n<pre>1      0.0924 m^2</pre>\n<pre>2      0.1444 m^2</pre>\n<pre>3      0.2830 m^2</pre>\n<pre>4      0.4678 m^2</pre>\n<pre>5      0.8315 m^2</pre>\n<pre>6      1.2992 m^2</pre>\n<pre>7      1.8709 m^2</pre>\n<pre>8      2.5465 m^2</pre>\n<pre>9      3.3260 m^2</pre>\n<pre>10     5.1968 m^2</pre>\n<pre>11     20.7870 m^2</pre>\n<pre>12     230.9661 m^2</pre>\n<pre>13     2078.695 m^2</pre>\n<pre>14     'Not Monitored'</pre>\n<pre>15     'Do Not Use'</pre>",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::RfErrorDetection::RFBand>
{
    using type = data_gnss::RfErrorDetection::RFBand;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "UNKNOWN", "" },
        { uint32_t(1), "L1", "" },
        { uint32_t(2), "L2", "" },
        { uint32_t(5), "L5", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "RFBand",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::RfErrorDetection::JammingState>
{
    using type = data_gnss::RfErrorDetection::JammingState;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "UNKNOWN", "" },
        { uint32_t(1), "NONE", "" },
        { uint32_t(2), "PARTIAL", "" },
        { uint32_t(3), "SIGNIFICANT", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "JammingState",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::RfErrorDetection::SpoofingState>
{
    using type = data_gnss::RfErrorDetection::SpoofingState;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "UNKNOWN", "" },
        { uint32_t(1), "NONE", "" },
        { uint32_t(2), "PARTIAL", "" },
        { uint32_t(3), "SIGNIFICANT", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "SpoofingState",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::RfErrorDetection::ValidFlags>
{
    using type = data_gnss::RfErrorDetection::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "rf_band", "" },
        { uint32_t(2), "jamming_state", "" },
        { uint32_t(4), "spoofing_state", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::RfErrorDetection>
{
    using type = data_gnss::RfErrorDetection;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "rf_band",
            /* .docs          = */ "RF Band of the reported information",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::RfErrorDetection::RFBand>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::RfErrorDetection::RFBand, &type::rf_band>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "jamming_state",
            /* .docs          = */ "GNSS Jamming State (as reported by the GNSS module)",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::RfErrorDetection::JammingState>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::RfErrorDetection::JammingState, &type::jamming_state>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "spoofing_state",
            /* .docs          = */ "GNSS Spoofing State (as reported by the GNSS module)",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::RfErrorDetection::SpoofingState>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::RfErrorDetection::SpoofingState, &type::spoofing_state>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "Reserved for future use",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::RfErrorDetection::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::RfErrorDetection::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::RfErrorDetection",
        /* .title       = */ "rf_error_detection",
        /* .docs        = */ "GNSS Error Detection subsystem status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::BaseStationInfo::IndicatorFlags>
{
    using type = data_gnss::BaseStationInfo::IndicatorFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "gps", "" },
        { uint32_t(2), "glonass", "" },
        { uint32_t(4), "galileo", "" },
        { uint32_t(8), "beidou", "" },
        { uint32_t(16), "ref_station", "" },
        { uint32_t(32), "single_receiver", "" },
        { uint32_t(64), "quarter_cycle_bit1", "" },
        { uint32_t(128), "quarter_cycle_bit2", "" },
        { uint32_t(192), "quarter_cycle_bits", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "IndicatorFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::BaseStationInfo::ValidFlags>
{
    using type = data_gnss::BaseStationInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "ecef_position", "" },
        { uint32_t(8), "height", "" },
        { uint32_t(16), "station_id", "" },
        { uint32_t(32), "indicators", "" },
        { uint32_t(63), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::BaseStationInfo>
{
    using type = data_gnss::BaseStationInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week the message was received [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ecef_pos",
            /* .docs          = */ "Earth-centered, Earth-fixed [m]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::ecef_pos>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "height",
            /* .docs          = */ "Antenna Height above the marker used in the survey [m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::height>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "station_id",
            /* .docs          = */ "Range: 0-4095",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::station_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "indicators",
            /* .docs          = */ "Bitfield",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::BaseStationInfo::IndicatorFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::BaseStationInfo::IndicatorFlags, &type::indicators>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::BaseStationInfo::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::BaseStationInfo::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::BaseStationInfo",
        /* .title       = */ "base_station_info",
        /* .docs        = */ "RTCM reported base station information (sourced from RTCM Message 1005 or 1006)\n\nValid Flag Mapping:",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::RtkCorrectionsStatus::ValidFlags>
{
    using type = data_gnss::RtkCorrectionsStatus::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "epoch_status", "" },
        { uint32_t(8), "dongle_status", "" },
        { uint32_t(16), "gps_latency", "" },
        { uint32_t(32), "glonass_latency", "" },
        { uint32_t(64), "galileo_latency", "" },
        { uint32_t(128), "beidou_latency", "" },
        { uint32_t(255), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::RtkCorrectionsStatus::EpochStatus>
{
    using type = data_gnss::RtkCorrectionsStatus::EpochStatus;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "antenna_location_received", "" },
        { uint32_t(2), "antenna_description_received", "" },
        { uint32_t(4), "gps_received", "" },
        { uint32_t(8), "glonass_received", "" },
        { uint32_t(16), "galileo_received", "" },
        { uint32_t(32), "beidou_received", "" },
        { uint32_t(64), "using_gps_msm_messages", "Using MSM messages for GPS corrections instead of RTCM messages 1001-1004" },
        { uint32_t(128), "using_glonass_msm_messages", "Using MSM messages for GLONASS corrections instead of RTCM messages 1009-1012" },
        { uint32_t(256), "dongle_status_read_failed", "A read of the dongle status was attempted, but failed" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "EpochStatus",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::RtkCorrectionsStatus>
{
    using type = data_gnss::RtkCorrectionsStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "epoch_status",
            /* .docs          = */ "Status of the corrections received during this epoch",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::RtkCorrectionsStatus::EpochStatus>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::RtkCorrectionsStatus::EpochStatus, &type::epoch_status>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "dongle_status",
            /* .docs          = */ "RTK Dongle Status Flags (valid only when using RTK dongle, see Get RTK Device Status Flags (0x0F,0x01) for details)",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::dongle_status>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gps_correction_latency",
            /* .docs          = */ "Latency of last GPS correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::gps_correction_latency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "glonass_correction_latency",
            /* .docs          = */ "Latency of last GLONASS correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::glonass_correction_latency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "galileo_correction_latency",
            /* .docs          = */ "Latency of last Galileo correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::galileo_correction_latency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beidou_correction_latency",
            /* .docs          = */ "Latency of last Beidou correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::beidou_correction_latency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "Reserved for future use",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::RtkCorrectionsStatus::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::RtkCorrectionsStatus::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::RtkCorrectionsStatus",
        /* .title       = */ "rtk_corrections_status",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::SatelliteStatus::ValidFlags>
{
    using type = data_gnss::SatelliteStatus::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "gnss_id", "" },
        { uint32_t(8), "satellite_id", "" },
        { uint32_t(16), "elevation", "" },
        { uint32_t(32), "azimuth", "" },
        { uint32_t(64), "health", "" },
        { uint32_t(127), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::SatelliteStatus>
{
    using type = data_gnss::SatelliteStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gnss_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssConstellationId>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GnssConstellationId, &type::gnss_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::satellite_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "elevation",
            /* .docs          = */ "Elevation of the satellite relative to the rover [degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::elevation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "azimuth",
            /* .docs          = */ "Azimuth of the satellite relative to the rover [degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::azimuth>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "True if the satellite is healthy.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::health>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::SatelliteStatus::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::SatelliteStatus::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SatelliteStatus",
        /* .title       = */ "satellite_status",
        /* .docs        = */ "Status information for a GNSS satellite.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GnssSignalId>
{
    using type = data_gnss::GnssSignalId;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "UNKNOWN", "" },
        { uint32_t(1), "GPS_L1CA", "" },
        { uint32_t(2), "GPS_L1P", "" },
        { uint32_t(3), "GPS_L1Z", "" },
        { uint32_t(4), "GPS_L2CA", "" },
        { uint32_t(5), "GPS_L2P", "" },
        { uint32_t(6), "GPS_L2Z", "" },
        { uint32_t(7), "GPS_L2CL", "" },
        { uint32_t(8), "GPS_L2CM", "" },
        { uint32_t(9), "GPS_L2CML", "" },
        { uint32_t(10), "GPS_L5I", "" },
        { uint32_t(11), "GPS_L5Q", "" },
        { uint32_t(12), "GPS_L5IQ", "" },
        { uint32_t(13), "GPS_L1CD", "" },
        { uint32_t(14), "GPS_L1CP", "" },
        { uint32_t(15), "GPS_L1CDP", "" },
        { uint32_t(32), "GLONASS_G1CA", "" },
        { uint32_t(33), "GLONASS_G1P", "" },
        { uint32_t(34), "GLONASS_G2C", "" },
        { uint32_t(35), "GLONASS_G2P", "" },
        { uint32_t(64), "GALILEO_E1C", "" },
        { uint32_t(65), "GALILEO_E1A", "" },
        { uint32_t(66), "GALILEO_E1B", "" },
        { uint32_t(67), "GALILEO_E1BC", "" },
        { uint32_t(68), "GALILEO_E1ABC", "" },
        { uint32_t(69), "GALILEO_E6C", "" },
        { uint32_t(70), "GALILEO_E6A", "" },
        { uint32_t(71), "GALILEO_E6B", "" },
        { uint32_t(72), "GALILEO_E6BC", "" },
        { uint32_t(73), "GALILEO_E6ABC", "" },
        { uint32_t(74), "GALILEO_E5BI", "" },
        { uint32_t(75), "GALILEO_E5BQ", "" },
        { uint32_t(76), "GALILEO_E5BIQ", "" },
        { uint32_t(77), "GALILEO_E5ABI", "" },
        { uint32_t(78), "GALILEO_E5ABQ", "" },
        { uint32_t(79), "GALILEO_E5ABIQ", "" },
        { uint32_t(80), "GALILEO_E5AI", "" },
        { uint32_t(81), "GALILEO_E5AQ", "" },
        { uint32_t(82), "GALILEO_E5AIQ", "" },
        { uint32_t(96), "SBAS_L1CA", "" },
        { uint32_t(97), "SBAS_L5I", "" },
        { uint32_t(98), "SBAS_L5Q", "" },
        { uint32_t(99), "SBAS_L5IQ", "" },
        { uint32_t(128), "QZSS_L1CA", "" },
        { uint32_t(129), "QZSS_LEXS", "" },
        { uint32_t(130), "QZSS_LEXL", "" },
        { uint32_t(131), "QZSS_LEXSL", "" },
        { uint32_t(132), "QZSS_L2CM", "" },
        { uint32_t(133), "QZSS_L2CL", "" },
        { uint32_t(134), "QZSS_L2CML", "" },
        { uint32_t(135), "QZSS_L5I", "" },
        { uint32_t(136), "QZSS_L5Q", "" },
        { uint32_t(137), "QZSS_L5IQ", "" },
        { uint32_t(138), "QZSS_L1CD", "" },
        { uint32_t(139), "QZSS_L1CP", "" },
        { uint32_t(140), "QZSS_L1CDP", "" },
        { uint32_t(160), "BEIDOU_B1I", "" },
        { uint32_t(161), "BEIDOU_B1Q", "" },
        { uint32_t(162), "BEIDOU_B1IQ", "" },
        { uint32_t(163), "BEIDOU_B3I", "" },
        { uint32_t(164), "BEIDOU_B3Q", "" },
        { uint32_t(165), "BEIDOU_B3IQ", "" },
        { uint32_t(166), "BEIDOU_B2I", "" },
        { uint32_t(167), "BEIDOU_B2Q", "" },
        { uint32_t(168), "BEIDOU_B2IQ", "" },
        { uint32_t(169), "BEIDOU_B2A", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "GnssSignalId",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::Raw::GnssSignalQuality>
{
    using type = data_gnss::Raw::GnssSignalQuality;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "NONE", "" },
        { uint32_t(1), "SEARCHING", "" },
        { uint32_t(2), "ACQUIRED", "" },
        { uint32_t(3), "UNUSABLE", "" },
        { uint32_t(4), "TIME_LOCKED", "" },
        { uint32_t(5), "FULLY_LOCKED", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "GnssSignalQuality",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::Raw::ValidFlags>
{
    using type = data_gnss::Raw::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "receiver_id", "" },
        { uint32_t(8), "tracking_channel", "" },
        { uint32_t(16), "gnss_id", "" },
        { uint32_t(32), "satellite_id", "" },
        { uint32_t(64), "signal_id", "" },
        { uint32_t(128), "signal_strength", "" },
        { uint32_t(256), "quality", "" },
        { uint32_t(512), "pseudorange", "" },
        { uint32_t(1024), "carrier_phase", "" },
        { uint32_t(2048), "doppler", "" },
        { uint32_t(4096), "range_uncertainty", "" },
        { uint32_t(8192), "carrier_phase_uncertainty", "" },
        { uint32_t(16384), "doppler_uncertainty", "" },
        { uint32_t(32768), "lock_time", "" },
        { uint32_t(65535), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::Raw>
{
    using type = data_gnss::Raw;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "When the measurement comes from RTCM, this will be the reference station ID; otherwise, it's the receiver number (1,2,...)",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tracking_channel",
            /* .docs          = */ "Channel the receiver is using to track this satellite.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::tracking_channel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gnss_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssConstellationId>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GnssConstellationId, &type::gnss_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::satellite_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "signal_id",
            /* .docs          = */ "Signal identifier for the satellite.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssSignalId>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GnssSignalId, &type::signal_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "signal_strength",
            /* .docs          = */ "Carrier to noise ratio [dBHz].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::signal_strength>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "quality",
            /* .docs          = */ "Indicator of signal quality.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::Raw::GnssSignalQuality>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::Raw::GnssSignalQuality, &type::quality>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pseudorange",
            /* .docs          = */ "Pseudo-range measurement [meters].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::pseudorange>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "carrier_phase",
            /* .docs          = */ "Carrier phase measurement [Carrier periods].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::carrier_phase>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "doppler",
            /* .docs          = */ "Measured doppler shift [Hz].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::doppler>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_uncert",
            /* .docs          = */ "Uncertainty of the pseudo-range measurement [m].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::range_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "phase_uncert",
            /* .docs          = */ "Uncertainty of the phase measurement [Carrier periods].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::phase_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "doppler_uncert",
            /* .docs          = */ "Uncertainty of the measured doppler shift [Hz].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::doppler_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lock_time",
            /* .docs          = */ "DOC\nMinimum carrier phase lock time [s].  Note: the maximum value is dependent on the receiver.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::lock_time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::Raw::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::Raw::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::Raw",
        /* .title       = */ "raw",
        /* .docs        = */ "GNSS Raw observation.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsEphemeris::ValidFlags>
{
    using type = data_gnss::GpsEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "ephemeris", "" },
        { uint32_t(2), "modern_data", "" },
        { uint32_t(4), "isc_l5", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GpsEphemeris>
{
    using type = data_gnss::GpsEphemeris;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::satellite_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite and signal health",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::health>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iodc",
            /* .docs          = */ "Issue of Data Clock. This increments each time the data changes and\nrolls over at 4. It is used to make sure various raw data elements from\ndifferent sources line up correctly.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::iodc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iode",
            /* .docs          = */ "Issue of Data Ephemeris.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::iode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oc",
            /* .docs          = */ "Reference time for clock data.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_oc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af0",
            /* .docs          = */ "Clock bias in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af0>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af1",
            /* .docs          = */ "Clock drift in [s/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af1>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af2",
            /* .docs          = */ "Clock drift rate in [s/s^2].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af2>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_gd",
            /* .docs          = */ "T Group Delay [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_gd>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L1CA",
            /* .docs          = */ "Inter-signal correction (L1).",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ISC_L1CA>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L2C",
            /* .docs          = */ "Inter-signal correction (L2, or L5 if isc_l5 flag is set).",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ISC_L2C>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oe",
            /* .docs          = */ "Reference time for ephemeris in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_oe>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Semi-major axis [m].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::a>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a_dot",
            /* .docs          = */ "Semi-major axis rate [m/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::a_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mean_anomaly",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::mean_anomaly>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::delta_mean_motion>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion_dot",
            /* .docs          = */ "[rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::delta_mean_motion_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "eccentricity",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::eccentricity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "argument_of_perigee",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::argument_of_perigee>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega",
            /* .docs          = */ "Longitude of Ascending Node [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::omega>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega_dot",
            /* .docs          = */ "Rate of Right Ascension [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::omega_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::inclination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination_dot",
            /* .docs          = */ "Inclination angle rate of change [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::inclination_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_ic",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_ic>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_is",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_is>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_uc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_uc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_us",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_us>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_rc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rs",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_rs>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GpsEphemeris::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GpsEphemeris::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsEphemeris",
        /* .title       = */ "GPS Ephemeris",
        /* .docs        = */ "GPS Ephemeris Data",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GalileoEphemeris::ValidFlags>
{
    using type = data_gnss::GalileoEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "ephemeris", "" },
        { uint32_t(2), "modern_data", "" },
        { uint32_t(4), "isc_l5", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GalileoEphemeris>
{
    using type = data_gnss::GalileoEphemeris;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::satellite_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite and signal health",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::health>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iodc",
            /* .docs          = */ "Issue of Data Clock. This increments each time the data changes and\nrolls over at 4. It is used to make sure various raw data elements from\ndifferent sources line up correctly.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::iodc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iode",
            /* .docs          = */ "Issue of Data Ephemeris.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::iode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oc",
            /* .docs          = */ "Reference time for clock data.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_oc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af0",
            /* .docs          = */ "Clock bias in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af0>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af1",
            /* .docs          = */ "Clock drift in [s/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af1>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af2",
            /* .docs          = */ "Clock drift rate in [s/s^2].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af2>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_gd",
            /* .docs          = */ "T Group Delay [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_gd>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L1CA",
            /* .docs          = */ "Inter-signal correction (L1).",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ISC_L1CA>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L2C",
            /* .docs          = */ "Inter-signal correction (L2, or L5 if isc_l5 flag is set).",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ISC_L2C>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oe",
            /* .docs          = */ "Reference time for ephemeris in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_oe>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Semi-major axis [m].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::a>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a_dot",
            /* .docs          = */ "Semi-major axis rate [m/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::a_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mean_anomaly",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::mean_anomaly>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::delta_mean_motion>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion_dot",
            /* .docs          = */ "[rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::delta_mean_motion_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "eccentricity",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::eccentricity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "argument_of_perigee",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::argument_of_perigee>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega",
            /* .docs          = */ "Longitude of Ascending Node [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::omega>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega_dot",
            /* .docs          = */ "Rate of Right Ascension [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::omega_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::inclination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination_dot",
            /* .docs          = */ "Inclination angle rate of change [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::inclination_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_ic",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_ic>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_is",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_is>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_uc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_uc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_us",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_us>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_rc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rs",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_rs>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GalileoEphemeris::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GalileoEphemeris::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GalileoEphemeris",
        /* .title       = */ "Galileo Ephemeris",
        /* .docs        = */ "Galileo Ephemeris Data",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GloEphemeris::ValidFlags>
{
    using type = data_gnss::GloEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "ephemeris", "" },
        { uint32_t(1), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GloEphemeris>
{
    using type = data_gnss::GloEphemeris;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::satellite_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "freq_number",
            /* .docs          = */ "GLONASS frequency number (-7 to 24)",
            /* .type          = */ {Type::S8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, int8_t, &type::freq_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tk",
            /* .docs          = */ "Frame start time within current day [seconds]",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::tk>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tb",
            /* .docs          = */ "Ephemeris reference time [seconds]",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::tb>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sat_type",
            /* .docs          = */ "Type of satellite (M) GLONASS = 0, GLONASS-M = 1",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::sat_type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gamma",
            /* .docs          = */ "Relative deviation of carrier frequency from nominal [dimensionless]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::gamma>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tau_n",
            /* .docs          = */ "Time correction relative to GLONASS Time [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::tau_n>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "x",
            /* .docs          = */ "Satellite PE-90 position [m]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::x>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "v",
            /* .docs          = */ "Satellite PE-90 velocity [m/s]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::v>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Satellite PE-90 acceleration due to perturbations [m/s^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::a>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite Health (Bn), Non-zero indicates satellite malfunction",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::health>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P",
            /* .docs          = */ "Satellite operation mode (See GLONASS ICD)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::P>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "NT",
            /* .docs          = */ "Day number within a 4 year period.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::NT>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_tau_n",
            /* .docs          = */ "Time difference between L1 and L2[m/s]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::delta_tau_n>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "Ft",
            /* .docs          = */ "User Range Accuracy (See GLONASS ICD)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::Ft>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "En",
            /* .docs          = */ "Age of current information [days]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::En>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P1",
            /* .docs          = */ "Time interval between adjacent values of tb [minutes]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::P1>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P2",
            /* .docs          = */ "Oddness '1' or evenness '0' of the value of tb.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::P2>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P3",
            /* .docs          = */ "Number of satellites in almanac for this frame",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::P3>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P4",
            /* .docs          = */ "Flag indicating ephemeris parameters are present",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::P4>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GloEphemeris::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GloEphemeris::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GloEphemeris",
        /* .title       = */ "Glonass Ephemeris",
        /* .docs        = */ "Glonass Ephemeris Data",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::BeidouEphemeris::ValidFlags>
{
    using type = data_gnss::BeidouEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "ephemeris", "" },
        { uint32_t(2), "modern_data", "" },
        { uint32_t(4), "isc_l5", "" },
        { uint32_t(7), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::BeidouEphemeris>
{
    using type = data_gnss::BeidouEphemeris;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "index",
            /* .docs          = */ "Index of this field in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::index>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::count>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::satellite_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite and signal health",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::health>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iodc",
            /* .docs          = */ "Issue of Data Clock. This increments each time the data changes and\nrolls over at 4. It is used to make sure various raw data elements from\ndifferent sources line up correctly.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::iodc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iode",
            /* .docs          = */ "Issue of Data Ephemeris.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::iode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oc",
            /* .docs          = */ "Reference time for clock data.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_oc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af0",
            /* .docs          = */ "Clock bias in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af0>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af1",
            /* .docs          = */ "Clock drift in [s/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af1>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af2",
            /* .docs          = */ "Clock drift rate in [s/s^2].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::af2>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_gd",
            /* .docs          = */ "T Group Delay [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_gd>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L1CA",
            /* .docs          = */ "Inter-signal correction (L1).",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ISC_L1CA>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L2C",
            /* .docs          = */ "Inter-signal correction (L2, or L5 if isc_l5 flag is set).",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::ISC_L2C>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oe",
            /* .docs          = */ "Reference time for ephemeris in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::t_oe>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Semi-major axis [m].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::a>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a_dot",
            /* .docs          = */ "Semi-major axis rate [m/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::a_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mean_anomaly",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::mean_anomaly>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::delta_mean_motion>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion_dot",
            /* .docs          = */ "[rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::delta_mean_motion_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "eccentricity",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::eccentricity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "argument_of_perigee",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::argument_of_perigee>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega",
            /* .docs          = */ "Longitude of Ascending Node [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::omega>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega_dot",
            /* .docs          = */ "Rate of Right Ascension [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::omega_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::inclination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination_dot",
            /* .docs          = */ "Inclination angle rate of change [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::inclination_dot>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_ic",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_ic>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_is",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_is>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_uc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_uc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_us",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_us>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_rc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rs",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::c_rs>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::BeidouEphemeris::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::BeidouEphemeris::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::BeidouEphemeris",
        /* .title       = */ "BeiDou Ephemeris",
        /* .docs        = */ "BeiDou Ephemeris Data",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsIonoCorr::ValidFlags>
{
    using type = data_gnss::GpsIonoCorr::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "alpha", "" },
        { uint32_t(8), "beta", "" },
        { uint32_t(15), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GpsIonoCorr>
{
    using type = data_gnss::GpsIonoCorr;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "alpha",
            /* .docs          = */ "Ionospheric Correction Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::alpha>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Ionospheric Correction Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::beta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GpsIonoCorr::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GpsIonoCorr::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsIonoCorr",
        /* .title       = */ "GPS Ionospheric Correction",
        /* .docs        = */ "Ionospheric Correction Terms for GNSS",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GalileoIonoCorr::ValidFlags>
{
    using type = data_gnss::GalileoIonoCorr::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "alpha", "" },
        { uint32_t(8), "disturbance_flags", "" },
        { uint32_t(15), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::GalileoIonoCorr>
{
    using type = data_gnss::GalileoIonoCorr;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "alpha",
            /* .docs          = */ "Coefficients for the model.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::alpha>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "disturbance_flags",
            /* .docs          = */ "Region disturbance flags (bits 1-5).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::disturbance_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::GalileoIonoCorr::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::GalileoIonoCorr::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GalileoIonoCorr",
        /* .title       = */ "Galileo Ionospheric Correction",
        /* .docs        = */ "Ionospheric Correction Terms for Galileo",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::BeidouIonoCorr::ValidFlags>
{
    using type = data_gnss::BeidouIonoCorr::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tow", "" },
        { uint32_t(2), "week_number", "" },
        { uint32_t(4), "alpha", "" },
        { uint32_t(8), "beta", "" },
        { uint32_t(16), "alpha_corr", "" },
        { uint32_t(31), "flags", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_gnss::BeidouIonoCorr>
{
    using type = data_gnss::BeidouIonoCorr;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::week_number>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "alpha",
            /* .docs          = */ "Ionospheric Delay Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::alpha>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Ionospheric Delay Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::beta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "alpha_corr",
            /* .docs          = */ "Ionospheric Delay Correction Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::alpha_corr>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 9,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_gnss::BeidouIonoCorr::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_gnss::BeidouIonoCorr::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::BeidouIonoCorr",
        /* .title       = */ "BeiDou Ionospheric Correction",
        /* .docs        = */ "Ionospheric Correction Terms for BeiDou",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* DATA_GNSS_FIELDS[] = {
    &MetadataFor<data_gnss::PosLlh>::value,
    &MetadataFor<data_gnss::PosEcef>::value,
    &MetadataFor<data_gnss::VelNed>::value,
    &MetadataFor<data_gnss::VelEcef>::value,
    &MetadataFor<data_gnss::Dop>::value,
    &MetadataFor<data_gnss::UtcTime>::value,
    &MetadataFor<data_gnss::GpsTime>::value,
    &MetadataFor<data_gnss::ClockInfo>::value,
    &MetadataFor<data_gnss::FixInfo>::value,
    &MetadataFor<data_gnss::SvInfo>::value,
    &MetadataFor<data_gnss::HwStatus>::value,
    &MetadataFor<data_gnss::DgpsInfo>::value,
    &MetadataFor<data_gnss::DgpsChannel>::value,
    &MetadataFor<data_gnss::ClockInfo2>::value,
    &MetadataFor<data_gnss::GpsLeapSeconds>::value,
    &MetadataFor<data_gnss::SbasInfo>::value,
    &MetadataFor<data_gnss::SbasCorrection>::value,
    &MetadataFor<data_gnss::RfErrorDetection>::value,
    &MetadataFor<data_gnss::SatelliteStatus>::value,
    &MetadataFor<data_gnss::Raw>::value,
    &MetadataFor<data_gnss::BaseStationInfo>::value,
    &MetadataFor<data_gnss::RtkCorrectionsStatus>::value,
    &MetadataFor<data_gnss::GpsEphemeris>::value,
    &MetadataFor<data_gnss::GloEphemeris>::value,
    &MetadataFor<data_gnss::GalileoEphemeris>::value,
    &MetadataFor<data_gnss::BeidouEphemeris>::value,
    &MetadataFor<data_gnss::GpsIonoCorr>::value,
    &MetadataFor<data_gnss::GalileoIonoCorr>::value,
    &MetadataFor<data_gnss::BeidouIonoCorr>::value,
};

static constexpr DescriptorSetInfo DATA_GNSS = {
    /*.descriptor =*/ mip::data_gnss::DESCRIPTOR_SET,
    /*.name       =*/ "Gnss Data",
    /*.fields     =*/ DATA_GNSS_FIELDS,
};

} // namespace mip::metadata

