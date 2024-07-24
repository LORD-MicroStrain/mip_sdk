#pragma once

#include "common.hpp"

#include <mip/definitions/data_gnss.hpp>

#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<data_gnss::PosLlh::ValidFlags>
{
    using type = data_gnss::PosLlh::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "lat_lon", "" },
        { 2, "ellipsoid_height", "" },
        { 4, "msl_height", "" },
        { 8, "horizontal_accuracy", "" },
        { 16, "vertical_accuracy", "" },
        { 31, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::latitude>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "longitude",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::longitude>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ellipsoid_height",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::ellipsoid_height>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "msl_height",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::msl_height>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "horizontal_accuracy",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::horizontal_accuracy>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "vertical_accuracy",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::vertical_accuracy>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::PosLlh::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::PosLlh::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::PosEcef::ValidFlags>
{
    using type = data_gnss::PosEcef::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "position", "" },
        { 2, "position_accuracy", "" },
        { 3, "flags", "" },
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
            /* .accessor      = */ utils::access<type, Vector3d, &type::x>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "x_accuracy",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::x_accuracy>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::PosEcef::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::PosEcef::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::VelNed::ValidFlags>
{
    using type = data_gnss::VelNed::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "velocity", "" },
        { 2, "speed_3d", "" },
        { 4, "ground_speed", "" },
        { 8, "heading", "" },
        { 16, "speed_accuracy", "" },
        { 32, "heading_accuracy", "" },
        { 63, "flags", "" },
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
            /* .accessor      = */ utils::access<type, Vector3f, &type::v>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "speed",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::speed>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ground_speed",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::ground_speed>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::heading>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "speed_accuracy",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::speed_accuracy>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading_accuracy",
            /* .docs          = */ "[degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::heading_accuracy>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::VelNed::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::VelNed::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::VelEcef::ValidFlags>
{
    using type = data_gnss::VelEcef::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "velocity", "" },
        { 2, "velocity_accuracy", "" },
        { 3, "flags", "" },
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
            /* .accessor      = */ utils::access<type, Vector3f, &type::v>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "v_accuracy",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::v_accuracy>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::VelEcef::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::VelEcef::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::Dop::ValidFlags>
{
    using type = data_gnss::Dop::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "gdop", "" },
        { 2, "pdop", "" },
        { 4, "hdop", "" },
        { 8, "vdop", "" },
        { 16, "tdop", "" },
        { 32, "ndop", "" },
        { 64, "edop", "" },
        { 127, "flags", "" },
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
            /* .accessor      = */ utils::access<type, float, &type::gdop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pdop",
            /* .docs          = */ "Position DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::pdop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "hdop",
            /* .docs          = */ "Horizontal DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::hdop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "vdop",
            /* .docs          = */ "Vertical DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::vdop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tdop",
            /* .docs          = */ "Time DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::tdop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ndop",
            /* .docs          = */ "Northing DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::ndop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "edop",
            /* .docs          = */ "Easting DOP",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::edop>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::Dop::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::Dop::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::Dop",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported dilution of precision information.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::UtcTime::ValidFlags>
{
    using type = data_gnss::UtcTime::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "gnss_date_time", "" },
        { 2, "leap_seconds_known", "" },
        { 3, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint16_t, &type::year>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "month",
            /* .docs          = */ "Month (1-12)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::month>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "day",
            /* .docs          = */ "Day (1-31)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::day>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "hour",
            /* .docs          = */ "Hour (0-23)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::hour>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "min",
            /* .docs          = */ "Minute (0-59)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::min>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sec",
            /* .docs          = */ "Second (0-59)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::sec>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "msec",
            /* .docs          = */ "Millisecond(0-999)",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ utils::access<type, uint32_t, &type::msec>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::UtcTime::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::UtcTime::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::UtcTime",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported Coordinated Universal Time",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsTime::ValidFlags>
{
    using type = data_gnss::GpsTime::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 3, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::tow>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GpsTime::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GpsTime::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsTime",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported GPS Time",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::ClockInfo::ValidFlags>
{
    using type = data_gnss::ClockInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "bias", "" },
        { 2, "drift", "" },
        { 4, "accuracy_estimate", "" },
        { 7, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::bias>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "drift",
            /* .docs          = */ "[seconds/second]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::drift>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "accuracy_estimate",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::accuracy_estimate>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::ClockInfo::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::ClockInfo::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::ClockInfo",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported receiver clock parameters",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::FixInfo::FixType>
{
    using type = data_gnss::FixInfo::FixType;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "FIX_3D", "" },
        { 1, "FIX_2D", "" },
        { 2, "FIX_TIME_ONLY", "" },
        { 3, "FIX_NONE", "" },
        { 4, "FIX_INVALID", "" },
        { 5, "FIX_RTK_FLOAT", "" },
        { 6, "FIX_RTK_FIXED", "" },
        { 7, "FIX_DIFFERENTIAL", "" },
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
        { 1, "sbas_used", "" },
        { 2, "dgnss_used", "" },
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
        { 1, "fix_type", "" },
        { 2, "num_sv", "" },
        { 4, "fix_flags", "" },
        { 7, "flags", "" },
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
            /* .accessor      = */ utils::access<type, data_gnss::FixInfo::FixType, &type::fix_type>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "num_sv",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::num_sv>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "fix_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::FixInfo::FixFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::FixInfo::FixFlags, &type::fix_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::FixInfo::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::FixInfo::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::FixInfo",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported position fix type",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::SvInfo::SVFlags>
{
    using type = data_gnss::SvInfo::SVFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "used_for_navigation", "" },
        { 2, "healthy", "" },
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
        { 1, "channel", "" },
        { 2, "sv_id", "" },
        { 4, "carrier_noise_ratio", "" },
        { 8, "azimuth", "" },
        { 16, "elevation", "" },
        { 32, "sv_flags", "" },
        { 63, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::channel>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sv_id",
            /* .docs          = */ "GNSS Satellite ID",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::sv_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "carrier_noise_ratio",
            /* .docs          = */ "[dBHz]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::carrier_noise_ratio>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "azimuth",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::S16, nullptr},
            /* .accessor      = */ utils::access<type, int16_t, &type::azimuth>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "elevation",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::S16, nullptr},
            /* .accessor      = */ utils::access<type, int16_t, &type::elevation>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sv_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::SvInfo::SVFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SvInfo::SVFlags, &type::sv_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::SvInfo::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SvInfo::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SvInfo",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported space vehicle information\n\nWhen enabled, these fields will arrive in separate MIP packets",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::HwStatus::ReceiverState>
{
    using type = data_gnss::HwStatus::ReceiverState;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "OFF", "" },
        { 1, "ON", "" },
        { 2, "UNKNOWN", "" },
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
        { 1, "INIT", "" },
        { 2, "SHORT", "" },
        { 3, "OPEN", "" },
        { 4, "GOOD", "" },
        { 5, "UNKNOWN", "" },
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
        { 0, "OFF", "" },
        { 1, "ON", "" },
        { 2, "UNKNOWN", "" },
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
        { 1, "sensor_state", "" },
        { 2, "antenna_state", "" },
        { 4, "antenna_power", "" },
        { 7, "flags", "" },
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
            /* .accessor      = */ utils::access<type, data_gnss::HwStatus::ReceiverState, &type::receiver_state>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "antenna_state",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::HwStatus::AntennaState>::value},
            /* .accessor      = */ utils::access<type, data_gnss::HwStatus::AntennaState, &type::antenna_state>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "antenna_power",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::HwStatus::AntennaPower>::value},
            /* .accessor      = */ utils::access<type, data_gnss::HwStatus::AntennaPower, &type::antenna_power>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::HwStatus::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::HwStatus::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::DgpsInfo::ValidFlags>
{
    using type = data_gnss::DgpsInfo::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "age", "" },
        { 2, "base_station_id", "" },
        { 4, "base_station_status", "" },
        { 8, "num_channels", "" },
        { 15, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::sv_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "age",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::age>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_correction",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::range_correction>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_rate_correction",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::range_rate_correction>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::DgpsInfo::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::DgpsInfo::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::DgpsInfo",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported DGNSS status\n\n<pre>Possible Base Station Status Values:</pre>\n<pre>  0 - UDRE Scale Factor = 1.0</pre>\n<pre>  1 - UDRE Scale Factor = 0.75</pre>\n<pre>  2 - UDRE Scale Factor = 0.5</pre>\n<pre>  3 - UDRE Scale Factor = 0.3</pre>\n<pre>  4 - UDRE Scale Factor = 0.2</pre>\n<pre>  5 - UDRE Scale Factor = 0.1</pre>\n<pre>  6 - Reference Station Transmission Not Monitored</pre>\n<pre>  7 - Reference Station Not Working</pre>\n\n(UDRE = User Differential Range Error)",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::DgpsChannel::ValidFlags>
{
    using type = data_gnss::DgpsChannel::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "id", "" },
        { 2, "age", "" },
        { 4, "range_correction", "" },
        { 8, "range_rate_correction", "" },
        { 15, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::sv_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "age",
            /* .docs          = */ "[s]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::age>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_correction",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::range_correction>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_rate_correction",
            /* .docs          = */ "[m/s]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::range_rate_correction>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::DgpsChannel::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::DgpsChannel::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::DgpsChannel",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported DGPS Channel Status status\n\nWhen enabled, a separate field for each active space vehicle will be sent in the packet.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::ClockInfo2::ValidFlags>
{
    using type = data_gnss::ClockInfo2::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "bias", "" },
        { 2, "drift", "" },
        { 4, "bias_accuracy", "" },
        { 8, "drift_accuracy", "" },
        { 15, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::bias>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "drift",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::drift>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "bias_accuracy_estimate",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::bias_accuracy_estimate>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "drift_accuracy_estimate",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::drift_accuracy_estimate>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::ClockInfo2::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::ClockInfo2::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::ClockInfo2",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported receiver clock parameters\n\nThis supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsLeapSeconds::ValidFlags>
{
    using type = data_gnss::GpsLeapSeconds::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 2, "leap_seconds", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::leap_seconds>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GpsLeapSeconds::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GpsLeapSeconds::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsLeapSeconds",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS reported leap seconds (difference between GPS and UTC Time)",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::SbasSystem>
{
    using type = data_gnss::SbasSystem;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "UNKNOWN", "" },
        { 1, "WAAS", "" },
        { 2, "EGNOS", "" },
        { 3, "MSAS", "" },
        { 4, "GAGAN", "" },
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
        { 1, "range_available", "" },
        { 2, "corrections_available", "" },
        { 4, "integrity_available", "" },
        { 8, "test_mode", "" },
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
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "sbas_system", "" },
        { 8, "sbas_id", "" },
        { 16, "count", "" },
        { 32, "sbas_status", "" },
        { 63, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sbas_system",
            /* .docs          = */ "SBAS system id",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::SbasSystem>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SbasSystem, &type::sbas_system>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sbas_id",
            /* .docs          = */ "SBAS satellite id.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::sbas_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Number of SBAS corrections",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sbas_status",
            /* .docs          = */ "Status of the SBAS service",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::SbasInfo::SbasStatus>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SbasInfo::SbasStatus, &type::sbas_status>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::SbasInfo::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SbasInfo::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SbasInfo",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS SBAS status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GnssConstellationId>
{
    using type = data_gnss::GnssConstellationId;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "UNKNOWN", "" },
        { 1, "GPS", "" },
        { 2, "GLONASS", "" },
        { 3, "GALILEO", "" },
        { 4, "BEIDOU", "" },
        { 5, "SBAS", "" },
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
        { 1, "udrei", "" },
        { 2, "pseudorange_correction", "" },
        { 4, "iono_correction", "" },
        { 7, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::index>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week the message was received [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gnss_id",
            /* .docs          = */ "GNSS constellation id",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssConstellationId>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GnssConstellationId, &type::gnss_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sv_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::sv_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "udrei",
            /* .docs          = */ "[See above 0-13 usable, 14 not monitored, 15 - do not use]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::udrei>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pseudorange_correction",
            /* .docs          = */ "Pseudo-range correction [meters].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::pseudorange_correction>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iono_correction",
            /* .docs          = */ "Ionospheric correction [meters].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::iono_correction>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::SbasCorrection::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SbasCorrection::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SbasCorrection",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS calculated SBAS Correction\n\nUDREI - the variance of a normal distribution associated with the user differential range errors for a\nsatellite after application of fast and long-term corrections, excluding atmospheric effects\n\n<pre>UDREI  Variance</pre>\n<pre>-----------------------</pre>\n<pre>0      0.0520 m^2</pre>\n<pre>1      0.0924 m^2</pre>\n<pre>2      0.1444 m^2</pre>\n<pre>3      0.2830 m^2</pre>\n<pre>4      0.4678 m^2</pre>\n<pre>5      0.8315 m^2</pre>\n<pre>6      1.2992 m^2</pre>\n<pre>7      1.8709 m^2</pre>\n<pre>8      2.5465 m^2</pre>\n<pre>9      3.3260 m^2</pre>\n<pre>10     5.1968 m^2</pre>\n<pre>11     20.7870 m^2</pre>\n<pre>12     230.9661 m^2</pre>\n<pre>13     2078.695 m^2</pre>\n<pre>14     'Not Monitored'</pre>\n<pre>15     'Do Not Use'</pre>",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::RfErrorDetection::RFBand>
{
    using type = data_gnss::RfErrorDetection::RFBand;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "UNKNOWN", "" },
        { 1, "L1", "" },
        { 2, "L2", "" },
        { 5, "L5", "" },
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
        { 0, "UNKNOWN", "" },
        { 1, "NONE", "" },
        { 2, "PARTIAL", "" },
        { 3, "SIGNIFICANT", "" },
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
        { 0, "UNKNOWN", "" },
        { 1, "NONE", "" },
        { 2, "PARTIAL", "" },
        { 3, "SIGNIFICANT", "" },
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
        { 1, "rf_band", "" },
        { 2, "jamming_state", "" },
        { 4, "spoofing_state", "" },
        { 7, "flags", "" },
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
            /* .accessor      = */ utils::access<type, data_gnss::RfErrorDetection::RFBand, &type::rf_band>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "jamming_state",
            /* .docs          = */ "GNSS Jamming State (as reported by the GNSS module)",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::RfErrorDetection::JammingState>::value},
            /* .accessor      = */ utils::access<type, data_gnss::RfErrorDetection::JammingState, &type::jamming_state>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "spoofing_state",
            /* .docs          = */ "GNSS Spoofing State (as reported by the GNSS module)",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::RfErrorDetection::SpoofingState>::value},
            /* .accessor      = */ utils::access<type, data_gnss::RfErrorDetection::SpoofingState, &type::spoofing_state>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "Reserved for future use",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::RfErrorDetection::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::RfErrorDetection::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::RfErrorDetection",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS Error Detection subsystem status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::BaseStationInfo::IndicatorFlags>
{
    using type = data_gnss::BaseStationInfo::IndicatorFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "gps", "" },
        { 2, "glonass", "" },
        { 4, "galileo", "" },
        { 8, "beidou", "" },
        { 16, "ref_station", "" },
        { 32, "single_receiver", "" },
        { 64, "quarter_cycle_bit1", "" },
        { 128, "quarter_cycle_bit2", "" },
        { 192, "quarter_cycle_bits", "" },
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
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "ecef_position", "" },
        { 8, "height", "" },
        { 16, "station_id", "" },
        { 32, "indicators", "" },
        { 63, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ecef_pos",
            /* .docs          = */ "Earth-centered, Earth-fixed [m]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ utils::access<type, Vector3d, &type::ecef_pos>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "height",
            /* .docs          = */ "Antenna Height above the marker used in the survey [m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::height>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "station_id",
            /* .docs          = */ "Range: 0-4095",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::station_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "indicators",
            /* .docs          = */ "Bitfield",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::BaseStationInfo::IndicatorFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::BaseStationInfo::IndicatorFlags, &type::indicators>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::BaseStationInfo::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::BaseStationInfo::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::BaseStationInfo",
        /* .title       = */ "None",
        /* .docs        = */ "RTCM reported base station information (sourced from RTCM Message 1005 or 1006)\n\nValid Flag Mapping:",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::RtkCorrectionsStatus::ValidFlags>
{
    using type = data_gnss::RtkCorrectionsStatus::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "epoch_status", "" },
        { 8, "dongle_status", "" },
        { 16, "gps_latency", "" },
        { 32, "glonass_latency", "" },
        { 64, "galileo_latency", "" },
        { 128, "beidou_latency", "" },
        { 255, "flags", "" },
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
        { 1, "antenna_location_received", "" },
        { 2, "antenna_description_received", "" },
        { 4, "gps_received", "" },
        { 8, "glonass_received", "" },
        { 16, "galileo_received", "" },
        { 32, "beidou_received", "" },
        { 64, "using_gps_msm_messages", "Using MSM messages for GPS corrections instead of RTCM messages 1001-1004" },
        { 128, "using_glonass_msm_messages", "Using MSM messages for GLONASS corrections instead of RTCM messages 1009-1012" },
        { 256, "dongle_status_read_failed", "A read of the dongle status was attempted, but failed" },
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
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "epoch_status",
            /* .docs          = */ "Status of the corrections received during this epoch",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::RtkCorrectionsStatus::EpochStatus>::value},
            /* .accessor      = */ utils::access<type, data_gnss::RtkCorrectionsStatus::EpochStatus, &type::epoch_status>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "dongle_status",
            /* .docs          = */ "RTK Dongle Status Flags (valid only when using RTK dongle, see Get RTK Device Status Flags (0x0F,0x01) for details)",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ utils::access<type, uint32_t, &type::dongle_status>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gps_correction_latency",
            /* .docs          = */ "Latency of last GPS correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::gps_correction_latency>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "glonass_correction_latency",
            /* .docs          = */ "Latency of last GLONASS correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::glonass_correction_latency>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "galileo_correction_latency",
            /* .docs          = */ "Latency of last Galileo correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::galileo_correction_latency>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beidou_correction_latency",
            /* .docs          = */ "Latency of last Beidou correction [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::beidou_correction_latency>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "Reserved for future use",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ utils::access<type, uint32_t, &type::reserved>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::RtkCorrectionsStatus::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::RtkCorrectionsStatus::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::RtkCorrectionsStatus",
        /* .title       = */ "None",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::SatelliteStatus::ValidFlags>
{
    using type = data_gnss::SatelliteStatus::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "gnss_id", "" },
        { 8, "satellite_id", "" },
        { 16, "elevation", "" },
        { 32, "azimuth", "" },
        { 64, "health", "" },
        { 127, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::index>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gnss_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssConstellationId>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GnssConstellationId, &type::gnss_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::satellite_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "elevation",
            /* .docs          = */ "Elevation of the satellite relative to the rover [degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::elevation>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "azimuth",
            /* .docs          = */ "Azimuth of the satellite relative to the rover [degrees]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::azimuth>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "True if the satellite is healthy.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ utils::access<type, bool, &type::health>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::SatelliteStatus::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::SatelliteStatus::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::SatelliteStatus",
        /* .title       = */ "None",
        /* .docs        = */ "Status information for a GNSS satellite.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GnssSignalId>
{
    using type = data_gnss::GnssSignalId;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "UNKNOWN", "" },
        { 1, "GPS_L1CA", "" },
        { 2, "GPS_L1P", "" },
        { 3, "GPS_L1Z", "" },
        { 4, "GPS_L2CA", "" },
        { 5, "GPS_L2P", "" },
        { 6, "GPS_L2Z", "" },
        { 7, "GPS_L2CL", "" },
        { 8, "GPS_L2CM", "" },
        { 9, "GPS_L2CML", "" },
        { 10, "GPS_L5I", "" },
        { 11, "GPS_L5Q", "" },
        { 12, "GPS_L5IQ", "" },
        { 13, "GPS_L1CD", "" },
        { 14, "GPS_L1CP", "" },
        { 15, "GPS_L1CDP", "" },
        { 32, "GLONASS_G1CA", "" },
        { 33, "GLONASS_G1P", "" },
        { 34, "GLONASS_G2C", "" },
        { 35, "GLONASS_G2P", "" },
        { 64, "GALILEO_E1C", "" },
        { 65, "GALILEO_E1A", "" },
        { 66, "GALILEO_E1B", "" },
        { 67, "GALILEO_E1BC", "" },
        { 68, "GALILEO_E1ABC", "" },
        { 69, "GALILEO_E6C", "" },
        { 70, "GALILEO_E6A", "" },
        { 71, "GALILEO_E6B", "" },
        { 72, "GALILEO_E6BC", "" },
        { 73, "GALILEO_E6ABC", "" },
        { 74, "GALILEO_E5BI", "" },
        { 75, "GALILEO_E5BQ", "" },
        { 76, "GALILEO_E5BIQ", "" },
        { 77, "GALILEO_E5ABI", "" },
        { 78, "GALILEO_E5ABQ", "" },
        { 79, "GALILEO_E5ABIQ", "" },
        { 80, "GALILEO_E5AI", "" },
        { 81, "GALILEO_E5AQ", "" },
        { 82, "GALILEO_E5AIQ", "" },
        { 96, "SBAS_L1CA", "" },
        { 97, "SBAS_L5I", "" },
        { 98, "SBAS_L5Q", "" },
        { 99, "SBAS_L5IQ", "" },
        { 128, "QZSS_L1CA", "" },
        { 129, "QZSS_LEXS", "" },
        { 130, "QZSS_LEXL", "" },
        { 131, "QZSS_LEXSL", "" },
        { 132, "QZSS_L2CM", "" },
        { 133, "QZSS_L2CL", "" },
        { 134, "QZSS_L2CML", "" },
        { 135, "QZSS_L5I", "" },
        { 136, "QZSS_L5Q", "" },
        { 137, "QZSS_L5IQ", "" },
        { 138, "QZSS_L1CD", "" },
        { 139, "QZSS_L1CP", "" },
        { 140, "QZSS_L1CDP", "" },
        { 160, "BEIDOU_B1I", "" },
        { 161, "BEIDOU_B1Q", "" },
        { 162, "BEIDOU_B1IQ", "" },
        { 163, "BEIDOU_B3I", "" },
        { 164, "BEIDOU_B3Q", "" },
        { 165, "BEIDOU_B3IQ", "" },
        { 166, "BEIDOU_B2I", "" },
        { 167, "BEIDOU_B2Q", "" },
        { 168, "BEIDOU_B2IQ", "" },
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
        { 0, "NONE", "" },
        { 1, "SEARCHING", "" },
        { 2, "ACQUIRED", "" },
        { 3, "UNUSABLE", "" },
        { 4, "TIME_LOCKED", "" },
        { 5, "FULLY_LOCKED", "" },
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
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "receiver_id", "" },
        { 8, "tracking_channel", "" },
        { 16, "gnss_id", "" },
        { 32, "satellite_id", "" },
        { 64, "signal_id", "" },
        { 128, "signal_strength", "" },
        { 256, "quality", "" },
        { 512, "pseudorange", "" },
        { 1024, "carrier_phase", "" },
        { 2048, "doppler", "" },
        { 4096, "range_uncertainty", "" },
        { 8192, "carrier_phase_uncertainty", "" },
        { 16384, "doppler_uncertainty", "" },
        { 32768, "lock_time", "" },
        { 65535, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::index>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "When the measurement comes from RTCM, this will be the reference station ID; otherwise, it's the receiver number (1,2,...)",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::receiver_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tracking_channel",
            /* .docs          = */ "Channel the receiver is using to track this satellite.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::tracking_channel>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gnss_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssConstellationId>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GnssConstellationId, &type::gnss_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::satellite_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "signal_id",
            /* .docs          = */ "Signal identifier for the satellite.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::GnssSignalId>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GnssSignalId, &type::signal_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "signal_strength",
            /* .docs          = */ "Carrier to noise ratio [dBHz].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::signal_strength>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "quality",
            /* .docs          = */ "Indicator of signal quality.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_gnss::Raw::GnssSignalQuality>::value},
            /* .accessor      = */ utils::access<type, data_gnss::Raw::GnssSignalQuality, &type::quality>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pseudorange",
            /* .docs          = */ "Pseudo-range measurement [meters].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::pseudorange>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "carrier_phase",
            /* .docs          = */ "Carrier phase measurement [Carrier periods].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::carrier_phase>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "doppler",
            /* .docs          = */ "Measured doppler shift [Hz].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::doppler>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "range_uncert",
            /* .docs          = */ "Uncertainty of the pseudo-range measurement [m].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::range_uncert>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "phase_uncert",
            /* .docs          = */ "Uncertainty of the phase measurement [Carrier periods].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::phase_uncert>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "doppler_uncert",
            /* .docs          = */ "Uncertainty of the measured doppler shift [Hz].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::doppler_uncert>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lock_time",
            /* .docs          = */ "DOC\nMinimum carrier phase lock time [s].  Note: the maximum value is dependent on the receiver.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::lock_time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::Raw::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::Raw::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::Raw",
        /* .title       = */ "None",
        /* .docs        = */ "GNSS Raw observation.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsEphemeris::ValidFlags>
{
    using type = data_gnss::GpsEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "ephemeris", "" },
        { 2, "modern_data", "" },
        { 3, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::index>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::satellite_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite and signal health",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::health>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iodc",
            /* .docs          = */ "Issue of Data Clock. This increments each time the data changes and\nrolls over at 4. It is used to make sure various raw data elements from\ndifferent sources line up correctly.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::iodc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iode",
            /* .docs          = */ "Issue of Data Ephemeris.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::iode>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oc",
            /* .docs          = */ "Reference time for clock data.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::t_oc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af0",
            /* .docs          = */ "Clock bias in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::af0>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af1",
            /* .docs          = */ "Clock drift in [s/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::af1>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af2",
            /* .docs          = */ "Clock drift rate in [s/s^2].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::af2>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_gd",
            /* .docs          = */ "T Group Delay [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::t_gd>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L1CA",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::ISC_L1CA>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L2C",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::ISC_L2C>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oe",
            /* .docs          = */ "Reference time for ephemeris in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::t_oe>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Semi-major axis [m].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::a>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a_dot",
            /* .docs          = */ "Semi-major axis rate [m/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::a_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mean_anomaly",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::mean_anomaly>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::delta_mean_motion>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion_dot",
            /* .docs          = */ "[rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::delta_mean_motion_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "eccentricity",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::eccentricity>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "argument_of_perigee",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::argument_of_perigee>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega",
            /* .docs          = */ "Longitude of Ascending Node [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::omega>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega_dot",
            /* .docs          = */ "Rate of Right Ascension [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::omega_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::inclination>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination_dot",
            /* .docs          = */ "Inclination angle rate of change [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::inclination_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_ic",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_ic>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_is",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_is>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_uc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_uc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_us",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_us>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_rc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rs",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_rs>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GpsEphemeris::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GpsEphemeris::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GpsEphemeris",
        /* .title       = */ "None",
        /* .docs        = */ "GPS Ephemeris Data",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GalileoEphemeris::ValidFlags>
{
    using type = data_gnss::GalileoEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "ephemeris", "" },
        { 2, "modern_data", "" },
        { 3, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::index>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::satellite_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite and signal health",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::health>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iodc",
            /* .docs          = */ "Issue of Data Clock. This increments each time the data changes and\nrolls over at 4. It is used to make sure various raw data elements from\ndifferent sources line up correctly.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::iodc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "iode",
            /* .docs          = */ "Issue of Data Ephemeris.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::iode>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oc",
            /* .docs          = */ "Reference time for clock data.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::t_oc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af0",
            /* .docs          = */ "Clock bias in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::af0>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af1",
            /* .docs          = */ "Clock drift in [s/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::af1>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "af2",
            /* .docs          = */ "Clock drift rate in [s/s^2].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::af2>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_gd",
            /* .docs          = */ "T Group Delay [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::t_gd>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L1CA",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::ISC_L1CA>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "ISC_L2C",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::ISC_L2C>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "t_oe",
            /* .docs          = */ "Reference time for ephemeris in [s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::t_oe>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Semi-major axis [m].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::a>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a_dot",
            /* .docs          = */ "Semi-major axis rate [m/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::a_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mean_anomaly",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::mean_anomaly>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::delta_mean_motion>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_mean_motion_dot",
            /* .docs          = */ "[rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::delta_mean_motion_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "eccentricity",
            /* .docs          = */ "",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::eccentricity>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "argument_of_perigee",
            /* .docs          = */ "[rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::argument_of_perigee>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega",
            /* .docs          = */ "Longitude of Ascending Node [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::omega>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "omega_dot",
            /* .docs          = */ "Rate of Right Ascension [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::omega_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [rad].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::inclination>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination_dot",
            /* .docs          = */ "Inclination angle rate of change [rad/s].",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::inclination_dot>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_ic",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_ic>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_is",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_is>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_uc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_uc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_us",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_us>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rc",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_rc>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "c_rs",
            /* .docs          = */ "Harmonic Correction Term.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::c_rs>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GalileoEphemeris::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GalileoEphemeris::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_gnss::GalileoEphemeris",
        /* .title       = */ "None",
        /* .docs        = */ "Galileo Ephemeris Data",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GloEphemeris::ValidFlags>
{
    using type = data_gnss::GloEphemeris::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "ephemeris", "" },
        { 1, "flags", "" },
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::index>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "count",
            /* .docs          = */ "Total number of fields in this epoch.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::count>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS Time of week [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "satellite_id",
            /* .docs          = */ "GNSS satellite id within the constellation.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::satellite_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "freq_number",
            /* .docs          = */ "GLONASS frequency number (-7 to 24)",
            /* .type          = */ {Type::S8, nullptr},
            /* .accessor      = */ utils::access<type, int8_t, &type::freq_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tk",
            /* .docs          = */ "Frame start time within current day [seconds]",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ utils::access<type, uint32_t, &type::tk>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tb",
            /* .docs          = */ "Ephemeris reference time [seconds]",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ utils::access<type, uint32_t, &type::tb>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "sat_type",
            /* .docs          = */ "Type of satellite (M) GLONASS = 0, GLONASS-M = 1",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::sat_type>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gamma",
            /* .docs          = */ "Relative deviation of carrier frequency from nominal [dimensionless]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::gamma>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tau_n",
            /* .docs          = */ "Time correction relative to GLONASS Time [seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::tau_n>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "x",
            /* .docs          = */ "Satellite PE-90 position [m]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ utils::access<type, Vector3d, &type::x>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "v",
            /* .docs          = */ "Satellite PE-90 velocity [m/s]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::v>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "a",
            /* .docs          = */ "Satellite PE-90 acceleration due to perturbations [m/s^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::a>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "health",
            /* .docs          = */ "Satellite Health (Bn), Non-zero indicates satellite malfunction",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::health>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P",
            /* .docs          = */ "Satellite operation mode (See GLONASS ICD)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::P>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "NT",
            /* .docs          = */ "Day number within a 4 year period.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::NT>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "delta_tau_n",
            /* .docs          = */ "Time difference between L1 and L2[m/s]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::delta_tau_n>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "Ft",
            /* .docs          = */ "User Range Accuracy (See GLONASS ICD)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::Ft>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "En",
            /* .docs          = */ "Age of current information [days]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::En>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P1",
            /* .docs          = */ "Time interval between adjacent values of tb [minutes]",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::P1>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P2",
            /* .docs          = */ "Oddness '1' or evenness '0' of the value of tb.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::P2>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P3",
            /* .docs          = */ "Number of satellites in almanac for this frame",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::P3>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "P4",
            /* .docs          = */ "Flag indicating ephemeris parameters are present",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::P4>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GloEphemeris::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GloEphemeris::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GpsIonoCorr::ValidFlags>
{
    using type = data_gnss::GpsIonoCorr::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "alpha", "" },
        { 8, "beta", "" },
        { 15, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "alpha",
            /* .docs          = */ "Ionospheric Correction Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::alpha>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Ionospheric Correction Terms.",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::beta>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GpsIonoCorr::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GpsIonoCorr::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_gnss::GalileoIonoCorr::ValidFlags>
{
    using type = data_gnss::GalileoIonoCorr::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "tow", "" },
        { 2, "week_number", "" },
        { 4, "alpha", "" },
        { 8, "disturbance_flags", "" },
        { 15, "flags", "" },
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
            /* .accessor      = */ utils::access<type, double, &type::time_of_week>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "week_number",
            /* .docs          = */ "GPS Week since 1980 [weeks]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "alpha",
            /* .docs          = */ "Coefficients for the model.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ utils::access<type, Vector3d, &type::alpha>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "disturbance_flags",
            /* .docs          = */ "Region disturbance flags (bits 1-5).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::disturbance_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<data_gnss::GalileoIonoCorr::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, data_gnss::GalileoIonoCorr::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};


static constexpr inline std::initializer_list<const FieldInfo*> ALL_DATA_GNSS = {
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
    &MetadataFor<data_gnss::BaseStationInfo>::value,
    &MetadataFor<data_gnss::RtkCorrectionsStatus>::value,
    &MetadataFor<data_gnss::SatelliteStatus>::value,
    &MetadataFor<data_gnss::Raw>::value,
    &MetadataFor<data_gnss::GpsEphemeris>::value,
    &MetadataFor<data_gnss::GalileoEphemeris>::value,
    &MetadataFor<data_gnss::GloEphemeris>::value,
    &MetadataFor<data_gnss::GpsIonoCorr>::value,
    &MetadataFor<data_gnss::GalileoIonoCorr>::value,
};


} // namespace mip::metadata

