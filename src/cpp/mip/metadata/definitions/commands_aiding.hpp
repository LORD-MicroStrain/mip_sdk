#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/commands_aiding.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_aiding::FrameConfig::Format>
{
    using type = commands_aiding::FrameConfig::Format;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "EULER", "Translation vector followed by euler angles (roll, pitch, yaw)." },
        { uint32_t(2), "QUATERNION", "Translation vector followed by quaternion (w, x, y, z)." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Format",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::FrameConfig::Rotation>
{
    using type = commands_aiding::FrameConfig::Rotation;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "euler",
            /* .docs          = */ "Rotation represented as euler angles in RPY format [rad]. Range +/- pi.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::euler>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {ParameterInfo::Condition::Type::ENUM, microstrain::Index(1) /* format */, static_cast<uint16_t>(commands_aiding::FrameConfig::Format::EULER)} /* format == EULER */,
        },
        {
            /* .name          = */ "quaternion",
            /* .docs          = */ "Rotation represented as a quaternion in WXYZ format.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Quatf, &type::quaternion>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {ParameterInfo::Condition::Type::ENUM, microstrain::Index(1) /* format */, static_cast<uint16_t>(commands_aiding::FrameConfig::Format::QUATERNION)} /* format == QUATERNION */,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Rotation",
        /* .title       = */ "Rotation",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_aiding::FrameConfig::Response>
{
    using type = commands_aiding::FrameConfig::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Reference frame number. Limit 4.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "format",
            /* .docs          = */ "Format of the transformation.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::FrameConfig::Format>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::FrameConfig::Format, &type::format>,
            /* .attributes    = */ {true, true, false, false, false, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tracking_enabled",
            /* .docs          = */ "If enabled, the Kalman filter will track errors.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::tracking_enabled>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "translation",
            /* .docs          = */ "Translation X, Y, and Z.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::translation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "rotation",
            /* .docs          = */ "Rotation as specified by format.",
            /* .type          = */ {Type::UNION, &MetadataFor<commands_aiding::FrameConfig::Rotation>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::FrameConfig::Rotation, &type::rotation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::FrameConfig::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::FrameConfig>
{
    using type = commands_aiding::FrameConfig;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Reference frame number. Limit 4.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "format",
            /* .docs          = */ "Format of the transformation.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::FrameConfig::Format>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::FrameConfig::Format, &type::format>,
            /* .attributes    = */ {true, true, false, false, false, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tracking_enabled",
            /* .docs          = */ "If enabled, the Kalman filter will track errors.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::tracking_enabled>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "translation",
            /* .docs          = */ "Translation X, Y, and Z.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::translation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "rotation",
            /* .docs          = */ "Rotation as specified by format.",
            /* .type          = */ {Type::UNION, &MetadataFor<commands_aiding::FrameConfig::Rotation>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::FrameConfig::Rotation, &type::rotation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::FrameConfig",
        /* .title       = */ "Frame Configuration",
        /* .docs        = */ "Defines an aiding frame associated with a specific sensor frame ID.\nThe frame ID used in this command should mirror the frame ID used in the aiding command\n(if that aiding measurement is measured in this reference frame).\n\nThis transform satisfies the following relationship:\n\nEQSTART p^{veh} = R p^{sensor_frame} + t EQEND<br/>\n\nWhere:<br/>\nEQSTART R EQEND is rotation matrix defined by the rotation component and EQSTART t EQEND is the translation vector<br/><br/>\nEQSTART p^{sensor_frame} EQEND is a 3-element position vector expressed in the external sensor frame<br/>\nEQSTART p^{veh} EQEND is a 3-element position vector expressed in the vehicle frame<br/>\n\nRotation can be defined using Euler angles OR quaternions.  If Format selector is set to Euler Angles, the fourth element\nin the rotation vector is ignored and should be set to 0.\n\nWhen the tracking_enabled flag is 1, the Kalman filter will track errors in the provided frame definition; when 0, no errors are tracked.\n\nExample: GNSS antenna lever arm\n\nFrame ID: 1\nFormat: 1 (Euler)\nTranslation: [0,1,] (GNSS with a 1 meter Y offset in the vehicle frame)\nRotation: [0,0,0,0] (Rotational component is not relevant for GNSS measurements, set to zero)",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_aiding::EchoControl::Mode>
{
    using type = commands_aiding::EchoControl::Mode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "SUPPRESS_ACK", "Suppresses the usual command ack field for aiding messages." },
        { uint32_t(1), "STANDARD", "Normal ack/nack behavior." },
        { uint32_t(2), "RESPONSE", "Echo the data back as a response." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Mode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::EchoControl::Response>
{
    using type = commands_aiding::EchoControl::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "mode",
            /* .docs          = */ "Controls data echoing.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::EchoControl::Mode>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::EchoControl::Mode, &type::mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::EchoControl::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::EchoControl>
{
    using type = commands_aiding::EchoControl;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "mode",
            /* .docs          = */ "Controls data echoing.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::EchoControl::Mode>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::EchoControl::Mode, &type::mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::EchoControl",
        /* .title       = */ "Echo Control",
        /* .docs        = */ "Controls command response behavior to external aiding commands",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_aiding::Time::Timebase>
{
    using type = commands_aiding::Time::Timebase;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "INTERNAL_REFERENCE", "Timestamp provided is with respect to internal clock." },
        { uint32_t(2), "EXTERNAL_TIME", "Timestamp provided is with respect to external clock, synced by PPS source." },
        { uint32_t(3), "TIME_OF_ARRIVAL", "Timestamp provided is a fixed latency relative to time of message arrival." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Timebase",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::Time>
{
    using type = commands_aiding::Time;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "timebase",
            /* .docs          = */ "Timebase reference, e.g. internal, external, GPS, UTC, etc.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::Time::Timebase>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time::Timebase, &type::timebase>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "Reserved, set to 0x01.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "nanoseconds",
            /* .docs          = */ "Nanoseconds since the timebase epoch.",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint64_t, &type::nanoseconds>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Time",
        /* .title       = */ "Time",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_aiding::PosEcef::ValidFlags>
{
    using type = commands_aiding::PosEcef::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "X", "" },
        { uint32_t(2), "Y", "" },
        { uint32_t(4), "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::PosEcef>
{
    using type = commands_aiding::PosEcef;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "position",
            /* .docs          = */ "ECEF position [m].",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::position>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "ECEF position uncertainty [m]. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_aiding::PosEcef::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::PosEcef::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::PosEcef",
        /* .title       = */ "ECEF Position",
        /* .docs        = */ "Cartesian vector position aiding command. Coordinates are given in the WGS84 ECEF system.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::PosLlh::ValidFlags>
{
    using type = commands_aiding::PosLlh::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "Latitude", "" },
        { uint32_t(2), "Longitude", "" },
        { uint32_t(4), "Height", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::PosLlh>
{
    using type = commands_aiding::PosLlh;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "latitude",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::latitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "longitude",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::longitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "height",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::height>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "NED position uncertainty. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_aiding::PosLlh::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::PosLlh::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::PosLlh",
        /* .title       = */ "LLH Position",
        /* .docs        = */ "Geodetic position aiding command.\nCoordinates are given in WGS84 geodetic latitude, longitude, and height above the ellipsoid.\nUncertainty is given in NED coordinates, which are parallel to incremental changes in latitude, longitude, and height.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::HeightAboveEllipsoid>
{
    using type = commands_aiding::HeightAboveEllipsoid;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "height",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::height>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::HeightAboveEllipsoid",
        /* .title       = */ "Height Above Ellipsoid",
        /* .docs        = */ "Estimated value of the height above ellipsoid.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::VelEcef::ValidFlags>
{
    using type = commands_aiding::VelEcef::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "X", "" },
        { uint32_t(2), "Y", "" },
        { uint32_t(4), "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::VelEcef>
{
    using type = commands_aiding::VelEcef;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "ECEF velocity [m/s].",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "ECEF velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_aiding::VelEcef::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::VelEcef::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::VelEcef",
        /* .title       = */ "ECEF Velocity",
        /* .docs        = */ "ECEF velocity aiding command. Coordinates are given in the WGS84 ECEF frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::VelNed::ValidFlags>
{
    using type = commands_aiding::VelNed::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "X", "" },
        { uint32_t(2), "Y", "" },
        { uint32_t(4), "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::VelNed>
{
    using type = commands_aiding::VelNed;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "NED velocity [m/s].",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "NED velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_aiding::VelNed::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::VelNed::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::VelNed",
        /* .title       = */ "NED Velocity",
        /* .docs        = */ "NED velocity aiding command. Coordinates are given in the local North East Down frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::VelBodyFrame::ValidFlags>
{
    using type = commands_aiding::VelBodyFrame::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "X", "" },
        { uint32_t(2), "Y", "" },
        { uint32_t(4), "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::VelBodyFrame>
{
    using type = commands_aiding::VelBodyFrame;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "[m/s]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[m/s] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_aiding::VelBodyFrame::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::VelBodyFrame::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::VelBodyFrame",
        /* .title       = */ "Body Frame Velocity",
        /* .docs        = */ "Estimated of velocity of the vehicle in the frame associated with the given sensor ID, relative to the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::HeadingTrue>
{
    using type = commands_aiding::HeadingTrue;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading",
            /* .docs          = */ "Heading [radians]. Range +/- Pi.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "Cannot be 0 unless the valid flags are 0.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::HeadingTrue",
        /* .title       = */ "True Heading",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::MagneticField::ValidFlags>
{
    using type = commands_aiding::MagneticField::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "X", "" },
        { uint32_t(2), "Y", "" },
        { uint32_t(4), "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::MagneticField>
{
    using type = commands_aiding::MagneticField;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "magnetic_field",
            /* .docs          = */ "[G]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::magnetic_field>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[G] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_aiding::MagneticField::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::MagneticField::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::MagneticField",
        /* .title       = */ "Magnetic Field",
        /* .docs        = */ "Estimate of magnetic field in the frame associated with the given sensor ID.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::Pressure>
{
    using type = commands_aiding::Pressure;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_aiding::Time, &type::time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pressure",
            /* .docs          = */ "[mbar]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pressure>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[mbar] 1-sigma uncertainty. Cannot be 0 unless the valid flags are 0.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::Pressure",
        /* .title       = */ "Pressure",
        /* .docs        = */ "Estimated value of air pressure.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* COMMANDS_AIDING_FIELDS[] = {
    &MetadataFor<commands_aiding::FrameConfig>::value,
    &MetadataFor<commands_aiding::EchoControl>::value,
    &MetadataFor<commands_aiding::PosEcef>::value,
    &MetadataFor<commands_aiding::PosLlh>::value,
    &MetadataFor<commands_aiding::HeightAboveEllipsoid>::value,
    &MetadataFor<commands_aiding::VelEcef>::value,
    &MetadataFor<commands_aiding::VelNed>::value,
    &MetadataFor<commands_aiding::VelBodyFrame>::value,
    &MetadataFor<commands_aiding::HeadingTrue>::value,
    &MetadataFor<commands_aiding::MagneticField>::value,
    &MetadataFor<commands_aiding::Pressure>::value,
    &MetadataFor<commands_aiding::FrameConfig::Response>::value,
    &MetadataFor<commands_aiding::EchoControl::Response>::value,
};

static constexpr DescriptorSetInfo COMMANDS_AIDING = {
    /*.descriptor =*/ mip::commands_aiding::DESCRIPTOR_SET,
    /*.name       =*/ "Aiding Commands",
    /*.fields     =*/ COMMANDS_AIDING_FIELDS,
};

} // namespace mip::metadata

