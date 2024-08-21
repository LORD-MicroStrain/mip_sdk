#pragma once

#include "common.hpp"

#include <mip/definitions/commands_aiding.hpp>

#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_aiding::FrameConfig::Format>
{
    using type = commands_aiding::FrameConfig::Format;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 1, "EULER", "Translation vector followed by euler angles (roll, pitch, yaw)." },
        { 2, "QUATERNION", "Translation vector followed by quaternion (w, x, y, z)." },
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
            /* .accessor      = */ utils::access<type, Vector3f, &type::euler>,
            /* .functions     = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {ParameterInfo::Condition::Type::ENUM, microstrain::Index(1) /* format */, static_cast<uint16_t>(commands_aiding::FrameConfig::Format::EULER)} /* format == EULER */,
        },
        {
            /* .name          = */ "quaternion",
            /* .docs          = */ "Rotation represented as a quaternion in WXYZ format.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ utils::access<type, Quatf, &type::quaternion>,
            /* .functions     = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {ParameterInfo::Condition::Type::ENUM, microstrain::Index(1) /* format */, static_cast<uint16_t>(commands_aiding::FrameConfig::Format::QUATERNION)} /* format == QUATERNION */,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Rotation",
        /* .title       = */ "None",
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, true, true, true, true,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "format",
            /* .docs          = */ "Format of the transformation.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::FrameConfig::Format>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::FrameConfig::Format, &type::format>,
            /* .functions     = */ {true, true, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tracking_enabled",
            /* .docs          = */ "If enabled, the Kalman filter will track errors.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ utils::access<type, bool, &type::tracking_enabled>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "translation",
            /* .docs          = */ "Translation X, Y, and Z.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::translation>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "rotation",
            /* .docs          = */ "Rotation as specified by format.",
            /* .type          = */ {Type::UNION, &MetadataFor<commands_aiding::FrameConfig::Rotation>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::FrameConfig::Rotation, &type::rotation>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::FrameConfig::Response",
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
struct MetadataFor<commands_aiding::FrameConfig>
{
    using type = commands_aiding::FrameConfig;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Reference frame number. Limit 4.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, true, true, true, true,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "format",
            /* .docs          = */ "Format of the transformation.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::FrameConfig::Format>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::FrameConfig::Format, &type::format>,
            /* .functions     = */ {true, true, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "tracking_enabled",
            /* .docs          = */ "If enabled, the Kalman filter will track errors.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ utils::access<type, bool, &type::tracking_enabled>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "translation",
            /* .docs          = */ "Translation X, Y, and Z.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::translation>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "rotation",
            /* .docs          = */ "Rotation as specified by format.",
            /* .type          = */ {Type::UNION, &MetadataFor<commands_aiding::FrameConfig::Rotation>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::FrameConfig::Rotation, &type::rotation>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::FrameConfig",
        /* .title       = */ "Frame Configuration",
        /* .docs        = */ "Defines an aiding frame associated with a specific sensor frame ID.  The frame ID used in this command\nshould mirror the frame ID used in the aiding command (if that aiding measurement is measured in this reference frame)\n\nThis transform satisfies the following relationship:\n\nEQSTART p^{veh} = R p^{sensor_frame} + t EQEND<br/>\n\nWhere:<br/>\nEQSTART R EQEND is rotation matrix defined by the rotation component and EQSTART t EQEND is the translation vector<br/><br/>\nEQSTART p^{sensor_frame} EQEND is a 3-element position vector expressed in the external sensor frame<br/>\nEQSTART p^{veh} EQEND is a 3-element position vector expressed in the vehicle frame<br/>\n\nRotation can be defined using Euler angles OR quaternions.  If Format selector is set to Euler Angles, the fourth element\nin the rotation vector is ignored and should be set to 0.\n\nWhen the tracking_enabled flag is 1, the Kalman filter will track errors in the provided frame definition; when 0, no errors are tracked.\n\nExample: GNSS antenna lever arm\n\nFrame ID: 1\nFormat: 1 (Euler)\nTranslation: [0,1,] (GNSS with a 1 meter Y offset in the vehicle frame)\nRotation: [0,0,0,0] (Rotational component is not relevant for GNSS measurements, set to zero)\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true,  true},
        /* .proprietary = */ false,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_aiding::AidingEchoControl::Mode>
{
    using type = commands_aiding::AidingEchoControl::Mode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "SUPPRESS_ACK", "Suppresses the usual command ack field for aiding messages." },
        { 1, "STANDARD", "Normal ack/nack behavior." },
        { 2, "RESPONSE", "Echo the data back as a response." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Mode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::AidingEchoControl::Response>
{
    using type = commands_aiding::AidingEchoControl::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "mode",
            /* .docs          = */ "Controls data echoing.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::AidingEchoControl::Mode>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::AidingEchoControl::Mode, &type::mode>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::AidingEchoControl::Response",
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
struct MetadataFor<commands_aiding::AidingEchoControl>
{
    using type = commands_aiding::AidingEchoControl;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "mode",
            /* .docs          = */ "Controls data echoing.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_aiding::AidingEchoControl::Mode>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::AidingEchoControl::Mode, &type::mode>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::AidingEchoControl",
        /* .title       = */ "Aiding Command Echo Control",
        /* .docs        = */ "Controls command response behavior to external aiding commands",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true,  true},
        /* .proprietary = */ false,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_aiding::Time::Timebase>
{
    using type = commands_aiding::Time::Timebase;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 1, "INTERNAL_REFERENCE", "Timestamp provided is with respect to internal clock." },
        { 2, "EXTERNAL_TIME", "Timestamp provided is with respect to external clock, synced by PPS source." },
        { 3, "TIME_OF_ARRIVAL", "Timestamp provided is a fixed latency relative to time of message arrival." },
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
            /* .accessor      = */ utils::access<type, commands_aiding::Time::Timebase, &type::timebase>,
            /* .functions     = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "Reserved, set to 0x01.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions     = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "nanoseconds",
            /* .docs          = */ "Nanoseconds since the timebase epoch.",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ utils::access<type, uint64_t, &type::nanoseconds>,
            /* .functions     = */ NO_FUNCTIONS,
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
struct MetadataFor<commands_aiding::EcefPos::ValidFlags>
{
    using type = commands_aiding::EcefPos::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "X", "" },
        { 2, "Y", "" },
        { 4, "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::EcefPos>
{
    using type = commands_aiding::EcefPos;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "position",
            /* .docs          = */ "ECEF position [m].",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ utils::access<type, Vector3d, &type::position>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "ECEF position uncertainty [m]. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<commands_aiding::EcefPos::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::EcefPos::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::EcefPos",
        /* .title       = */ "ECEF Position",
        /* .docs        = */ "Cartesian vector position aiding command. Coordinates are given in the WGS84 ECEF system.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::LlhPos::ValidFlags>
{
    using type = commands_aiding::LlhPos::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "Latitude", "" },
        { 2, "Longitude", "" },
        { 4, "Height", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::LlhPos>
{
    using type = commands_aiding::LlhPos;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "latitude",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::latitude>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "longitude",
            /* .docs          = */ "[deg]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::longitude>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "height",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ utils::access<type, double, &type::height>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "NED position uncertainty. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<commands_aiding::LlhPos::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::LlhPos::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::LlhPos",
        /* .title       = */ "LLH Position",
        /* .docs        = */ "Geodetic position aiding command. Coordinates are given in WGS84 geodetic latitude, longitude, and height above the ellipsoid.\nUncertainty is given in NED coordinates, which are parallel to incremental changes in latitude, longitude, and height.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::Height>
{
    using type = commands_aiding::Height;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "height",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::height>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::Height",
        /* .title       = */ "Height",
        /* .docs        = */ "Estimated value of height.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::EcefVel::ValidFlags>
{
    using type = commands_aiding::EcefVel::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "X", "" },
        { 2, "Y", "" },
        { 4, "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::EcefVel>
{
    using type = commands_aiding::EcefVel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "ECEF velocity [m/s].",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::velocity>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "ECEF velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<commands_aiding::EcefVel::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::EcefVel::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::EcefVel",
        /* .title       = */ "ECEF Velocity",
        /* .docs        = */ "ECEF velocity aiding command. Coordinates are given in the WGS84 ECEF frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::NedVel::ValidFlags>
{
    using type = commands_aiding::NedVel::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "X", "" },
        { 2, "Y", "" },
        { 4, "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::NedVel>
{
    using type = commands_aiding::NedVel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "NED velocity [m/s].",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::velocity>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "NED velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<commands_aiding::NedVel::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::NedVel::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::NedVel",
        /* .title       = */ "NED Velocity",
        /* .docs        = */ "NED velocity aiding command. Coordinates are given in the local North-East-Down frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::VehicleFixedFrameVelocity::ValidFlags>
{
    using type = commands_aiding::VehicleFixedFrameVelocity::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "X", "" },
        { 2, "Y", "" },
        { 4, "Z", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_aiding::VehicleFixedFrameVelocity>
{
    using type = commands_aiding::VehicleFixedFrameVelocity;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "[m/s]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::velocity>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[m/s] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<commands_aiding::VehicleFixedFrameVelocity::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::VehicleFixedFrameVelocity::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::VehicleFixedFrameVelocity",
        /* .title       = */ "Vehicle Frame Velocity",
        /* .docs        = */ "Estimate of velocity of the vehicle in the frame associated\nwith the given sensor ID.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::TrueHeading>
{
    using type = commands_aiding::TrueHeading;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time",
            /* .docs          = */ "Timestamp of the measurement.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_aiding::Time>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading",
            /* .docs          = */ "Heading [radians]. Range +/- Pi.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::heading>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "Cannot be 0 unless the valid flags are 0.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_aiding::TrueHeading",
        /* .title       = */ "True Heading",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_aiding::MagneticField::ValidFlags>
{
    using type = commands_aiding::MagneticField::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "X", "" },
        { 2, "Y", "" },
        { 4, "Z", "" },
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
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "magnetic_field",
            /* .docs          = */ "[G]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::magnetic_field>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[G] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<type, Vector3f, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "Valid flags. Axes with 0 will be completely ignored.",
            /* .type          = */ {Type::BITFIELD, &MetadataFor<commands_aiding::MagneticField::ValidFlags>::value},
            /* .accessor      = */ utils::access<type, commands_aiding::MagneticField::ValidFlags, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
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
            /* .accessor      = */ utils::access<type, commands_aiding::Time, &type::time>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::frame_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pressure",
            /* .docs          = */ "[mbar]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::pressure>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "[mbar] 1-sigma uncertainty. Cannot be 0 unless the valid flags are 0.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::uncertainty>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ utils::access<type, uint16_t, &type::valid_flags>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};


static constexpr inline std::initializer_list<const FieldInfo*> ALL_COMMANDS_AIDING = {
    &MetadataFor<commands_aiding::FrameConfig>::value,
    &MetadataFor<commands_aiding::FrameConfig::Response>::value,
    &MetadataFor<commands_aiding::AidingEchoControl>::value,
    &MetadataFor<commands_aiding::AidingEchoControl::Response>::value,
    &MetadataFor<commands_aiding::EcefPos>::value,
    &MetadataFor<commands_aiding::LlhPos>::value,
    &MetadataFor<commands_aiding::Height>::value,
    &MetadataFor<commands_aiding::EcefVel>::value,
    &MetadataFor<commands_aiding::NedVel>::value,
    &MetadataFor<commands_aiding::VehicleFixedFrameVelocity>::value,
    &MetadataFor<commands_aiding::TrueHeading>::value,
    &MetadataFor<commands_aiding::MagneticField>::value,
    &MetadataFor<commands_aiding::Pressure>::value,
};


} // namespace mip::metadata

