#pragma once

#include "mip/metadata/common.hpp"
#include "mip/metadata/mip_metadata.hpp"

#include <mip/definitions/commands_filter.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_filter::Reset>
{
    using type = commands_filter::Reset;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::Reset",
            /* .title       = */ "Reset Navigation Filter",
            /* .docs        = */ "Resets the filter to the initialization state.\n\nIf the auto-initialization feature is disabled, the initial attitude or heading must be set in\norder to enter the run state after a reset.",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::Reset>::value > { using type = commands_filter::Reset; };
template<> struct TypeForDescriptor<commands_filter::Reset::DESCRIPTOR.as_u16()> { using type = commands_filter::Reset; };

template<>
struct MetadataFor<commands_filter::SetInitialAttitude>
{
    using type = commands_filter::SetInitialAttitude;

    using ParamTypes = std::tuple<
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.roll;
        if constexpr(I == 1) return value_.pitch;
        if constexpr(I == 2) return value_.heading;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "roll",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::roll>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pitch",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pitch>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SetInitialAttitude",
            /* .title       = */ "Set Initial Attitude",
            /* .docs        = */ "Set the sensor initial attitude.\n\nThis command can only be issued in the 'Init' state and should be used with a good\nestimate of the vehicle attitude.  The Euler angles are the sensor body frame with respect\nto the NED frame.\n\nThe valid input ranges are as follows:\n\nRoll:    [-pi, pi]\nPitch:   [-pi/2, pi/2]\nHeading: [-pi, pi]\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SetInitialAttitude>::value > { using type = commands_filter::SetInitialAttitude; };
template<> struct TypeForDescriptor<commands_filter::SetInitialAttitude::DESCRIPTOR.as_u16()> { using type = commands_filter::SetInitialAttitude; };

template<>
struct MetadataFor<commands_filter::EstimationControl::EnableFlags>
{
    using type = commands_filter::EstimationControl::EnableFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "gyro_bias", "" },
        { uint32_t(2), "accel_bias", "" },
        { uint32_t(4), "gyro_scale_factor", "" },
        { uint32_t(8), "accel_scale_factor", "" },
        { uint32_t(16), "antenna_offset", "" },
        { uint32_t(32), "auto_mag_hard_iron", "" },
        { uint32_t(64), "auto_mag_soft_iron", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "EnableFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<> struct TypeForBitsInfo< &MetadataFor<commands_filter::EstimationControl::EnableFlags>::value > { using type = commands_filter::EstimationControl::EnableFlags; };

template<>
struct MetadataFor<commands_filter::EstimationControl::Response>
{
    using type = commands_filter::EstimationControl::Response;

    using ParamTypes = std::tuple<
        commands_filter::EstimationControl::EnableFlags
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "See above",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_filter::EstimationControl::EnableFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::EstimationControl::EnableFlags, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::EstimationControl::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::EstimationControl::Response>::value > { using type = commands_filter::EstimationControl::Response; };

template<>
struct MetadataFor<commands_filter::EstimationControl>
{
    using type = commands_filter::EstimationControl;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::EstimationControl::EnableFlags
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "See above",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_filter::EstimationControl::EnableFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::EstimationControl::EnableFlags, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::EstimationControl",
            /* .title       = */ "Estimation Control Flags",
            /* .docs        = */ "Estimation Control Flags\n\nControls which parameters are estimated by the Kalman Filter.\n\nDesired settings should be logically ORed together.\n\nExamples:\n\n0x0001 - Enable Gyro Bias Estimation Only\n0x0063 - Enable Gyro Bias, Accel Bias, and Mag Auto Hard and Soft Iron Cal States Only\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::EstimationControl>::value > { using type = commands_filter::EstimationControl; };
template<> struct TypeForDescriptor<commands_filter::EstimationControl::DESCRIPTOR.as_u16()> { using type = commands_filter::EstimationControl; };

template<>
struct MetadataFor<commands_filter::ExternalGnssUpdate>
{
    using type = commands_filter::ExternalGnssUpdate;

    using ParamTypes = std::tuple<
        double,
        uint16_t,
        double,
        double,
        double,
        Vector3f,
        Vector3f,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.gps_time;
        if constexpr(I == 1) return value_.gps_week;
        if constexpr(I == 2) return value_.latitude;
        if constexpr(I == 3) return value_.longitude;
        if constexpr(I == 4) return value_.height;
        if constexpr(I == 5) return value_.velocity;
        if constexpr(I == 6) return value_.pos_uncertainty;
        if constexpr(I == 7) return value_.vel_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gps_time",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::gps_time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gps_week",
            /* .docs          = */ "[GPS week number, not modulus 1024]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::gps_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
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
            /* .name          = */ "height",
            /* .docs          = */ "Above WGS84 ellipsoid [meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::height>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity",
            /* .docs          = */ "NED Frame [meters/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pos_uncertainty",
            /* .docs          = */ "NED Frame, 1-sigma [meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::pos_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "vel_uncertainty",
            /* .docs          = */ "NED Frame, 1-sigma [meters/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::vel_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::ExternalGnssUpdate",
            /* .title       = */ "External GNSS Update",
            /* .docs        = */ "Provide a filter measurement from an external GNSS\n\nThe GNSS source control must be set to 'external' for this command to succeed, otherwise it will be NACK'd.\nPlease refer to your device user manual for information on the maximum rate of this message.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::ExternalGnssUpdate>::value > { using type = commands_filter::ExternalGnssUpdate; };
template<> struct TypeForDescriptor<commands_filter::ExternalGnssUpdate::DESCRIPTOR.as_u16()> { using type = commands_filter::ExternalGnssUpdate; };

template<>
struct MetadataFor<commands_filter::ExternalHeadingUpdate>
{
    using type = commands_filter::ExternalHeadingUpdate;

    using ParamTypes = std::tuple<
        float,
        float,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.heading;
        if constexpr(I == 1) return value_.heading_uncertainty;
        if constexpr(I == 2) return value_.type;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "heading",
            /* .docs          = */ "Bounded by +-PI [radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading_uncertainty",
            /* .docs          = */ "1-sigma [radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "type",
            /* .docs          = */ "1 - True, 2 - Magnetic",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::ExternalHeadingUpdate",
            /* .title       = */ "External Heading Update",
            /* .docs        = */ "Provide a filter measurement from an external heading source\n\nThe heading must be the sensor frame with respect to the NED frame.\n\nThe heading update control must be set to external for this command to update the filter; otherwise it is NACK'd.\nHeading angle uncertainties of &lt;= 0.0 will be NACK'd\n\nPlease refer to your device user manual for information on the maximum rate of this message.\n\nOn -25 models, if the declination source (0x0D, 0x43) is not valid, true heading updates will be NACK'd.\nOn -45 models, if the declination source is invalid, magnetic heading updates will be NACK'd.\n\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::ExternalHeadingUpdate>::value > { using type = commands_filter::ExternalHeadingUpdate; };
template<> struct TypeForDescriptor<commands_filter::ExternalHeadingUpdate::DESCRIPTOR.as_u16()> { using type = commands_filter::ExternalHeadingUpdate; };

template<>
struct MetadataFor<commands_filter::ExternalHeadingUpdateWithTime>
{
    using type = commands_filter::ExternalHeadingUpdateWithTime;

    using ParamTypes = std::tuple<
        double,
        uint16_t,
        float,
        float,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.gps_time;
        if constexpr(I == 1) return value_.gps_week;
        if constexpr(I == 2) return value_.heading;
        if constexpr(I == 3) return value_.heading_uncertainty;
        if constexpr(I == 4) return value_.type;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gps_time",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::gps_time>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "gps_week",
            /* .docs          = */ "[GPS week number, not modulus 1024]",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::gps_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading",
            /* .docs          = */ "Relative to true north, bounded by +-PI [radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading_uncertainty",
            /* .docs          = */ "1-sigma [radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "type",
            /* .docs          = */ "1 - True, 2 - Magnetic",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::ExternalHeadingUpdateWithTime",
            /* .title       = */ "External Heading Update With Time",
            /* .docs        = */ "Provide a filter measurement from an external heading source at a specific GPS time\n\nThis is more accurate than the External Heading Update (0x0D, 0x17) and should be used in applications\nwhere the rate of heading change will cause significant measurement error due to the sampling, transmission,\nand processing time required.  Accurate time stamping of the heading information is important.\n\nThe heading must be the sensor frame with respect to the NED frame.\n\nThe heading update control must be set to external for this command to update the filter; otherwise it is NACK'd.\nHeading angle uncertainties of &lt;= 0.0 will be NACK'd\n\nPlease refer to your device user manual for information on the maximum rate of this message.\n\nOn -25 models, if the declination source (0x0D, 0x43) is not valid, true heading updates will be NACK'd.\nOn -45 models, if the declination source is invalid, magnetic heading updates will be NACK'd.\n\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::ExternalHeadingUpdateWithTime>::value > { using type = commands_filter::ExternalHeadingUpdateWithTime; };
template<> struct TypeForDescriptor<commands_filter::ExternalHeadingUpdateWithTime::DESCRIPTOR.as_u16()> { using type = commands_filter::ExternalHeadingUpdateWithTime; };

template<>
struct MetadataFor<commands_filter::TareOrientation::MipTareAxes>
{
    using type = commands_filter::TareOrientation::MipTareAxes;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "roll", "" },
        { uint32_t(2), "pitch", "" },
        { uint32_t(4), "yaw", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "MipTareAxes",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForBitsInfo< &MetadataFor<commands_filter::TareOrientation::MipTareAxes>::value > { using type = commands_filter::TareOrientation::MipTareAxes; };

template<>
struct MetadataFor<commands_filter::TareOrientation::Response>
{
    using type = commands_filter::TareOrientation::Response;

    using ParamTypes = std::tuple<
        commands_filter::TareOrientation::MipTareAxes
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.axes;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "axes",
            /* .docs          = */ "Axes to tare",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_filter::TareOrientation::MipTareAxes>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::TareOrientation::MipTareAxes, &type::axes>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::TareOrientation::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::TareOrientation::Response>::value > { using type = commands_filter::TareOrientation::Response; };

template<>
struct MetadataFor<commands_filter::TareOrientation>
{
    using type = commands_filter::TareOrientation;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::TareOrientation::MipTareAxes
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.axes;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "axes",
            /* .docs          = */ "Axes to tare",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_filter::TareOrientation::MipTareAxes>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::TareOrientation::MipTareAxes, &type::axes>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::TareOrientation",
            /* .title       = */ "Tare Sensor Orientation",
            /* .docs        = */ "Tare the device orientation.\n\nThis function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.\nThis command is provided as a convenient way to set the sensor to vehicle frame transformation.\nThe filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::TareOrientation>::value > { using type = commands_filter::TareOrientation; };
template<> struct TypeForDescriptor<commands_filter::TareOrientation::DESCRIPTOR.as_u16()> { using type = commands_filter::TareOrientation; };

template<>
struct MetadataFor<commands_filter::VehicleDynamicsMode::DynamicsMode>
{
    using type = commands_filter::VehicleDynamicsMode::DynamicsMode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "PORTABLE", "" },
        { uint32_t(2), "AUTOMOTIVE", "" },
        { uint32_t(3), "AIRBORNE", "" },
        { uint32_t(4), "AIRBORNE_HIGH_G", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "DynamicsMode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::VehicleDynamicsMode::DynamicsMode>::value > { using type = commands_filter::VehicleDynamicsMode::DynamicsMode; };

template<>
struct MetadataFor<commands_filter::VehicleDynamicsMode::Response>
{
    using type = commands_filter::VehicleDynamicsMode::Response;

    using ParamTypes = std::tuple<
        commands_filter::VehicleDynamicsMode::DynamicsMode
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.mode;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "mode",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::VehicleDynamicsMode::DynamicsMode>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::VehicleDynamicsMode::DynamicsMode, &type::mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::VehicleDynamicsMode::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::VehicleDynamicsMode::Response>::value > { using type = commands_filter::VehicleDynamicsMode::Response; };

template<>
struct MetadataFor<commands_filter::VehicleDynamicsMode>
{
    using type = commands_filter::VehicleDynamicsMode;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::VehicleDynamicsMode::DynamicsMode
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.mode;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "mode",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::VehicleDynamicsMode::DynamicsMode>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::VehicleDynamicsMode::DynamicsMode, &type::mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::VehicleDynamicsMode",
            /* .title       = */ "Vehicle Dynamics Mode",
            /* .docs        = */ "Controls the vehicle dynamics mode.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::VehicleDynamicsMode>::value > { using type = commands_filter::VehicleDynamicsMode; };
template<> struct TypeForDescriptor<commands_filter::VehicleDynamicsMode::DESCRIPTOR.as_u16()> { using type = commands_filter::VehicleDynamicsMode; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleRotationEuler::Response>
{
    using type = commands_filter::SensorToVehicleRotationEuler::Response;

    using ParamTypes = std::tuple<
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.roll;
        if constexpr(I == 1) return value_.pitch;
        if constexpr(I == 2) return value_.yaw;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "roll",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::roll>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pitch",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pitch>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "yaw",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::yaw>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleRotationEuler::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleRotationEuler::Response>::value > { using type = commands_filter::SensorToVehicleRotationEuler::Response; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleRotationEuler>
{
    using type = commands_filter::SensorToVehicleRotationEuler;

    using ParamTypes = std::tuple<
        FunctionSelector,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.roll;
        if constexpr(I == 2) return value_.pitch;
        if constexpr(I == 3) return value_.yaw;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "roll",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::roll>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "pitch",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pitch>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "yaw",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::yaw>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleRotationEuler",
            /* .title       = */ "Sensor to Vehicle Frame Rotation Euler",
            /* .docs        = */ "Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.\n\nNote: This is the rotation, the inverse of the transformation.\nThese angles define the rotation from the sensor body frame to the fixed vehicle frame.<br/>\nPlease reference the device Theory of Operation for more information.<br/>\nThe rotation is stored in the device as a quaternion.  When Euler angles are read back from the device, they may not\nbe equivalent in value to the Euler angles used to set the rotation, but they are functionally equivalent.<br/>\n<br/><br/>\nThis rotation affects the following output quantities:<br/><br/>\nIMU:<br/>\nScaled Acceleration<br/>\nScaled Gyro<br/>\nScaled Magnetometer<br/>\nDelta Theta<br/>\nDelta Velocity<br/>\n<br/><br/>\nEstimation Filter:<br/>\nEstimated Orientation, Quaternion<br/>\nEstimated Orientation, Matrix<br/>\nEstimated Orientation, Euler Angles<br/>\nEstimated Linear Acceleration<br/>\nEstimated Angular Rate<br/>\nEstimated Gravity Vector<br/>",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleRotationEuler>::value > { using type = commands_filter::SensorToVehicleRotationEuler; };
template<> struct TypeForDescriptor<commands_filter::SensorToVehicleRotationEuler::DESCRIPTOR.as_u16()> { using type = commands_filter::SensorToVehicleRotationEuler; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleRotationDcm::Response>
{
    using type = commands_filter::SensorToVehicleRotationDcm::Response;

    using ParamTypes = std::tuple<
        Matrix3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.dcm;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "dcm",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::dcm>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleRotationDcm::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleRotationDcm::Response>::value > { using type = commands_filter::SensorToVehicleRotationDcm::Response; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleRotationDcm>
{
    using type = commands_filter::SensorToVehicleRotationDcm;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Matrix3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.dcm;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "dcm",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::dcm>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleRotationDcm",
            /* .title       = */ "Sensor to Vehicle Frame Rotation DCM",
            /* .docs        = */ "Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.\n\nNote: This is the rotation, the inverse of the transformation.\nThis matrix defines the rotation from the sensor body frame to the fixed vehicle frame.<br/>\nPlease reference the device Theory of Operation for more information.<br/>\nThe matrix must be orthonormal (tolerance 1e-3) or the device will NACK the command.\nThe rotation is stored in the device as a quaternion.  When the DCM is read back from the device, the components may not\nbe exactly equivalent in value to the DCM used to set the rotation, but they are functionally equivalent.<br/>\n<br/>\nMatrix element order:<br/><br/>\n\nEQSTART T_{SEN}^{VEH} = \\begin{bmatrix} 0 &amp; 1 &amp; 2\\\\  3 &amp; 4 &amp; 5\\\\ 6 &amp; 7 &amp; 8 \\end{bmatrix} EQEND\n\n<br/><br/>\nThis rotation affects the following output quantities:<br/><br/>\nIMU:<br/>\nScaled Acceleration<br/>\nScaled Gyro<br/>\nScaled Magnetometer<br/>\nDelta Theta<br/>\nDelta Velocity<br/>\n<br/><br/>\nEstimation Filter:<br/>\nEstimated Orientation, Quaternion<br/>\nEstimated Orientation, Matrix<br/>\nEstimated Orientation, Euler Angles<br/>\nEstimated Linear Acceleration<br/>\nEstimated Angular Rate<br/>\nEstimated Gravity Vector<br/>",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleRotationDcm>::value > { using type = commands_filter::SensorToVehicleRotationDcm; };
template<> struct TypeForDescriptor<commands_filter::SensorToVehicleRotationDcm::DESCRIPTOR.as_u16()> { using type = commands_filter::SensorToVehicleRotationDcm; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleRotationQuaternion::Response>
{
    using type = commands_filter::SensorToVehicleRotationQuaternion::Response;

    using ParamTypes = std::tuple<
        Quatf
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.quat;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "quat",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Quatf, &type::quat>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleRotationQuaternion::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleRotationQuaternion::Response>::value > { using type = commands_filter::SensorToVehicleRotationQuaternion::Response; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleRotationQuaternion>
{
    using type = commands_filter::SensorToVehicleRotationQuaternion;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Quatf
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.quat;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "quat",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Quatf, &type::quat>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleRotationQuaternion",
            /* .title       = */ "Sensor to Vehicle Frame Rotation Quaternion",
            /* .docs        = */ "Set the sensor to vehicle frame rotation using a quaternion.\n\nNote: This is the rotation, the inverse of the transformation.\nThis quaternion defines the rotation from the sensor body frame to the fixed vehicle frame.<br/>\nPlease reference the device Theory of Operation for more information.<br/>\nThe quaternion must be unit length (tolerance 1e-3) or the device will NACK the command.\nThe rotation is stored in the device as a unit quaternion.  When the quaternion elements are read back from the device, they may not\nbe equivalent in value to the quaternion used to set the rotation, due to normalization.<br/>\n<br/>\nQuaternion element definition:<br/><br/>\n<br/>\nEQSTART Q_{SEN}^{VEH} = \\begin{bmatrix} q_{0} &amp; q_{1}*i  &amp; q_{2}*j  &amp; q_{3}*k \\end{bmatrix} EQEND\n<br/><br/>\nThis rotation affects the following output quantities:<br/><br/>\nIMU:<br/>\nScaled Acceleration<br/>\nScaled Gyro<br/>\nScaled Magnetometer<br/>\nDelta Theta<br/>\nDelta Velocity<br/>\n<br/><br/>\nEstimation Filter:<br/>\nEstimated Orientation, Quaternion<br/>\nEstimated Orientation, Matrix<br/>\nEstimated Orientation, Euler Angles<br/>\nEstimated Linear Acceleration<br/>\nEstimated Angular Rate<br/>\nEstimated Gravity Vector<br/>",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleRotationQuaternion>::value > { using type = commands_filter::SensorToVehicleRotationQuaternion; };
template<> struct TypeForDescriptor<commands_filter::SensorToVehicleRotationQuaternion::DESCRIPTOR.as_u16()> { using type = commands_filter::SensorToVehicleRotationQuaternion; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleOffset::Response>
{
    using type = commands_filter::SensorToVehicleOffset::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "offset",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleOffset::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleOffset::Response>::value > { using type = commands_filter::SensorToVehicleOffset::Response; };

template<>
struct MetadataFor<commands_filter::SensorToVehicleOffset>
{
    using type = commands_filter::SensorToVehicleOffset;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "offset",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SensorToVehicleOffset",
            /* .title       = */ "Sensor to Vehicle Frame Offset",
            /* .docs        = */ "Set the sensor to vehicle frame offset, expressed in the sensor frame.\n\nThis is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.\nIt simply adds the offset to the position output to express it in the origin of the user's vehicle frame.\n\nThis offset affects the following output quantities:\nEstimated LLH Position\n\nThe magnitude of the offset vector is limited to 10 meters",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SensorToVehicleOffset>::value > { using type = commands_filter::SensorToVehicleOffset; };
template<> struct TypeForDescriptor<commands_filter::SensorToVehicleOffset::DESCRIPTOR.as_u16()> { using type = commands_filter::SensorToVehicleOffset; };

template<>
struct MetadataFor<commands_filter::AntennaOffset::Response>
{
    using type = commands_filter::AntennaOffset::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "offset",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AntennaOffset::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AntennaOffset::Response>::value > { using type = commands_filter::AntennaOffset::Response; };

template<>
struct MetadataFor<commands_filter::AntennaOffset>
{
    using type = commands_filter::AntennaOffset;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "offset",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AntennaOffset",
            /* .title       = */ "GNSS Antenna Offset Control",
            /* .docs        = */ "Configure the GNSS antenna offset.\n\nFor 5-series products, this is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.\n\nFor 7-series products, this is expressed in the vehicle frame, from the sensor origin to the GNSS antenna RF center.\n\nThis command should also be used for CV7 / GV7-INS NMEA Input over GPIO.\n\nThe magnitude of the offset vector is limited to 10 meters\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AntennaOffset>::value > { using type = commands_filter::AntennaOffset; };
template<> struct TypeForDescriptor<commands_filter::AntennaOffset::DESCRIPTOR.as_u16()> { using type = commands_filter::AntennaOffset; };

template<>
struct MetadataFor<commands_filter::GnssSource::Source>
{
    using type = commands_filter::GnssSource::Source;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "ALL_INT", "All internal receivers" },
        { uint32_t(2), "EXT", "External GNSS messages provided by user" },
        { uint32_t(3), "INT_1", "Internal GNSS Receiver 1 only" },
        { uint32_t(4), "INT_2", "Internal GNSS Receiver 2 only" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Source",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::GnssSource::Source>::value > { using type = commands_filter::GnssSource::Source; };

template<>
struct MetadataFor<commands_filter::GnssSource::Response>
{
    using type = commands_filter::GnssSource::Response;

    using ParamTypes = std::tuple<
        commands_filter::GnssSource::Source
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::GnssSource::Source>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::GnssSource::Source, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GnssSource::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GnssSource::Response>::value > { using type = commands_filter::GnssSource::Response; };

template<>
struct MetadataFor<commands_filter::GnssSource>
{
    using type = commands_filter::GnssSource;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::GnssSource::Source
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::GnssSource::Source>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::GnssSource::Source, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GnssSource",
            /* .title       = */ "GNSS Aiding Source Control",
            /* .docs        = */ "Control the source of GNSS information used to update the Kalman Filter.\n\nChanging the GNSS source while the sensor is in the 'running' state may temporarily place\nit back in the 'init' state until the new source of GNSS data is received.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GnssSource>::value > { using type = commands_filter::GnssSource; };
template<> struct TypeForDescriptor<commands_filter::GnssSource::DESCRIPTOR.as_u16()> { using type = commands_filter::GnssSource; };

template<>
struct MetadataFor<commands_filter::HeadingSource::Source>
{
    using type = commands_filter::HeadingSource::Source;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "NONE", "See note 3" },
        { uint32_t(1), "MAG", "" },
        { uint32_t(2), "GNSS_VEL", "See notes 1,2" },
        { uint32_t(3), "EXTERNAL", "" },
        { uint32_t(4), "GNSS_VEL_AND_MAG", "" },
        { uint32_t(5), "GNSS_VEL_AND_EXTERNAL", "" },
        { uint32_t(6), "MAG_AND_EXTERNAL", "" },
        { uint32_t(7), "GNSS_VEL_AND_MAG_AND_EXTERNAL", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Source",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::HeadingSource::Source>::value > { using type = commands_filter::HeadingSource::Source; };

template<>
struct MetadataFor<commands_filter::HeadingSource::Response>
{
    using type = commands_filter::HeadingSource::Response;

    using ParamTypes = std::tuple<
        commands_filter::HeadingSource::Source
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::HeadingSource::Source>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::HeadingSource::Source, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::HeadingSource::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::HeadingSource::Response>::value > { using type = commands_filter::HeadingSource::Response; };

template<>
struct MetadataFor<commands_filter::HeadingSource>
{
    using type = commands_filter::HeadingSource;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::HeadingSource::Source
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::HeadingSource::Source>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::HeadingSource::Source, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::HeadingSource",
            /* .title       = */ "Heading Aiding Source Control",
            /* .docs        = */ "Control the source of heading information used to update the Kalman Filter.\n\n1. To use internal GNSS velocity vector for heading updates, the target application\nmust have minimal (preferably no) side-slip.  This option is good for wheeled vehicles.\n\n2. On some devices, when using GNSS velocity vector for heading updates, the X-axis of the device\nmust align with the direction of travel.  Please reference the user guide for your particular device to\ndetermine if this limitation is applicable.\n\n3. When none is selected, the heading estimate can still converge if GNSS is available and sufficient dynamic motion\n(change in direction of travel and acceleration) is experienced.  The heading may drift when: stationary, traveling\nat a constant speed, or during a constant course over ground.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::HeadingSource>::value > { using type = commands_filter::HeadingSource; };
template<> struct TypeForDescriptor<commands_filter::HeadingSource::DESCRIPTOR.as_u16()> { using type = commands_filter::HeadingSource; };

template<>
struct MetadataFor<commands_filter::AutoInitControl::Response>
{
    using type = commands_filter::AutoInitControl::Response;

    using ParamTypes = std::tuple<
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "See above",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AutoInitControl::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AutoInitControl::Response>::value > { using type = commands_filter::AutoInitControl::Response; };

template<>
struct MetadataFor<commands_filter::AutoInitControl>
{
    using type = commands_filter::AutoInitControl;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "See above",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AutoInitControl",
            /* .title       = */ "Auto-initialization Control",
            /* .docs        = */ "Filter Auto-initialization Control\n\nEnable/Disable automatic initialization upon device startup.\n\nPossible enable values:\n\n0x00 - Disable auto-initialization\n0x01 - Enable auto-initialization\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AutoInitControl>::value > { using type = commands_filter::AutoInitControl; };
template<> struct TypeForDescriptor<commands_filter::AutoInitControl::DESCRIPTOR.as_u16()> { using type = commands_filter::AutoInitControl; };

template<>
struct MetadataFor<commands_filter::AccelNoise::Response>
{
    using type = commands_filter::AccelNoise::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Accel Noise 1-sigma [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AccelNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AccelNoise::Response>::value > { using type = commands_filter::AccelNoise::Response; };

template<>
struct MetadataFor<commands_filter::AccelNoise>
{
    using type = commands_filter::AccelNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Accel Noise 1-sigma [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AccelNoise",
            /* .title       = */ "Accelerometer Noise Standard Deviation",
            /* .docs        = */ "Accelerometer Noise Standard Deviation\n\nEach of the noise values must be greater than 0.0.\n\nThe noise value represents process noise in the Estimation Filter.\nChanging this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.\nDefault values provide good performance for most laboratory conditions.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AccelNoise>::value > { using type = commands_filter::AccelNoise; };
template<> struct TypeForDescriptor<commands_filter::AccelNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::AccelNoise; };

template<>
struct MetadataFor<commands_filter::GyroNoise::Response>
{
    using type = commands_filter::GyroNoise::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Gyro Noise 1-sigma [rad/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GyroNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GyroNoise::Response>::value > { using type = commands_filter::GyroNoise::Response; };

template<>
struct MetadataFor<commands_filter::GyroNoise>
{
    using type = commands_filter::GyroNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Gyro Noise 1-sigma [rad/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GyroNoise",
            /* .title       = */ "Gyroscope Noise Standard Deviation",
            /* .docs        = */ "Gyroscope Noise Standard Deviation\n\nEach of the noise values must be greater than 0.0\n\nThe noise value represents process noise in the Estimation Filter.\nChanging this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.\nDefault values provide good performance for most laboratory conditions.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GyroNoise>::value > { using type = commands_filter::GyroNoise; };
template<> struct TypeForDescriptor<commands_filter::GyroNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::GyroNoise; };

template<>
struct MetadataFor<commands_filter::AccelBiasModel::Response>
{
    using type = commands_filter::AccelBiasModel::Response;

    using ParamTypes = std::tuple<
        Vector3f,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.beta;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Accel Bias Beta [1/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::beta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Accel Noise 1-sigma [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AccelBiasModel::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AccelBiasModel::Response>::value > { using type = commands_filter::AccelBiasModel::Response; };

template<>
struct MetadataFor<commands_filter::AccelBiasModel>
{
    using type = commands_filter::AccelBiasModel;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.beta;
        if constexpr(I == 2) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Accel Bias Beta [1/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::beta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Accel Noise 1-sigma [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AccelBiasModel",
            /* .title       = */ "Accelerometer Bias Model Parameters",
            /* .docs        = */ "Accelerometer Bias Model Parameters\n\nNoise values must be greater than 0.0\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AccelBiasModel>::value > { using type = commands_filter::AccelBiasModel; };
template<> struct TypeForDescriptor<commands_filter::AccelBiasModel::DESCRIPTOR.as_u16()> { using type = commands_filter::AccelBiasModel; };

template<>
struct MetadataFor<commands_filter::GyroBiasModel::Response>
{
    using type = commands_filter::GyroBiasModel::Response;

    using ParamTypes = std::tuple<
        Vector3f,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.beta;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Gyro Bias Beta [1/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::beta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Gyro Noise 1-sigma [rad/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GyroBiasModel::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GyroBiasModel::Response>::value > { using type = commands_filter::GyroBiasModel::Response; };

template<>
struct MetadataFor<commands_filter::GyroBiasModel>
{
    using type = commands_filter::GyroBiasModel;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.beta;
        if constexpr(I == 2) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "beta",
            /* .docs          = */ "Gyro Bias Beta [1/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::beta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Gyro Noise 1-sigma [rad/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GyroBiasModel",
            /* .title       = */ "Gyroscope Bias Model Parameters",
            /* .docs        = */ "Gyroscope Bias Model Parameters\n\nNoise values must be greater than 0.0\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GyroBiasModel>::value > { using type = commands_filter::GyroBiasModel; };
template<> struct TypeForDescriptor<commands_filter::GyroBiasModel::DESCRIPTOR.as_u16()> { using type = commands_filter::GyroBiasModel; };

template<>
struct MetadataFor<commands_filter::AltitudeAiding::AidingSelector>
{
    using type = commands_filter::AltitudeAiding::AidingSelector;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "NONE", "No altitude aiding" },
        { uint32_t(1), "PRESURE", "Enable pressure sensor aiding" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "AidingSelector",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::AltitudeAiding::AidingSelector>::value > { using type = commands_filter::AltitudeAiding::AidingSelector; };

template<>
struct MetadataFor<commands_filter::AltitudeAiding::Response>
{
    using type = commands_filter::AltitudeAiding::Response;

    using ParamTypes = std::tuple<
        commands_filter::AltitudeAiding::AidingSelector
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.selector;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "selector",
            /* .docs          = */ "See above",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::AltitudeAiding::AidingSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::AltitudeAiding::AidingSelector, &type::selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AltitudeAiding::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AltitudeAiding::Response>::value > { using type = commands_filter::AltitudeAiding::Response; };

template<>
struct MetadataFor<commands_filter::AltitudeAiding>
{
    using type = commands_filter::AltitudeAiding;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::AltitudeAiding::AidingSelector
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.selector;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "selector",
            /* .docs          = */ "See above",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::AltitudeAiding::AidingSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::AltitudeAiding::AidingSelector, &type::selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AltitudeAiding",
            /* .title       = */ "Altitude Aiding Control",
            /* .docs        = */ "Select altitude input for absolute altitude and/or vertical velocity. The primary altitude reading is always GNSS.\nAiding inputs are used to improve GNSS altitude readings when GNSS is available and to backup GNSS during outages.\n\nPressure altitude is based on 'instant sea level pressure' which is dependent on location and weather conditions and can vary by more than 40 meters.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AltitudeAiding>::value > { using type = commands_filter::AltitudeAiding; };
template<> struct TypeForDescriptor<commands_filter::AltitudeAiding::DESCRIPTOR.as_u16()> { using type = commands_filter::AltitudeAiding; };

template<>
struct MetadataFor<commands_filter::PitchRollAiding::AidingSource>
{
    using type = commands_filter::PitchRollAiding::AidingSource;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "NONE", "No pitch/roll aiding" },
        { uint32_t(1), "GRAVITY_VEC", "Enable gravity vector aiding" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "AidingSource",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::PitchRollAiding::AidingSource>::value > { using type = commands_filter::PitchRollAiding::AidingSource; };

template<>
struct MetadataFor<commands_filter::PitchRollAiding::Response>
{
    using type = commands_filter::PitchRollAiding::Response;

    using ParamTypes = std::tuple<
        commands_filter::PitchRollAiding::AidingSource
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "Controls the aiding source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::PitchRollAiding::AidingSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::PitchRollAiding::AidingSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::PitchRollAiding::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::PitchRollAiding::Response>::value > { using type = commands_filter::PitchRollAiding::Response; };

template<>
struct MetadataFor<commands_filter::PitchRollAiding>
{
    using type = commands_filter::PitchRollAiding;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::PitchRollAiding::AidingSource
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "Controls the aiding source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::PitchRollAiding::AidingSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::PitchRollAiding::AidingSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::PitchRollAiding",
            /* .title       = */ "Pitch/Roll Aiding Control",
            /* .docs        = */ "Select pitch/roll aiding input. Pitch/roll reading is always derived from GNSS corrected inertial solution.\nAiding inputs are used to improve that solution during periods of low dynamics and GNSS outages.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::PitchRollAiding>::value > { using type = commands_filter::PitchRollAiding; };
template<> struct TypeForDescriptor<commands_filter::PitchRollAiding::DESCRIPTOR.as_u16()> { using type = commands_filter::PitchRollAiding; };

template<>
struct MetadataFor<commands_filter::AutoZupt::Response>
{
    using type = commands_filter::AutoZupt::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
        if constexpr(I == 1) return value_.threshold;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "threshold",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::threshold>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AutoZupt::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AutoZupt::Response>::value > { using type = commands_filter::AutoZupt::Response; };

template<>
struct MetadataFor<commands_filter::AutoZupt>
{
    using type = commands_filter::AutoZupt;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
        if constexpr(I == 2) return value_.threshold;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "threshold",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::threshold>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AutoZupt",
            /* .title       = */ "Zero Velocity Update Control",
            /* .docs        = */ "The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.\nThe device will NACK threshold values that are less than zero (i.e.negative.)",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AutoZupt>::value > { using type = commands_filter::AutoZupt; };
template<> struct TypeForDescriptor<commands_filter::AutoZupt::DESCRIPTOR.as_u16()> { using type = commands_filter::AutoZupt; };

template<>
struct MetadataFor<commands_filter::AutoAngularZupt::Response>
{
    using type = commands_filter::AutoAngularZupt::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
        if constexpr(I == 1) return value_.threshold;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "threshold",
            /* .docs          = */ "[radians/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::threshold>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AutoAngularZupt::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AutoAngularZupt::Response>::value > { using type = commands_filter::AutoAngularZupt::Response; };

template<>
struct MetadataFor<commands_filter::AutoAngularZupt>
{
    using type = commands_filter::AutoAngularZupt;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
        if constexpr(I == 2) return value_.threshold;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "threshold",
            /* .docs          = */ "[radians/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::threshold>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AutoAngularZupt",
            /* .title       = */ "Zero Angular Rate Update Control",
            /* .docs        = */ "Zero Angular Rate Update\nThe ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.\nThe device will NACK threshold values that are less than zero (i.e.negative.)",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AutoAngularZupt>::value > { using type = commands_filter::AutoAngularZupt; };
template<> struct TypeForDescriptor<commands_filter::AutoAngularZupt::DESCRIPTOR.as_u16()> { using type = commands_filter::AutoAngularZupt; };

template<>
struct MetadataFor<commands_filter::CommandedZupt>
{
    using type = commands_filter::CommandedZupt;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::CommandedZupt",
            /* .title       = */ "Commanded Zero Velocity Update",
            /* .docs        = */ "Please see the device user manual for the maximum rate of this message.",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::CommandedZupt>::value > { using type = commands_filter::CommandedZupt; };
template<> struct TypeForDescriptor<commands_filter::CommandedZupt::DESCRIPTOR.as_u16()> { using type = commands_filter::CommandedZupt; };

template<>
struct MetadataFor<commands_filter::CommandedAngularZupt>
{
    using type = commands_filter::CommandedAngularZupt;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::CommandedAngularZupt",
            /* .title       = */ "Commanded Zero Angular Rate Update",
            /* .docs        = */ "Please see the device user manual for the maximum rate of this message.",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::CommandedAngularZupt>::value > { using type = commands_filter::CommandedAngularZupt; };
template<> struct TypeForDescriptor<commands_filter::CommandedAngularZupt::DESCRIPTOR.as_u16()> { using type = commands_filter::CommandedAngularZupt; };

template<>
struct MetadataFor<commands_filter::MagCaptureAutoCal>
{
    using type = commands_filter::MagCaptureAutoCal;

    using ParamTypes = std::tuple<
        FunctionSelector
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,

    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagCaptureAutoCal",
            /* .title       = */ "Magnetometer Capture Auto Calibration",
            /* .docs        = */ "This command captures the current value of the auto-calibration, applies it to the current fixed hard and soft iron calibration coefficients, and replaces the current fixed hard and soft iron calibration coefficients with the new values.\nThis may be used in place of (or in addition to) a manual hard and soft iron calibration utility. This command also resets the auto-calibration coefficients.\nFunction selector SAVE is the same as issuing the 0x0C, 0x3A and 0x0C, 0x3B commands with the SAVE function selector.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, false, true, false, false},
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagCaptureAutoCal>::value > { using type = commands_filter::MagCaptureAutoCal; };
template<> struct TypeForDescriptor<commands_filter::MagCaptureAutoCal::DESCRIPTOR.as_u16()> { using type = commands_filter::MagCaptureAutoCal; };

template<>
struct MetadataFor<commands_filter::GravityNoise::Response>
{
    using type = commands_filter::GravityNoise::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Gravity Noise 1-sigma [gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GravityNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GravityNoise::Response>::value > { using type = commands_filter::GravityNoise::Response; };

template<>
struct MetadataFor<commands_filter::GravityNoise>
{
    using type = commands_filter::GravityNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Gravity Noise 1-sigma [gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GravityNoise",
            /* .title       = */ "Gravity Noise Standard Deviation",
            /* .docs        = */ "Set the expected gravity noise 1-sigma values. This function can be used to tune the filter performance in the target application.\n\nNote: Noise values must be greater than 0.0\n\nThe noise value represents process noise in the Estimation Filter. Changing this value modifies how the filter responds to dynamic input and can be used to tune filter performance.\nDefault values provide good performance for most laboratory conditions.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GravityNoise>::value > { using type = commands_filter::GravityNoise; };
template<> struct TypeForDescriptor<commands_filter::GravityNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::GravityNoise; };

template<>
struct MetadataFor<commands_filter::PressureAltitudeNoise::Response>
{
    using type = commands_filter::PressureAltitudeNoise::Response;

    using ParamTypes = std::tuple<
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Pressure Altitude Noise 1-sigma [m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::PressureAltitudeNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::PressureAltitudeNoise::Response>::value > { using type = commands_filter::PressureAltitudeNoise::Response; };

template<>
struct MetadataFor<commands_filter::PressureAltitudeNoise>
{
    using type = commands_filter::PressureAltitudeNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Pressure Altitude Noise 1-sigma [m]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::PressureAltitudeNoise",
            /* .title       = */ "Pressure Altitude Noise Standard Deviation",
            /* .docs        = */ "Set the expected pressure altitude noise 1-sigma values. This function can be used to tune the filter performance in the target application.\n\nThe noise value must be greater than 0.0\n\nThis noise value represents pressure altitude model noise in the Estimation Filter.\nA lower value will increase responsiveness of the sensor to pressure changes, however height estimates will be more susceptible to error from air pressure fluctuations not due to changes in altitude. Default values provide good performance for most laboratory conditions.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::PressureAltitudeNoise>::value > { using type = commands_filter::PressureAltitudeNoise; };
template<> struct TypeForDescriptor<commands_filter::PressureAltitudeNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::PressureAltitudeNoise; };

template<>
struct MetadataFor<commands_filter::HardIronOffsetNoise::Response>
{
    using type = commands_filter::HardIronOffsetNoise::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Hard Iron Offset Noise 1-sigma [gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::HardIronOffsetNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::HardIronOffsetNoise::Response>::value > { using type = commands_filter::HardIronOffsetNoise::Response; };

template<>
struct MetadataFor<commands_filter::HardIronOffsetNoise>
{
    using type = commands_filter::HardIronOffsetNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Hard Iron Offset Noise 1-sigma [gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::HardIronOffsetNoise",
            /* .title       = */ "Hard Iron Offset Process Noise",
            /* .docs        = */ "Set the expected hard iron offset noise 1-sigma values. This function can be used to tune the filter performance in the target application.\n\nThis function can be used to tune the filter performance in the target application.\n\nNoise values must be greater than 0.0\n\nThe noise values represent process noise in the Estimation Filter.\nChanging this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::HardIronOffsetNoise>::value > { using type = commands_filter::HardIronOffsetNoise; };
template<> struct TypeForDescriptor<commands_filter::HardIronOffsetNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::HardIronOffsetNoise; };

template<>
struct MetadataFor<commands_filter::SoftIronMatrixNoise::Response>
{
    using type = commands_filter::SoftIronMatrixNoise::Response;

    using ParamTypes = std::tuple<
        Matrix3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Soft Iron Matrix Noise 1-sigma [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SoftIronMatrixNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SoftIronMatrixNoise::Response>::value > { using type = commands_filter::SoftIronMatrixNoise::Response; };

template<>
struct MetadataFor<commands_filter::SoftIronMatrixNoise>
{
    using type = commands_filter::SoftIronMatrixNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Matrix3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Soft Iron Matrix Noise 1-sigma [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SoftIronMatrixNoise",
            /* .title       = */ "Soft Iron Offset Process Noise",
            /* .docs        = */ "Set the expected soft iron matrix noise 1-sigma values.\nThis function can be used to tune the filter performance in the target application.\n\nNoise values must be greater than 0.0\n\nThe noise value represents process noise in the Estimation Filter.\nChanging this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SoftIronMatrixNoise>::value > { using type = commands_filter::SoftIronMatrixNoise; };
template<> struct TypeForDescriptor<commands_filter::SoftIronMatrixNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::SoftIronMatrixNoise; };

template<>
struct MetadataFor<commands_filter::MagNoise::Response>
{
    using type = commands_filter::MagNoise::Response;

    using ParamTypes = std::tuple<
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Mag Noise 1-sigma [gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagNoise::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagNoise::Response>::value > { using type = commands_filter::MagNoise::Response; };

template<>
struct MetadataFor<commands_filter::MagNoise>
{
    using type = commands_filter::MagNoise;

    using ParamTypes = std::tuple<
        FunctionSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.noise;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "noise",
            /* .docs          = */ "Mag Noise 1-sigma [gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::noise>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagNoise",
            /* .title       = */ "Magnetometer Noise Standard Deviation",
            /* .docs        = */ "Set the expected magnetometer noise 1-sigma values.\nThis function can be used to tune the filter performance in the target application.\n\nNoise values must be greater than 0.0 (gauss)\n\nThe noise value represents process noise in the Estimation Filter.\nChanging this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagNoise>::value > { using type = commands_filter::MagNoise; };
template<> struct TypeForDescriptor<commands_filter::MagNoise::DESCRIPTOR.as_u16()> { using type = commands_filter::MagNoise; };

template<>
struct MetadataFor<commands_filter::FilterMagParamSource>
{
    using type = commands_filter::FilterMagParamSource;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "NONE", "No source. See command documentation for default behavior" },
        { uint32_t(2), "WMM", "Magnetic field is assumed to conform to the World Magnetic Model, calculated using current location estimate as an input to the model." },
        { uint32_t(3), "MANUAL", "Magnetic field is assumed to have the parameter specified by the user." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FilterMagParamSource",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::FilterMagParamSource>::value > { using type = commands_filter::FilterMagParamSource; };

template<>
struct MetadataFor<commands_filter::InclinationSource::Response>
{
    using type = commands_filter::InclinationSource::Response;

    using ParamTypes = std::tuple<
        commands_filter::FilterMagParamSource,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
        if constexpr(I == 1) return value_.inclination;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "Inclination Source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterMagParamSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterMagParamSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [radians] (only required if source = MANUAL)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::inclination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::InclinationSource::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::InclinationSource::Response>::value > { using type = commands_filter::InclinationSource::Response; };

template<>
struct MetadataFor<commands_filter::InclinationSource>
{
    using type = commands_filter::InclinationSource;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::FilterMagParamSource,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
        if constexpr(I == 2) return value_.inclination;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "Inclination Source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterMagParamSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterMagParamSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "Inclination angle [radians] (only required if source = MANUAL)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::inclination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::InclinationSource",
            /* .title       = */ "Inclination Source",
            /* .docs        = */ "Set/Get the local magnetic field inclination angle source.\n\nThis can be used to correct for the local value of inclination (dip angle) of the earthmagnetic field.\nHaving a correct value is important for best performance of the auto-mag calibration feature. If you do not have an accurate inclination angle source, it is recommended that you leave the auto-mag calibration feature off.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::InclinationSource>::value > { using type = commands_filter::InclinationSource; };
template<> struct TypeForDescriptor<commands_filter::InclinationSource::DESCRIPTOR.as_u16()> { using type = commands_filter::InclinationSource; };

template<>
struct MetadataFor<commands_filter::MagneticDeclinationSource::Response>
{
    using type = commands_filter::MagneticDeclinationSource::Response;

    using ParamTypes = std::tuple<
        commands_filter::FilterMagParamSource,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
        if constexpr(I == 1) return value_.declination;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "Magnetic field declination angle source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterMagParamSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterMagParamSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "declination",
            /* .docs          = */ "Declination angle [radians] (only required if source = MANUAL)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::declination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagneticDeclinationSource::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagneticDeclinationSource::Response>::value > { using type = commands_filter::MagneticDeclinationSource::Response; };

template<>
struct MetadataFor<commands_filter::MagneticDeclinationSource>
{
    using type = commands_filter::MagneticDeclinationSource;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::FilterMagParamSource,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
        if constexpr(I == 2) return value_.declination;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "Magnetic field declination angle source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterMagParamSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterMagParamSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "declination",
            /* .docs          = */ "Declination angle [radians] (only required if source = MANUAL)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::declination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagneticDeclinationSource",
            /* .title       = */ "Magnetic Field Declination Source Control",
            /* .docs        = */ "Set/Get the local magnetic field declination angle source.\n\nThis can be used to correct for the local value of declination of the earthmagnetic field.\nHaving a correct value is important for best performance of the auto-mag calibration feature. If you do not have an accurate inclination angle source, it is recommended that you leave the auto-mag calibration feature off.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagneticDeclinationSource>::value > { using type = commands_filter::MagneticDeclinationSource; };
template<> struct TypeForDescriptor<commands_filter::MagneticDeclinationSource::DESCRIPTOR.as_u16()> { using type = commands_filter::MagneticDeclinationSource; };

template<>
struct MetadataFor<commands_filter::MagFieldMagnitudeSource::Response>
{
    using type = commands_filter::MagFieldMagnitudeSource::Response;

    using ParamTypes = std::tuple<
        commands_filter::FilterMagParamSource,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
        if constexpr(I == 1) return value_.magnitude;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "Magnetic Field Magnitude Source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterMagParamSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterMagParamSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "magnitude",
            /* .docs          = */ "Magnitude [gauss] (only required if source = MANUAL)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::magnitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagFieldMagnitudeSource::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagFieldMagnitudeSource::Response>::value > { using type = commands_filter::MagFieldMagnitudeSource::Response; };

template<>
struct MetadataFor<commands_filter::MagFieldMagnitudeSource>
{
    using type = commands_filter::MagFieldMagnitudeSource;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::FilterMagParamSource,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
        if constexpr(I == 2) return value_.magnitude;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "Magnetic Field Magnitude Source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterMagParamSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterMagParamSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "magnitude",
            /* .docs          = */ "Magnitude [gauss] (only required if source = MANUAL)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::magnitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagFieldMagnitudeSource",
            /* .title       = */ "Magnetic Field Magnitude Source",
            /* .docs        = */ "Set/Get the local magnetic field magnitude source.\n\nThis is used to specify the local magnitude of the earth's magnetic field.\nHaving a correct value for magnitude is important for best performance of the auto-mag calibration feature and for the magnetometer adaptive magnitude. If you do not have an accurate value for the local magnetic field magnitude, it is recommended that you leave the auto-mag calibration feature off.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagFieldMagnitudeSource>::value > { using type = commands_filter::MagFieldMagnitudeSource; };
template<> struct TypeForDescriptor<commands_filter::MagFieldMagnitudeSource::DESCRIPTOR.as_u16()> { using type = commands_filter::MagFieldMagnitudeSource; };

template<>
struct MetadataFor<commands_filter::ReferencePosition::Response>
{
    using type = commands_filter::ReferencePosition::Response;

    using ParamTypes = std::tuple<
        bool,
        double,
        double,
        double
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
        if constexpr(I == 1) return value_.latitude;
        if constexpr(I == 2) return value_.longitude;
        if constexpr(I == 3) return value_.altitude;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "enable/disable",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
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
            /* .name          = */ "altitude",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::altitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::ReferencePosition::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::ReferencePosition::Response>::value > { using type = commands_filter::ReferencePosition::Response; };

template<>
struct MetadataFor<commands_filter::ReferencePosition>
{
    using type = commands_filter::ReferencePosition;

    using ParamTypes = std::tuple<
        FunctionSelector,
        bool,
        double,
        double,
        double
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
        if constexpr(I == 2) return value_.latitude;
        if constexpr(I == 3) return value_.longitude;
        if constexpr(I == 4) return value_.altitude;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "enable/disable",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
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
            /* .name          = */ "altitude",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::DOUBLE, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, double, &type::altitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::ReferencePosition",
            /* .title       = */ "Set Reference Position",
            /* .docs        = */ "Set the Lat/Long/Alt reference position for the sensor.\n\nThis position is used by the sensor to calculate the WGS84 gravity and WMM2015 magnetic field parameters.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::ReferencePosition>::value > { using type = commands_filter::ReferencePosition; };
template<> struct TypeForDescriptor<commands_filter::ReferencePosition::DESCRIPTOR.as_u16()> { using type = commands_filter::ReferencePosition; };

template<>
struct MetadataFor<commands_filter::FilterAdaptiveMeasurement>
{
    using type = commands_filter::FilterAdaptiveMeasurement;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "DISABLED", "No adaptive measurement" },
        { uint32_t(1), "FIXED", "Enable fixed adaptive measurement (use specified limits)" },
        { uint32_t(2), "AUTO", "Enable auto adaptive measurement" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FilterAdaptiveMeasurement",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::FilterAdaptiveMeasurement>::value > { using type = commands_filter::FilterAdaptiveMeasurement; };

template<>
struct MetadataFor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response>
{
    using type = commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response;

    using ParamTypes = std::tuple<
        commands_filter::FilterAdaptiveMeasurement,
        float,
        float,
        float,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.adaptive_measurement;
        if constexpr(I == 1) return value_.frequency;
        if constexpr(I == 2) return value_.low_limit;
        if constexpr(I == 3) return value_.high_limit;
        if constexpr(I == 4) return value_.low_limit_uncertainty;
        if constexpr(I == 5) return value_.high_limit_uncertainty;
        if constexpr(I == 6) return value_.minimum_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "adaptive_measurement",
            /* .docs          = */ "Adaptive measurement selector",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterAdaptiveMeasurement>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterAdaptiveMeasurement, &type::adaptive_measurement>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frequency",
            /* .docs          = */ "Low-pass filter curoff frequency [hertz]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::frequency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "minimum_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::minimum_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response>::value > { using type = commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response; };

template<>
struct MetadataFor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement>
{
    using type = commands_filter::AccelMagnitudeErrorAdaptiveMeasurement;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::FilterAdaptiveMeasurement,
        float,
        float,
        float,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.adaptive_measurement;
        if constexpr(I == 2) return value_.frequency;
        if constexpr(I == 3) return value_.low_limit;
        if constexpr(I == 4) return value_.high_limit;
        if constexpr(I == 5) return value_.low_limit_uncertainty;
        if constexpr(I == 6) return value_.high_limit_uncertainty;
        if constexpr(I == 7) return value_.minimum_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "adaptive_measurement",
            /* .docs          = */ "Adaptive measurement selector",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterAdaptiveMeasurement>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterAdaptiveMeasurement, &type::adaptive_measurement>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frequency",
            /* .docs          = */ "Low-pass filter curoff frequency [hertz]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::frequency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "minimum_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::minimum_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AccelMagnitudeErrorAdaptiveMeasurement",
            /* .title       = */ "Gravity Magnitude Error Adaptive Measurement",
            /* .docs        = */ "Enable or disable the gravity magnitude error adaptive measurement.\nThis function can be used to tune the filter performance in the target application\n\nPick values that give you the least occurrence of invalid EF attitude output.\nThe default values are good for standard low dynamics applications.\nIncrease values for higher dynamic conditions, lower values for lower dynamic.\nToo low a value will result in excessive heading errors.\nHigher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.\n\nAdaptive measurements can be enabled/disabled without the need for providing the additional parameters.\nIn this case, only the function selector and enable value are required; all other parameters will remain at their previous values.\nWhen 'auto-adaptive' is selected, the filter and limit parameters are ignored.\nInstead, aiding measurements which rely on the gravity vector will be automatically reweighted by the Kalman filter according to the perceived measurement quality.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement>::value > { using type = commands_filter::AccelMagnitudeErrorAdaptiveMeasurement; };
template<> struct TypeForDescriptor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::DESCRIPTOR.as_u16()> { using type = commands_filter::AccelMagnitudeErrorAdaptiveMeasurement; };

template<>
struct MetadataFor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response>
{
    using type = commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response;

    using ParamTypes = std::tuple<
        commands_filter::FilterAdaptiveMeasurement,
        float,
        float,
        float,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.adaptive_measurement;
        if constexpr(I == 1) return value_.frequency;
        if constexpr(I == 2) return value_.low_limit;
        if constexpr(I == 3) return value_.high_limit;
        if constexpr(I == 4) return value_.low_limit_uncertainty;
        if constexpr(I == 5) return value_.high_limit_uncertainty;
        if constexpr(I == 6) return value_.minimum_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "adaptive_measurement",
            /* .docs          = */ "Adaptive measurement selector",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterAdaptiveMeasurement>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterAdaptiveMeasurement, &type::adaptive_measurement>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frequency",
            /* .docs          = */ "Low-pass filter curoff frequency [hertz]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::frequency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "minimum_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::minimum_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response>::value > { using type = commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response; };

template<>
struct MetadataFor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement>
{
    using type = commands_filter::MagMagnitudeErrorAdaptiveMeasurement;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::FilterAdaptiveMeasurement,
        float,
        float,
        float,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.adaptive_measurement;
        if constexpr(I == 2) return value_.frequency;
        if constexpr(I == 3) return value_.low_limit;
        if constexpr(I == 4) return value_.high_limit;
        if constexpr(I == 5) return value_.low_limit_uncertainty;
        if constexpr(I == 6) return value_.high_limit_uncertainty;
        if constexpr(I == 7) return value_.minimum_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "adaptive_measurement",
            /* .docs          = */ "Adaptive measurement selector",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterAdaptiveMeasurement>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterAdaptiveMeasurement, &type::adaptive_measurement>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frequency",
            /* .docs          = */ "Low-pass filter curoff frequency [hertz]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::frequency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "low_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::low_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "minimum_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::minimum_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagMagnitudeErrorAdaptiveMeasurement",
            /* .title       = */ "Magnetometer Magnitude Error Adaptive Measurement",
            /* .docs        = */ "Enable or disable the magnetometer magnitude error adaptive measurement.\nThis feature will reject magnetometer readings that are out of range of the thresholds specified (fixed adaptive) or calculated internally (auto-adaptive).\n\nPick values that give you the least occurrence of invalid EF attitude output.\nThe default values are good for standard low dynamics applications.\nIncrease values for higher dynamic conditions, lower values for lower dynamic.\nToo low a value will result in excessive heading errors.\nHigher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement>::value > { using type = commands_filter::MagMagnitudeErrorAdaptiveMeasurement; };
template<> struct TypeForDescriptor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement::DESCRIPTOR.as_u16()> { using type = commands_filter::MagMagnitudeErrorAdaptiveMeasurement; };

template<>
struct MetadataFor<commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response>
{
    using type = commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response;

    using ParamTypes = std::tuple<
        bool,
        float,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
        if constexpr(I == 1) return value_.frequency;
        if constexpr(I == 2) return value_.high_limit;
        if constexpr(I == 3) return value_.high_limit_uncertainty;
        if constexpr(I == 4) return value_.minimum_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "Enable/Disable",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frequency",
            /* .docs          = */ "Low-pass filter curoff frequency [hertz]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::frequency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "minimum_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::minimum_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response>::value > { using type = commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response; };

template<>
struct MetadataFor<commands_filter::MagDipAngleErrorAdaptiveMeasurement>
{
    using type = commands_filter::MagDipAngleErrorAdaptiveMeasurement;

    using ParamTypes = std::tuple<
        FunctionSelector,
        bool,
        float,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
        if constexpr(I == 2) return value_.frequency;
        if constexpr(I == 3) return value_.high_limit;
        if constexpr(I == 4) return value_.high_limit_uncertainty;
        if constexpr(I == 5) return value_.minimum_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "Enable/Disable",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "frequency",
            /* .docs          = */ "Low-pass filter curoff frequency [hertz]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::frequency>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "high_limit_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::high_limit_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "minimum_uncertainty",
            /* .docs          = */ "1-Sigma [meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::minimum_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MagDipAngleErrorAdaptiveMeasurement",
            /* .title       = */ "Magnetometer Dig Angle Error Adaptive Measurement",
            /* .docs        = */ "Enable or disable the magnetometer dip angle error adaptive measurement.\nThis function can be used to tune the filter performance in the target application\n\nPick values that give you the least occurrence of invalid EF attitude output.\nThe default values are good for standard low dynamics applications.\nIncrease values for higher dynamic conditions, lower values for lower dynamic.\nToo low a value will result in excessive heading errors.\nHigher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.\n\nThe magnetometer dip angle adaptive measurement is ignored if the auto-adaptive magnetometer magnitude or auto-adaptive accel magnitude options are selected.\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MagDipAngleErrorAdaptiveMeasurement>::value > { using type = commands_filter::MagDipAngleErrorAdaptiveMeasurement; };
template<> struct TypeForDescriptor<commands_filter::MagDipAngleErrorAdaptiveMeasurement::DESCRIPTOR.as_u16()> { using type = commands_filter::MagDipAngleErrorAdaptiveMeasurement; };

template<>
struct MetadataFor<commands_filter::AidingMeasurementEnable::AidingSource>
{
    using type = commands_filter::AidingMeasurementEnable::AidingSource;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "GNSS_POS_VEL", "GNSS Position and Velocity" },
        { uint32_t(1), "GNSS_HEADING", "GNSS Heading (dual antenna)" },
        { uint32_t(2), "ALTIMETER", "Pressure altimeter (built-in sensor)" },
        { uint32_t(3), "SPEED", "Speed sensor / Odometer" },
        { uint32_t(4), "MAGNETOMETER", "Magnetometer (built-in sensor)" },
        { uint32_t(5), "EXTERNAL_HEADING", "External heading input" },
        { uint32_t(6), "EXTERNAL_ALTIMETER", "External pressure altimeter input" },
        { uint32_t(7), "EXTERNAL_MAGNETOMETER", "External magnetomer input" },
        { uint32_t(8), "BODY_FRAME_VEL", "External body frame velocity input" },
        { uint32_t(65535), "ALL", "Save/load/reset all options" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "AidingSource",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::AidingMeasurementEnable::AidingSource>::value > { using type = commands_filter::AidingMeasurementEnable::AidingSource; };

template<>
struct MetadataFor<commands_filter::AidingMeasurementEnable::Response>
{
    using type = commands_filter::AidingMeasurementEnable::Response;

    using ParamTypes = std::tuple<
        commands_filter::AidingMeasurementEnable::AidingSource,
        bool
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.aiding_source;
        if constexpr(I == 1) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "aiding_source",
            /* .docs          = */ "Aiding measurement source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::AidingMeasurementEnable::AidingSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::AidingMeasurementEnable::AidingSource, &type::aiding_source>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "enable",
            /* .docs          = */ "Controls the aiding source",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AidingMeasurementEnable::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AidingMeasurementEnable::Response>::value > { using type = commands_filter::AidingMeasurementEnable::Response; };

template<>
struct MetadataFor<commands_filter::AidingMeasurementEnable>
{
    using type = commands_filter::AidingMeasurementEnable;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::AidingMeasurementEnable::AidingSource,
        bool
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.aiding_source;
        if constexpr(I == 2) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "aiding_source",
            /* .docs          = */ "Aiding measurement source",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::AidingMeasurementEnable::AidingSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::AidingMeasurementEnable::AidingSource, &type::aiding_source>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "enable",
            /* .docs          = */ "Controls the aiding source",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AidingMeasurementEnable",
            /* .title       = */ "Aiding Measurement Control",
            /* .docs        = */ "Enables / disables the specified aiding measurement source.\n\n",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AidingMeasurementEnable>::value > { using type = commands_filter::AidingMeasurementEnable; };
template<> struct TypeForDescriptor<commands_filter::AidingMeasurementEnable::DESCRIPTOR.as_u16()> { using type = commands_filter::AidingMeasurementEnable; };

template<>
struct MetadataFor<commands_filter::Run>
{
    using type = commands_filter::Run;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::Run",
            /* .title       = */ "Run Navigation Filter",
            /* .docs        = */ "Manual run command.\n\nIf the initialization configuration has the 'wait_for_run_command' option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::Run>::value > { using type = commands_filter::Run; };
template<> struct TypeForDescriptor<commands_filter::Run::DESCRIPTOR.as_u16()> { using type = commands_filter::Run; };

template<>
struct MetadataFor<commands_filter::KinematicConstraint::Response>
{
    using type = commands_filter::KinematicConstraint::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        uint8_t,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.acceleration_constraint_selection;
        if constexpr(I == 1) return value_.velocity_constraint_selection;
        if constexpr(I == 2) return value_.angular_constraint_selection;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "acceleration_constraint_selection",
            /* .docs          = */ "Acceleration constraint: <br/>\n0=None (default), <br/>\n1=Zero-acceleration.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::acceleration_constraint_selection>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity_constraint_selection",
            /* .docs          = */ "0=None (default), <br/>\n1=Zero-velocity, <br/>\n2=Wheeled-vehicle. <br/>",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::velocity_constraint_selection>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "angular_constraint_selection",
            /* .docs          = */ "0=None (default),\n1=Zero-angular rate (ZUPT).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::angular_constraint_selection>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::KinematicConstraint::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::KinematicConstraint::Response>::value > { using type = commands_filter::KinematicConstraint::Response; };

template<>
struct MetadataFor<commands_filter::KinematicConstraint>
{
    using type = commands_filter::KinematicConstraint;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        uint8_t,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.acceleration_constraint_selection;
        if constexpr(I == 2) return value_.velocity_constraint_selection;
        if constexpr(I == 3) return value_.angular_constraint_selection;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "acceleration_constraint_selection",
            /* .docs          = */ "Acceleration constraint: <br/>\n0=None (default), <br/>\n1=Zero-acceleration.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::acceleration_constraint_selection>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "velocity_constraint_selection",
            /* .docs          = */ "0=None (default), <br/>\n1=Zero-velocity, <br/>\n2=Wheeled-vehicle. <br/>",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::velocity_constraint_selection>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "angular_constraint_selection",
            /* .docs          = */ "0=None (default),\n1=Zero-angular rate (ZUPT).",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::angular_constraint_selection>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::KinematicConstraint",
            /* .title       = */ "Kinematic Constraint Control",
            /* .docs        = */ "Controls kinematic constraint model selection for the navigation filter.\n\nSee manual for explanation of how the kinematic constraints are applied.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::KinematicConstraint>::value > { using type = commands_filter::KinematicConstraint; };
template<> struct TypeForDescriptor<commands_filter::KinematicConstraint::DESCRIPTOR.as_u16()> { using type = commands_filter::KinematicConstraint; };

template<>
struct MetadataFor<commands_filter::FilterReferenceFrame>
{
    using type = commands_filter::FilterReferenceFrame;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "ECEF", "WGS84 Earth-fixed, earth centered coordinates" },
        { uint32_t(2), "LLH", "WGS84 Latitude, longitude, and height above ellipsoid" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FilterReferenceFrame",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::FilterReferenceFrame>::value > { using type = commands_filter::FilterReferenceFrame; };

template<>
struct MetadataFor<commands_filter::InitializationConfiguration::AlignmentSelector>
{
    using type = commands_filter::InitializationConfiguration::AlignmentSelector;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "dual_antenna", "Dual-antenna GNSS alignment" },
        { uint32_t(2), "kinematic", "GNSS kinematic alignment (GNSS velocity determines initial heading)" },
        { uint32_t(4), "magnetometer", "Magnetometer heading alignment (Internal magnetometer determines initial heading)" },
        { uint32_t(8), "external", "External heading alignment (External heading input determines heading)" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "AlignmentSelector",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForBitsInfo< &MetadataFor<commands_filter::InitializationConfiguration::AlignmentSelector>::value > { using type = commands_filter::InitializationConfiguration::AlignmentSelector; };

template<>
struct MetadataFor<commands_filter::InitializationConfiguration::InitialConditionSource>
{
    using type = commands_filter::InitializationConfiguration::InitialConditionSource;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "AUTO_POS_VEL_ATT", "Automatic position, velocity and attitude" },
        { uint32_t(1), "AUTO_POS_VEL_PITCH_ROLL", "Automatic position and velocity, automatic pitch and roll, and user-specified heading" },
        { uint32_t(2), "AUTO_POS_VEL", "Automatic position and velocity, with fully user-specified attitude" },
        { uint32_t(3), "MANUAL", "User-specified position, velocity, and attitude." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "InitialConditionSource",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::InitializationConfiguration::InitialConditionSource>::value > { using type = commands_filter::InitializationConfiguration::InitialConditionSource; };

template<>
struct MetadataFor<commands_filter::InitializationConfiguration::Response>
{
    using type = commands_filter::InitializationConfiguration::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        commands_filter::InitializationConfiguration::InitialConditionSource,
        commands_filter::InitializationConfiguration::AlignmentSelector,
        float,
        float,
        float,
        Vector3f,
        Vector3f,
        commands_filter::FilterReferenceFrame
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.wait_for_run_command;
        if constexpr(I == 1) return value_.initial_cond_src;
        if constexpr(I == 2) return value_.auto_heading_alignment_selector;
        if constexpr(I == 3) return value_.initial_heading;
        if constexpr(I == 4) return value_.initial_pitch;
        if constexpr(I == 5) return value_.initial_roll;
        if constexpr(I == 6) return value_.initial_position;
        if constexpr(I == 7) return value_.initial_velocity;
        if constexpr(I == 8) return value_.reference_frame_selector;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "wait_for_run_command",
            /* .docs          = */ "Initialize filter only after receiving 'run' command",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::wait_for_run_command>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_cond_src",
            /* .docs          = */ "Initial condition source:",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::InitializationConfiguration::InitialConditionSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::InitializationConfiguration::InitialConditionSource, &type::initial_cond_src>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "auto_heading_alignment_selector",
            /* .docs          = */ "Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_filter::InitializationConfiguration::AlignmentSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::InitializationConfiguration::AlignmentSelector, &type::auto_heading_alignment_selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_heading",
            /* .docs          = */ "User-specified initial platform heading (degrees).",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::initial_heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_pitch",
            /* .docs          = */ "User-specified initial platform pitch (degrees)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::initial_pitch>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_roll",
            /* .docs          = */ "User-specified initial platform roll (degrees)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::initial_roll>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_position",
            /* .docs          = */ "User-specified initial platform position (units determined by reference frame selector, see note.)",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::initial_position>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_velocity",
            /* .docs          = */ "User-specified initial platform velocity (units determined by reference frame selector, see note.)",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::initial_velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reference_frame_selector",
            /* .docs          = */ "User-specified initial position/velocity reference frames",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterReferenceFrame>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterReferenceFrame, &type::reference_frame_selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::InitializationConfiguration::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::InitializationConfiguration::Response>::value > { using type = commands_filter::InitializationConfiguration::Response; };

template<>
struct MetadataFor<commands_filter::InitializationConfiguration>
{
    using type = commands_filter::InitializationConfiguration;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        commands_filter::InitializationConfiguration::InitialConditionSource,
        commands_filter::InitializationConfiguration::AlignmentSelector,
        float,
        float,
        float,
        Vector3f,
        Vector3f,
        commands_filter::FilterReferenceFrame
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.wait_for_run_command;
        if constexpr(I == 2) return value_.initial_cond_src;
        if constexpr(I == 3) return value_.auto_heading_alignment_selector;
        if constexpr(I == 4) return value_.initial_heading;
        if constexpr(I == 5) return value_.initial_pitch;
        if constexpr(I == 6) return value_.initial_roll;
        if constexpr(I == 7) return value_.initial_position;
        if constexpr(I == 8) return value_.initial_velocity;
        if constexpr(I == 9) return value_.reference_frame_selector;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "wait_for_run_command",
            /* .docs          = */ "Initialize filter only after receiving 'run' command",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::wait_for_run_command>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_cond_src",
            /* .docs          = */ "Initial condition source:",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::InitializationConfiguration::InitialConditionSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::InitializationConfiguration::InitialConditionSource, &type::initial_cond_src>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "auto_heading_alignment_selector",
            /* .docs          = */ "Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_filter::InitializationConfiguration::AlignmentSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::InitializationConfiguration::AlignmentSelector, &type::auto_heading_alignment_selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_heading",
            /* .docs          = */ "User-specified initial platform heading (degrees).",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::initial_heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_pitch",
            /* .docs          = */ "User-specified initial platform pitch (degrees)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::initial_pitch>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_roll",
            /* .docs          = */ "User-specified initial platform roll (degrees)",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::initial_roll>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_position",
            /* .docs          = */ "User-specified initial platform position (units determined by reference frame selector, see note.)",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::initial_position>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "initial_velocity",
            /* .docs          = */ "User-specified initial platform velocity (units determined by reference frame selector, see note.)",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::initial_velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reference_frame_selector",
            /* .docs          = */ "User-specified initial position/velocity reference frames",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterReferenceFrame>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterReferenceFrame, &type::reference_frame_selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::InitializationConfiguration",
            /* .title       = */ "Navigation Filter Initialization",
            /* .docs        = */ "Controls the source and values used for initial conditions of the navigation solution.\n\nNotes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.\nFor the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.\nFor the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::InitializationConfiguration>::value > { using type = commands_filter::InitializationConfiguration; };
template<> struct TypeForDescriptor<commands_filter::InitializationConfiguration::DESCRIPTOR.as_u16()> { using type = commands_filter::InitializationConfiguration; };

template<>
struct MetadataFor<commands_filter::AdaptiveFilterOptions::Response>
{
    using type = commands_filter::AdaptiveFilterOptions::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        uint16_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.level;
        if constexpr(I == 1) return value_.time_limit;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "level",
            /* .docs          = */ "Auto-adaptive operating level: <br/>\n0=Off, <br/>\n1=Conservative, <br/>\n2=Moderate (default), <br/>\n3=Aggressive.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::level>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_limit",
            /* .docs          = */ "Maximum duration of measurement rejection before entering recovery mode    (ms)",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::time_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AdaptiveFilterOptions::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AdaptiveFilterOptions::Response>::value > { using type = commands_filter::AdaptiveFilterOptions::Response; };

template<>
struct MetadataFor<commands_filter::AdaptiveFilterOptions>
{
    using type = commands_filter::AdaptiveFilterOptions;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        uint16_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.level;
        if constexpr(I == 2) return value_.time_limit;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "level",
            /* .docs          = */ "Auto-adaptive operating level: <br/>\n0=Off, <br/>\n1=Conservative, <br/>\n2=Moderate (default), <br/>\n3=Aggressive.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::level>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_limit",
            /* .docs          = */ "Maximum duration of measurement rejection before entering recovery mode    (ms)",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::time_limit>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::AdaptiveFilterOptions",
            /* .title       = */ "Adaptive Filter Control",
            /* .docs        = */ "Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::AdaptiveFilterOptions>::value > { using type = commands_filter::AdaptiveFilterOptions; };
template<> struct TypeForDescriptor<commands_filter::AdaptiveFilterOptions::DESCRIPTOR.as_u16()> { using type = commands_filter::AdaptiveFilterOptions; };

template<>
struct MetadataFor<commands_filter::MultiAntennaOffset::Response>
{
    using type = commands_filter::MultiAntennaOffset::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.receiver_id;
        if constexpr(I == 1) return value_.antenna_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "antenna_offset",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::antenna_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MultiAntennaOffset::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MultiAntennaOffset::Response>::value > { using type = commands_filter::MultiAntennaOffset::Response; };

template<>
struct MetadataFor<commands_filter::MultiAntennaOffset>
{
    using type = commands_filter::MultiAntennaOffset;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.receiver_id;
        if constexpr(I == 2) return value_.antenna_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "Receiver: 1, 2, etc...",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "antenna_offset",
            /* .docs          = */ "Antenna lever arm offset vector in the vehicle frame (m)",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::antenna_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::MultiAntennaOffset",
            /* .title       = */ "GNSS Multi-Antenna Offset Control",
            /* .docs        = */ "Set the antenna lever arm.\n\nThis command works with devices that utilize multiple antennas.\n<br/><br/><b>Offset Limit</b>: 10 m magnitude (default)",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::MultiAntennaOffset>::value > { using type = commands_filter::MultiAntennaOffset; };
template<> struct TypeForDescriptor<commands_filter::MultiAntennaOffset::DESCRIPTOR.as_u16()> { using type = commands_filter::MultiAntennaOffset; };

template<>
struct MetadataFor<commands_filter::RelPosConfiguration::Response>
{
    using type = commands_filter::RelPosConfiguration::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        commands_filter::FilterReferenceFrame,
        Vector3d
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
        if constexpr(I == 1) return value_.reference_frame_selector;
        if constexpr(I == 2) return value_.reference_coordinates;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "0 - auto (RTK base station), 1 - manual",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reference_frame_selector",
            /* .docs          = */ "ECEF or LLH",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterReferenceFrame>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterReferenceFrame, &type::reference_frame_selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reference_coordinates",
            /* .docs          = */ "reference coordinates, units determined by source selection",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::reference_coordinates>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::RelPosConfiguration::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::RelPosConfiguration::Response>::value > { using type = commands_filter::RelPosConfiguration::Response; };

template<>
struct MetadataFor<commands_filter::RelPosConfiguration>
{
    using type = commands_filter::RelPosConfiguration;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        commands_filter::FilterReferenceFrame,
        Vector3d
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
        if constexpr(I == 2) return value_.reference_frame_selector;
        if constexpr(I == 3) return value_.reference_coordinates;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "0 - auto (RTK base station), 1 - manual",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reference_frame_selector",
            /* .docs          = */ "ECEF or LLH",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::FilterReferenceFrame>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::FilterReferenceFrame, &type::reference_frame_selector>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reference_coordinates",
            /* .docs          = */ "reference coordinates, units determined by source selection",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::reference_coordinates>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::RelPosConfiguration",
            /* .title       = */ "Relative Position Configuration",
            /* .docs        = */ "Configure the reference location for filter relative positioning outputs",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::RelPosConfiguration>::value > { using type = commands_filter::RelPosConfiguration; };
template<> struct TypeForDescriptor<commands_filter::RelPosConfiguration::DESCRIPTOR.as_u16()> { using type = commands_filter::RelPosConfiguration; };

template<>
struct MetadataFor<commands_filter::RefPointLeverArm::ReferencePointSelector>
{
    using type = commands_filter::RefPointLeverArm::ReferencePointSelector;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "VEH", "Defines the origin of the vehicle" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "ReferencePointSelector",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_filter::RefPointLeverArm::ReferencePointSelector>::value > { using type = commands_filter::RefPointLeverArm::ReferencePointSelector; };

template<>
struct MetadataFor<commands_filter::RefPointLeverArm::Response>
{
    using type = commands_filter::RefPointLeverArm::Response;

    using ParamTypes = std::tuple<
        commands_filter::RefPointLeverArm::ReferencePointSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.ref_point_sel;
        if constexpr(I == 1) return value_.lever_arm_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "ref_point_sel",
            /* .docs          = */ "Reserved, must be 1",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::RefPointLeverArm::ReferencePointSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::RefPointLeverArm::ReferencePointSelector, &type::ref_point_sel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lever_arm_offset",
            /* .docs          = */ "[m] Lever arm offset vector in the vehicle's reference frame.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::lever_arm_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::RefPointLeverArm::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::RefPointLeverArm::Response>::value > { using type = commands_filter::RefPointLeverArm::Response; };

template<>
struct MetadataFor<commands_filter::RefPointLeverArm>
{
    using type = commands_filter::RefPointLeverArm;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_filter::RefPointLeverArm::ReferencePointSelector,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.ref_point_sel;
        if constexpr(I == 2) return value_.lever_arm_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "ref_point_sel",
            /* .docs          = */ "Reserved, must be 1",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_filter::RefPointLeverArm::ReferencePointSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_filter::RefPointLeverArm::ReferencePointSelector, &type::ref_point_sel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lever_arm_offset",
            /* .docs          = */ "[m] Lever arm offset vector in the vehicle's reference frame.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::lever_arm_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::RefPointLeverArm",
            /* .title       = */ "Reference point lever arm",
            /* .docs        = */ "Lever arm offset with respect to the sensor for the indicated point of reference.\nThis is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.\nChanging this setting from default will result in a global position offset that depends on vehicle attitude,\nand a velocity offset that depends on vehicle attitude and angular rate.\n<br/>The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.\n<br/><br/>Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)\n<br/><br/><b>Offset Limits</b>\n<br/>Reference Point VEH (1): 10 m magnitude (default)",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::RefPointLeverArm>::value > { using type = commands_filter::RefPointLeverArm; };
template<> struct TypeForDescriptor<commands_filter::RefPointLeverArm::DESCRIPTOR.as_u16()> { using type = commands_filter::RefPointLeverArm; };

template<>
struct MetadataFor<commands_filter::SpeedMeasurement>
{
    using type = commands_filter::SpeedMeasurement;

    using ParamTypes = std::tuple<
        uint8_t,
        float,
        float,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
        if constexpr(I == 1) return value_.time_of_week;
        if constexpr(I == 2) return value_.speed;
        if constexpr(I == 3) return value_.speed_uncertainty;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "Reserved, must be 1.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "GPS time of week when speed was sampled",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "speed",
            /* .docs          = */ "Estimated speed along vehicle's x-axis (may be positive or negative) [meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::speed>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "speed_uncertainty",
            /* .docs          = */ "Estimated uncertainty in the speed measurement (1-sigma value) [meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::speed_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SpeedMeasurement",
            /* .title       = */ "Input speed measurement",
            /* .docs        = */ "Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.\nCan be used by an external odometer/speedometer, for example.\nThis command cannot be used if the internal odometer is configured.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SpeedMeasurement>::value > { using type = commands_filter::SpeedMeasurement; };
template<> struct TypeForDescriptor<commands_filter::SpeedMeasurement::DESCRIPTOR.as_u16()> { using type = commands_filter::SpeedMeasurement; };

template<>
struct MetadataFor<commands_filter::SpeedLeverArm::Response>
{
    using type = commands_filter::SpeedLeverArm::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.source;
        if constexpr(I == 1) return value_.lever_arm_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "source",
            /* .docs          = */ "Reserved, must be 1.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::source>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lever_arm_offset",
            /* .docs          = */ "[m] Lever arm offset vector in the vehicle's reference frame.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::lever_arm_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SpeedLeverArm::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SpeedLeverArm::Response>::value > { using type = commands_filter::SpeedLeverArm::Response; };

template<>
struct MetadataFor<commands_filter::SpeedLeverArm>
{
    using type = commands_filter::SpeedLeverArm;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        Vector3f
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.source;
        if constexpr(I == 2) return value_.lever_arm_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "source",
            /* .docs          = */ "Reserved, must be 1.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::source>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lever_arm_offset",
            /* .docs          = */ "[m] Lever arm offset vector in the vehicle's reference frame.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::lever_arm_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SpeedLeverArm",
            /* .title       = */ "Measurement speed lever arm",
            /* .docs        = */ "Lever arm offset for speed measurements.\nThis is used to compensate for an off-center measurement point\nhaving a different speed due to rotation of the vehicle.\nThe typical use case for this would be an odometer attached to a wheel\non a standard 4-wheeled vehicle. If the odometer is on the left wheel,\nit will report higher speed on right turns and lower speed on left turns.\nThis is because the outside edge of the curve is longer than the inside edge.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SpeedLeverArm>::value > { using type = commands_filter::SpeedLeverArm; };
template<> struct TypeForDescriptor<commands_filter::SpeedLeverArm::DESCRIPTOR.as_u16()> { using type = commands_filter::SpeedLeverArm; };

template<>
struct MetadataFor<commands_filter::WheeledVehicleConstraintControl::Response>
{
    using type = commands_filter::WheeledVehicleConstraintControl::Response;

    using ParamTypes = std::tuple<
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::WheeledVehicleConstraintControl::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::WheeledVehicleConstraintControl::Response>::value > { using type = commands_filter::WheeledVehicleConstraintControl::Response; };

template<>
struct MetadataFor<commands_filter::WheeledVehicleConstraintControl>
{
    using type = commands_filter::WheeledVehicleConstraintControl;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::WheeledVehicleConstraintControl",
            /* .title       = */ "Wheeled Vehicle Constraint Control",
            /* .docs        = */ "Configure the wheeled vehicle kinematic constraint.\n\nWhen enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.\nBy convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in\nany orientation on the vehicle if the appropriate mounting transformation has been specified).\nThis constraint will typically improve heading estimates for vehicles where the assumption is valid, such\nas an automobile, particularly when GNSS coverage is intermittent.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::WheeledVehicleConstraintControl>::value > { using type = commands_filter::WheeledVehicleConstraintControl; };
template<> struct TypeForDescriptor<commands_filter::WheeledVehicleConstraintControl::DESCRIPTOR.as_u16()> { using type = commands_filter::WheeledVehicleConstraintControl; };

template<>
struct MetadataFor<commands_filter::VerticalGyroConstraintControl::Response>
{
    using type = commands_filter::VerticalGyroConstraintControl::Response;

    using ParamTypes = std::tuple<
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::VerticalGyroConstraintControl::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::VerticalGyroConstraintControl::Response>::value > { using type = commands_filter::VerticalGyroConstraintControl::Response; };

template<>
struct MetadataFor<commands_filter::VerticalGyroConstraintControl>
{
    using type = commands_filter::VerticalGyroConstraintControl;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::VerticalGyroConstraintControl",
            /* .title       = */ "Vertical Gyro Constraint Control",
            /* .docs        = */ "Configure the vertical gyro kinematic constraint.\n\nWhen enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch\nand roll under the assumption that the sensor platform is not undergoing linear acceleration.\nThis constraint is useful to maintain accurate pitch and roll during GNSS signal outages.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::VerticalGyroConstraintControl>::value > { using type = commands_filter::VerticalGyroConstraintControl; };
template<> struct TypeForDescriptor<commands_filter::VerticalGyroConstraintControl::DESCRIPTOR.as_u16()> { using type = commands_filter::VerticalGyroConstraintControl; };

template<>
struct MetadataFor<commands_filter::GnssAntennaCalControl::Response>
{
    using type = commands_filter::GnssAntennaCalControl::Response;

    using ParamTypes = std::tuple<
        uint8_t,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.enable;
        if constexpr(I == 1) return value_.max_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "max_offset",
            /* .docs          = */ "Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::max_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GnssAntennaCalControl::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GnssAntennaCalControl::Response>::value > { using type = commands_filter::GnssAntennaCalControl::Response; };

template<>
struct MetadataFor<commands_filter::GnssAntennaCalControl>
{
    using type = commands_filter::GnssAntennaCalControl;

    using ParamTypes = std::tuple<
        FunctionSelector,
        uint8_t,
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.enable;
        if constexpr(I == 2) return value_.max_offset;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disable, 1 - Enable",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "max_offset",
            /* .docs          = */ "Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::max_offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::GnssAntennaCalControl",
            /* .title       = */ "GNSS Antenna Offset Calibration Control",
            /* .docs        = */ "Configure the GNSS antenna lever arm calibration.\n\nWhen enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::GnssAntennaCalControl>::value > { using type = commands_filter::GnssAntennaCalControl; };
template<> struct TypeForDescriptor<commands_filter::GnssAntennaCalControl::DESCRIPTOR.as_u16()> { using type = commands_filter::GnssAntennaCalControl; };

template<>
struct MetadataFor<commands_filter::SetInitialHeading>
{
    using type = commands_filter::SetInitialHeading;

    using ParamTypes = std::tuple<
        float
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.heading;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "heading",
            /* .docs          = */ "Initial heading in radians [-pi, pi]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_filter::SetInitialHeading",
            /* .title       = */ "Set Initial Heading Control",
            /* .docs        = */ "Set the initial heading angle.\n\nThe estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading\nargument will be ignored and the filter will initialize using the inferred magnetic heading.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_filter::SetInitialHeading>::value > { using type = commands_filter::SetInitialHeading; };
template<> struct TypeForDescriptor<commands_filter::SetInitialHeading::DESCRIPTOR.as_u16()> { using type = commands_filter::SetInitialHeading; };


static constexpr inline const FieldInfo* COMMANDS_FILTER_FIELDS[] = {
    &MetadataFor<commands_filter::Reset>::value,
    &MetadataFor<commands_filter::SetInitialAttitude>::value,
    &MetadataFor<commands_filter::SetInitialHeading>::value,
    &MetadataFor<commands_filter::Run>::value,
    &MetadataFor<commands_filter::VehicleDynamicsMode>::value,
    &MetadataFor<commands_filter::SensorToVehicleRotationEuler>::value,
    &MetadataFor<commands_filter::SensorToVehicleOffset>::value,
    &MetadataFor<commands_filter::AntennaOffset>::value,
    &MetadataFor<commands_filter::EstimationControl>::value,
    &MetadataFor<commands_filter::GnssSource>::value,
    &MetadataFor<commands_filter::ExternalGnssUpdate>::value,
    &MetadataFor<commands_filter::ExternalHeadingUpdate>::value,
    &MetadataFor<commands_filter::HeadingSource>::value,
    &MetadataFor<commands_filter::AutoInitControl>::value,
    &MetadataFor<commands_filter::AccelNoise>::value,
    &MetadataFor<commands_filter::GyroNoise>::value,
    &MetadataFor<commands_filter::AccelBiasModel>::value,
    &MetadataFor<commands_filter::GyroBiasModel>::value,
    &MetadataFor<commands_filter::AutoZupt>::value,
    &MetadataFor<commands_filter::ExternalHeadingUpdateWithTime>::value,
    &MetadataFor<commands_filter::AutoAngularZupt>::value,
    &MetadataFor<commands_filter::TareOrientation>::value,
    &MetadataFor<commands_filter::CommandedZupt>::value,
    &MetadataFor<commands_filter::CommandedAngularZupt>::value,
    &MetadataFor<commands_filter::ReferencePosition>::value,
    &MetadataFor<commands_filter::MagCaptureAutoCal>::value,
    &MetadataFor<commands_filter::GravityNoise>::value,
    &MetadataFor<commands_filter::PressureAltitudeNoise>::value,
    &MetadataFor<commands_filter::HardIronOffsetNoise>::value,
    &MetadataFor<commands_filter::SoftIronMatrixNoise>::value,
    &MetadataFor<commands_filter::MagNoise>::value,
    &MetadataFor<commands_filter::MagneticDeclinationSource>::value,
    &MetadataFor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement>::value,
    &MetadataFor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement>::value,
    &MetadataFor<commands_filter::MagDipAngleErrorAdaptiveMeasurement>::value,
    &MetadataFor<commands_filter::AltitudeAiding>::value,
    &MetadataFor<commands_filter::PitchRollAiding>::value,
    &MetadataFor<commands_filter::InclinationSource>::value,
    &MetadataFor<commands_filter::MagFieldMagnitudeSource>::value,
    &MetadataFor<commands_filter::SensorToVehicleRotationDcm>::value,
    &MetadataFor<commands_filter::SensorToVehicleRotationQuaternion>::value,
    &MetadataFor<commands_filter::AidingMeasurementEnable>::value,
    &MetadataFor<commands_filter::KinematicConstraint>::value,
    &MetadataFor<commands_filter::InitializationConfiguration>::value,
    &MetadataFor<commands_filter::AdaptiveFilterOptions>::value,
    &MetadataFor<commands_filter::MultiAntennaOffset>::value,
    &MetadataFor<commands_filter::RelPosConfiguration>::value,
    &MetadataFor<commands_filter::RefPointLeverArm>::value,
    &MetadataFor<commands_filter::SpeedMeasurement>::value,
    &MetadataFor<commands_filter::SpeedLeverArm>::value,
    &MetadataFor<commands_filter::VerticalGyroConstraintControl>::value,
    &MetadataFor<commands_filter::WheeledVehicleConstraintControl>::value,
    &MetadataFor<commands_filter::GnssAntennaCalControl>::value,
    &MetadataFor<commands_filter::VehicleDynamicsMode::Response>::value,
    &MetadataFor<commands_filter::SensorToVehicleRotationEuler::Response>::value,
    &MetadataFor<commands_filter::SensorToVehicleOffset::Response>::value,
    &MetadataFor<commands_filter::AntennaOffset::Response>::value,
    &MetadataFor<commands_filter::EstimationControl::Response>::value,
    &MetadataFor<commands_filter::GnssSource::Response>::value,
    &MetadataFor<commands_filter::HeadingSource::Response>::value,
    &MetadataFor<commands_filter::AutoInitControl::Response>::value,
    &MetadataFor<commands_filter::AccelNoise::Response>::value,
    &MetadataFor<commands_filter::GyroNoise::Response>::value,
    &MetadataFor<commands_filter::AccelBiasModel::Response>::value,
    &MetadataFor<commands_filter::GyroBiasModel::Response>::value,
    &MetadataFor<commands_filter::AutoZupt::Response>::value,
    &MetadataFor<commands_filter::AutoAngularZupt::Response>::value,
    &MetadataFor<commands_filter::ReferencePosition::Response>::value,
    &MetadataFor<commands_filter::GravityNoise::Response>::value,
    &MetadataFor<commands_filter::PressureAltitudeNoise::Response>::value,
    &MetadataFor<commands_filter::HardIronOffsetNoise::Response>::value,
    &MetadataFor<commands_filter::SoftIronMatrixNoise::Response>::value,
    &MetadataFor<commands_filter::TareOrientation::Response>::value,
    &MetadataFor<commands_filter::MagNoise::Response>::value,
    &MetadataFor<commands_filter::MagneticDeclinationSource::Response>::value,
    &MetadataFor<commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response>::value,
    &MetadataFor<commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response>::value,
    &MetadataFor<commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response>::value,
    &MetadataFor<commands_filter::AltitudeAiding::Response>::value,
    &MetadataFor<commands_filter::PitchRollAiding::Response>::value,
    &MetadataFor<commands_filter::InclinationSource::Response>::value,
    &MetadataFor<commands_filter::MagFieldMagnitudeSource::Response>::value,
    &MetadataFor<commands_filter::SensorToVehicleRotationDcm::Response>::value,
    &MetadataFor<commands_filter::SensorToVehicleRotationQuaternion::Response>::value,
    &MetadataFor<commands_filter::AidingMeasurementEnable::Response>::value,
    &MetadataFor<commands_filter::KinematicConstraint::Response>::value,
    &MetadataFor<commands_filter::InitializationConfiguration::Response>::value,
    &MetadataFor<commands_filter::AdaptiveFilterOptions::Response>::value,
    &MetadataFor<commands_filter::MultiAntennaOffset::Response>::value,
    &MetadataFor<commands_filter::RelPosConfiguration::Response>::value,
    &MetadataFor<commands_filter::RefPointLeverArm::Response>::value,
    &MetadataFor<commands_filter::SpeedLeverArm::Response>::value,
    &MetadataFor<commands_filter::VerticalGyroConstraintControl::Response>::value,
    &MetadataFor<commands_filter::WheeledVehicleConstraintControl::Response>::value,
    &MetadataFor<commands_filter::GnssAntennaCalControl::Response>::value,
};

//namespace commands_filter
//{
struct CommandSetFilter
{
    static inline constexpr uint8_t DESCRIPTOR_SET = commands_filter::DESCRIPTOR_SET;
    static inline constexpr CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, INVALID_FIELD_DESCRIPTOR};

    using Fields = std::tuple<
        ::mip::commands_filter::Reset,
        ::mip::commands_filter::SetInitialAttitude,
        ::mip::commands_filter::SetInitialHeading,
        ::mip::commands_filter::Run,
        ::mip::commands_filter::VehicleDynamicsMode,
        ::mip::commands_filter::SensorToVehicleRotationEuler,
        ::mip::commands_filter::SensorToVehicleOffset,
        ::mip::commands_filter::AntennaOffset,
        ::mip::commands_filter::EstimationControl,
        ::mip::commands_filter::GnssSource,
        ::mip::commands_filter::ExternalGnssUpdate,
        ::mip::commands_filter::ExternalHeadingUpdate,
        ::mip::commands_filter::HeadingSource,
        ::mip::commands_filter::AutoInitControl,
        ::mip::commands_filter::AccelNoise,
        ::mip::commands_filter::GyroNoise,
        ::mip::commands_filter::AccelBiasModel,
        ::mip::commands_filter::GyroBiasModel,
        ::mip::commands_filter::AutoZupt,
        ::mip::commands_filter::ExternalHeadingUpdateWithTime,
        ::mip::commands_filter::AutoAngularZupt,
        ::mip::commands_filter::TareOrientation,
        ::mip::commands_filter::CommandedZupt,
        ::mip::commands_filter::CommandedAngularZupt,
        ::mip::commands_filter::ReferencePosition,
        ::mip::commands_filter::MagCaptureAutoCal,
        ::mip::commands_filter::GravityNoise,
        ::mip::commands_filter::PressureAltitudeNoise,
        ::mip::commands_filter::HardIronOffsetNoise,
        ::mip::commands_filter::SoftIronMatrixNoise,
        ::mip::commands_filter::MagNoise,
        ::mip::commands_filter::MagneticDeclinationSource,
        ::mip::commands_filter::AccelMagnitudeErrorAdaptiveMeasurement,
        ::mip::commands_filter::MagMagnitudeErrorAdaptiveMeasurement,
        ::mip::commands_filter::MagDipAngleErrorAdaptiveMeasurement,
        ::mip::commands_filter::AltitudeAiding,
        ::mip::commands_filter::PitchRollAiding,
        ::mip::commands_filter::InclinationSource,
        ::mip::commands_filter::MagFieldMagnitudeSource,
        ::mip::commands_filter::SensorToVehicleRotationDcm,
        ::mip::commands_filter::SensorToVehicleRotationQuaternion,
        ::mip::commands_filter::AidingMeasurementEnable,
        ::mip::commands_filter::KinematicConstraint,
        ::mip::commands_filter::InitializationConfiguration,
        ::mip::commands_filter::AdaptiveFilterOptions,
        ::mip::commands_filter::MultiAntennaOffset,
        ::mip::commands_filter::RelPosConfiguration,
        ::mip::commands_filter::RefPointLeverArm,
        ::mip::commands_filter::SpeedMeasurement,
        ::mip::commands_filter::SpeedLeverArm,
        ::mip::commands_filter::VerticalGyroConstraintControl,
        ::mip::commands_filter::WheeledVehicleConstraintControl,
        ::mip::commands_filter::GnssAntennaCalControl,
        ::mip::commands_filter::VehicleDynamicsMode::Response,
        ::mip::commands_filter::SensorToVehicleRotationEuler::Response,
        ::mip::commands_filter::SensorToVehicleOffset::Response,
        ::mip::commands_filter::AntennaOffset::Response,
        ::mip::commands_filter::EstimationControl::Response,
        ::mip::commands_filter::GnssSource::Response,
        ::mip::commands_filter::HeadingSource::Response,
        ::mip::commands_filter::AutoInitControl::Response,
        ::mip::commands_filter::AccelNoise::Response,
        ::mip::commands_filter::GyroNoise::Response,
        ::mip::commands_filter::AccelBiasModel::Response,
        ::mip::commands_filter::GyroBiasModel::Response,
        ::mip::commands_filter::AutoZupt::Response,
        ::mip::commands_filter::AutoAngularZupt::Response,
        ::mip::commands_filter::ReferencePosition::Response,
        ::mip::commands_filter::GravityNoise::Response,
        ::mip::commands_filter::PressureAltitudeNoise::Response,
        ::mip::commands_filter::HardIronOffsetNoise::Response,
        ::mip::commands_filter::SoftIronMatrixNoise::Response,
        ::mip::commands_filter::TareOrientation::Response,
        ::mip::commands_filter::MagNoise::Response,
        ::mip::commands_filter::MagneticDeclinationSource::Response,
        ::mip::commands_filter::AccelMagnitudeErrorAdaptiveMeasurement::Response,
        ::mip::commands_filter::MagMagnitudeErrorAdaptiveMeasurement::Response,
        ::mip::commands_filter::MagDipAngleErrorAdaptiveMeasurement::Response,
        ::mip::commands_filter::AltitudeAiding::Response,
        ::mip::commands_filter::PitchRollAiding::Response,
        ::mip::commands_filter::InclinationSource::Response,
        ::mip::commands_filter::MagFieldMagnitudeSource::Response,
        ::mip::commands_filter::SensorToVehicleRotationDcm::Response,
        ::mip::commands_filter::SensorToVehicleRotationQuaternion::Response,
        ::mip::commands_filter::AidingMeasurementEnable::Response,
        ::mip::commands_filter::KinematicConstraint::Response,
        ::mip::commands_filter::InitializationConfiguration::Response,
        ::mip::commands_filter::AdaptiveFilterOptions::Response,
        ::mip::commands_filter::MultiAntennaOffset::Response,
        ::mip::commands_filter::RelPosConfiguration::Response,
        ::mip::commands_filter::RefPointLeverArm::Response,
        ::mip::commands_filter::SpeedLeverArm::Response,
        ::mip::commands_filter::VerticalGyroConstraintControl::Response,
        ::mip::commands_filter::WheeledVehicleConstraintControl::Response,
        ::mip::commands_filter::GnssAntennaCalControl::Response
    >;
};

//} // namespace commands_filter

template<>
struct MetadataFor<CommandSetFilter>
{
    using type = CommandSetFilter;
    
    static inline constexpr DescriptorSetInfo value = {
        /* .descriptor = */ commands_filter::DESCRIPTOR_SET,
        /* .name       = */ "Filter Commands",
        /* .fields     = */ COMMANDS_FILTER_FIELDS,
    };
};
template<> struct TypeForDescriptor< (commands_filter::DESCRIPTOR_SET << 8) > { using type = CommandSetFilter; };

static constexpr DescriptorSetInfo COMMANDS_FILTER = {
    /* .descriptor = */ mip::commands_filter::DESCRIPTOR_SET,
    /* .name       = */ "Filter Commands",
    /* .fields     = */ COMMANDS_FILTER_FIELDS,
};

} // namespace mip::metadata

