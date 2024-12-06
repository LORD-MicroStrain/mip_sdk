#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/data_filter.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<data_filter::PositionLlh>
{
    using type = data_filter::PositionLlh;

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
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - Invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::PositionLlh",
        /* .title       = */ "LLH Position",
        /* .docs        = */ "Filter reported position in the WGS84 geodetic frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::VelocityNed>
{
    using type = data_filter::VelocityNed;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "north",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::north>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "east",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::east>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "down",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::down>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - Invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::VelocityNed",
        /* .title       = */ "velocity_ned",
        /* .docs        = */ "Filter reported velocity in the NED local-level frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AttitudeQuaternion>
{
    using type = data_filter::AttitudeQuaternion;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "q",
            /* .docs          = */ "Quaternion elements EQSTART q = (q_w, q_x, q_y, q_z) EQEND",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Quatf, &type::q>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AttitudeQuaternion",
        /* .title       = */ "attitude_quaternion",
        /* .docs        = */ "4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.\nThis quaternion satisfies the following relationship:\n\nEQSTART p^{veh} = q^{-1} p^{ned} q EQEND<br/>\n\nWhere:<br/>\nEQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the rotation. <br/>\nEQSTART p^ned = (0, v^{ned}_x, v^{ned}_y, v^{ned}_z) EQEND and EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame.<br/>\nEQSTART p^veh = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AttitudeDcm>
{
    using type = data_filter::AttitudeDcm;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "dcm",
            /* .docs          = */ "Matrix elements in row-major order.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::dcm>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AttitudeDcm",
        /* .title       = */ "attitude_dcm",
        /* .docs        = */ "3x3 Direction Cosine Matrix EQSTART M_{ned}^{veh} EQEND describing the orientation of the device with respect to the NED local-level frame.\nThis matrix satisfies the following relationship:\n\nEQSTART v^{veh} = M_{ned}^{veh} v^{ned} EQEND<br/>\n\nWhere:<br/>\n\nEQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame. <br/>\nEQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>\n<br/>\nThe matrix elements are stored is row-major order: EQSTART M_{ned}^{veh} = \\begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \\end{bmatrix} EQEND",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::EulerAngles>
{
    using type = data_filter::EulerAngles;

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
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::EulerAngles",
        /* .title       = */ "euler_angles",
        /* .docs        = */ "Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.\nThe Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GyroBias>
{
    using type = data_filter::GyroBias;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias",
            /* .docs          = */ "(x, y, z) [radians/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::bias>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GyroBias",
        /* .title       = */ "gyro_bias",
        /* .docs        = */ "Filter reported gyro bias expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AccelBias>
{
    using type = data_filter::AccelBias;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias",
            /* .docs          = */ "(x, y, z) [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::bias>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AccelBias",
        /* .title       = */ "accel_bias",
        /* .docs        = */ "Filter reported accelerometer bias expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::PositionLlhUncertainty>
{
    using type = data_filter::PositionLlhUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "north",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::north>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "east",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::east>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "down",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::down>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::PositionLlhUncertainty",
        /* .title       = */ "LLH Position Uncertainty",
        /* .docs        = */ "Filter reported 1-sigma position uncertainty in the NED local-level frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::VelocityNedUncertainty>
{
    using type = data_filter::VelocityNedUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "north",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::north>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "east",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::east>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "down",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::down>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::VelocityNedUncertainty",
        /* .title       = */ "NED Velocity Uncertainty",
        /* .docs        = */ "Filter reported 1-sigma velocity uncertainties in the NED local-level frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::EulerAnglesUncertainty>
{
    using type = data_filter::EulerAnglesUncertainty;

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
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::EulerAnglesUncertainty",
        /* .title       = */ "euler_angles_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma Euler angle uncertainties.\nThe uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GyroBiasUncertainty>
{
    using type = data_filter::GyroBiasUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias_uncert",
            /* .docs          = */ "(x,y,z) [radians/sec]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::bias_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GyroBiasUncertainty",
        /* .title       = */ "gyro_bias_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AccelBiasUncertainty>
{
    using type = data_filter::AccelBiasUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias_uncert",
            /* .docs          = */ "(x,y,z) [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::bias_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AccelBiasUncertainty",
        /* .title       = */ "accel_bias_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::Timestamp>
{
    using type = data_filter::Timestamp;

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
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::Timestamp",
        /* .title       = */ "timestamp",
        /* .docs        = */ "GPS timestamp of the Filter data\n\nShould the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.\nUpon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.\nIf synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.\n\nNote: this data field may be deprecated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::FilterMode>
{
    using type = data_filter::FilterMode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "GX5_STARTUP", "" },
        { uint32_t(1), "GX5_INIT", "" },
        { uint32_t(2), "GX5_RUN_SOLUTION_VALID", "" },
        { uint32_t(3), "GX5_RUN_SOLUTION_ERROR", "" },
        { uint32_t(1), "INIT", "" },
        { uint32_t(2), "VERT_GYRO", "" },
        { uint32_t(3), "AHRS", "" },
        { uint32_t(4), "FULL_NAV", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FilterMode",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::FilterDynamicsMode>
{
    using type = data_filter::FilterDynamicsMode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "GX5_PORTABLE", "" },
        { uint32_t(2), "GX5_AUTOMOTIVE", "" },
        { uint32_t(3), "GX5_AIRBORNE", "" },
        { uint32_t(1), "GQ7_DEFAULT", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FilterDynamicsMode",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::FilterStatusFlags>
{
    using type = data_filter::FilterStatusFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(4096), "gx5_init_no_attitude", "" },
        { uint32_t(8192), "gx5_init_no_position_velocity", "" },
        { uint32_t(1), "gx5_run_imu_unavailable", "" },
        { uint32_t(2), "gx5_run_gps_unavailable", "" },
        { uint32_t(8), "gx5_run_matrix_singularity", "" },
        { uint32_t(16), "gx5_run_position_covariance_warning", "" },
        { uint32_t(32), "gx5_run_velocity_covariance_warning", "" },
        { uint32_t(64), "gx5_run_attitude_covariance_warning", "" },
        { uint32_t(128), "gx5_run_nan_in_solution_warning", "" },
        { uint32_t(256), "gx5_run_gyro_bias_est_high_warning", "" },
        { uint32_t(512), "gx5_run_accel_bias_est_high_warning", "" },
        { uint32_t(1024), "gx5_run_gyro_scale_factor_est_high_warning", "" },
        { uint32_t(2048), "gx5_run_accel_scale_factor_est_high_warning", "" },
        { uint32_t(4096), "gx5_run_mag_bias_est_high_warning", "" },
        { uint32_t(8192), "gx5_run_ant_offset_correction_est_high_warning", "" },
        { uint32_t(16384), "gx5_run_mag_hard_iron_est_high_warning", "" },
        { uint32_t(32768), "gx5_run_mag_soft_iron_est_high_warning", "" },
        { uint32_t(3), "gq7_filter_condition", "" },
        { uint32_t(4), "gq7_roll_pitch_warning", "" },
        { uint32_t(8), "gq7_heading_warning", "" },
        { uint32_t(16), "gq7_position_warning", "" },
        { uint32_t(32), "gq7_velocity_warning", "" },
        { uint32_t(64), "gq7_imu_bias_warning", "" },
        { uint32_t(128), "gq7_gnss_clk_warning", "" },
        { uint32_t(256), "gq7_antenna_lever_arm_warning", "" },
        { uint32_t(512), "gq7_mounting_transform_warning", "" },
        { uint32_t(1024), "gq7_time_sync_warning", "No time synchronization pulse detected" },
        { uint32_t(61440), "gq7_solution_error", "Filter computation warning flags. If any bits 12-15 are set, and all filter outputs will be invalid." },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "FilterStatusFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::Status>
{
    using type = data_filter::Status;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "filter_state",
            /* .docs          = */ "Device-specific filter state.  Please consult the user manual for definition.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_filter::FilterMode>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::FilterMode, &type::filter_state>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "dynamics_mode",
            /* .docs          = */ "Device-specific dynamics mode. Please consult the user manual for definition.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_filter::FilterDynamicsMode>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::FilterDynamicsMode, &type::dynamics_mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "status_flags",
            /* .docs          = */ "Device-specific status flags.  Please consult the user manual for definition.",
            /* .type          = */ {Type::BITS, &MetadataFor<data_filter::FilterStatusFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::FilterStatusFlags, &type::status_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::Status",
        /* .title       = */ "status",
        /* .docs        = */ "Device-specific filter status indicators.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::LinearAccel>
{
    using type = data_filter::LinearAccel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "accel",
            /* .docs          = */ "(x,y,z) [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::accel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::LinearAccel",
        /* .title       = */ "linear_accel",
        /* .docs        = */ "Filter-compensated linear acceleration expressed in the vehicle frame.\nNote: The estimated gravity has been removed from this data leaving only linear acceleration.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GravityVector>
{
    using type = data_filter::GravityVector;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gravity",
            /* .docs          = */ "(x, y, z) [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::gravity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GravityVector",
        /* .title       = */ "gravity_vector",
        /* .docs        = */ "Filter reported gravity vector expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::CompAccel>
{
    using type = data_filter::CompAccel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "accel",
            /* .docs          = */ "(x,y,z) [meters/second^2]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::accel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::CompAccel",
        /* .title       = */ "Compensated Acceleration",
        /* .docs        = */ "Filter-compensated acceleration expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::CompAngularRate>
{
    using type = data_filter::CompAngularRate;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gyro",
            /* .docs          = */ "(x, y, z) [radians/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::gyro>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::CompAngularRate",
        /* .title       = */ "comp_angular_rate",
        /* .docs        = */ "Filter-compensated angular rate expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::QuaternionAttitudeUncertainty>
{
    using type = data_filter::QuaternionAttitudeUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "q",
            /* .docs          = */ "[dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Quatf, &type::q>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::QuaternionAttitudeUncertainty",
        /* .title       = */ "quaternion_attitude_uncertainty",
        /* .docs        = */ "Filter reported quaternion uncertainties.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::Wgs84GravityMag>
{
    using type = data_filter::Wgs84GravityMag;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "magnitude",
            /* .docs          = */ "[meters/second^2]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::magnitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::Wgs84GravityMag",
        /* .title       = */ "wgs84_gravity_mag",
        /* .docs        = */ "Filter reported WGS84 gravity magnitude.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::HeadingUpdateState::HeadingSource>
{
    using type = data_filter::HeadingUpdateState::HeadingSource;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "NONE", "" },
        { uint32_t(1), "MAGNETOMETER", "" },
        { uint32_t(2), "GNSS_VELOCITY_VECTOR", "" },
        { uint32_t(4), "EXTERNAL", "" },
        { uint32_t(8), "DUAL_ANTENNA", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "HeadingSource",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::HeadingUpdateState>
{
    using type = data_filter::HeadingUpdateState;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "heading",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "heading_1sigma",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading_1sigma>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "source",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_filter::HeadingUpdateState::HeadingSource>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::HeadingUpdateState::HeadingSource, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "1 if a valid heading update was received in 2 seconds, 0 otherwise.",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::HeadingUpdateState",
        /* .title       = */ "heading_update_state",
        /* .docs        = */ "Filter reported heading update state.\n\nHeading updates can be applied from the sources listed below.  Note, some of these sources may be combined.\nThe heading value is always relative to true north.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagneticModel>
{
    using type = data_filter::MagneticModel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "intensity_north",
            /* .docs          = */ "[Gauss]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::intensity_north>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "intensity_east",
            /* .docs          = */ "[Gauss]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::intensity_east>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "intensity_down",
            /* .docs          = */ "[Gauss]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::intensity_down>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "inclination",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::inclination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "declination",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::declination>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagneticModel",
        /* .title       = */ "magnetic_model",
        /* .docs        = */ "The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.\nA valid GNSS location is required for the model to be valid.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AccelScaleFactor>
{
    using type = data_filter::AccelScaleFactor;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scale_factor",
            /* .docs          = */ "(x,y,z) [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scale_factor>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AccelScaleFactor",
        /* .title       = */ "accel_scale_factor",
        /* .docs        = */ "Filter reported accelerometer scale factor expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AccelScaleFactorUncertainty>
{
    using type = data_filter::AccelScaleFactorUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scale_factor_uncert",
            /* .docs          = */ "(x,y,z) [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scale_factor_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AccelScaleFactorUncertainty",
        /* .title       = */ "accel_scale_factor_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GyroScaleFactor>
{
    using type = data_filter::GyroScaleFactor;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scale_factor",
            /* .docs          = */ "(x,y,z) [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scale_factor>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GyroScaleFactor",
        /* .title       = */ "gyro_scale_factor",
        /* .docs        = */ "Filter reported gyro scale factor expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GyroScaleFactorUncertainty>
{
    using type = data_filter::GyroScaleFactorUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scale_factor_uncert",
            /* .docs          = */ "(x,y,z) [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scale_factor_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GyroScaleFactorUncertainty",
        /* .title       = */ "gyro_scale_factor_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagBias>
{
    using type = data_filter::MagBias;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias",
            /* .docs          = */ "(x,y,z) [Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::bias>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagBias",
        /* .title       = */ "mag_bias",
        /* .docs        = */ "Filter reported magnetometer bias expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagBiasUncertainty>
{
    using type = data_filter::MagBiasUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "bias_uncert",
            /* .docs          = */ "(x,y,z) [Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::bias_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagBiasUncertainty",
        /* .title       = */ "mag_bias_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::StandardAtmosphere>
{
    using type = data_filter::StandardAtmosphere;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "geometric_altitude",
            /* .docs          = */ "Input into calculation [meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::geometric_altitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "geopotential_altitude",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::geopotential_altitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "standard_temperature",
            /* .docs          = */ "[degC]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::standard_temperature>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "standard_pressure",
            /* .docs          = */ "[milliBar]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::standard_pressure>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "standard_density",
            /* .docs          = */ "[kilogram/meter^3]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::standard_density>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::StandardAtmosphere",
        /* .title       = */ "standard_atmosphere",
        /* .docs        = */ "Filter reported standard atmosphere parameters.\n\nThe US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::PressureAltitude>
{
    using type = data_filter::PressureAltitude;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "pressure_altitude",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::pressure_altitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::PressureAltitude",
        /* .title       = */ "pressure_altitude",
        /* .docs        = */ "Filter reported pressure altitude.\n\nThe US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.\nA valid pressure sensor reading is required for the pressure altitude to be valid.\nThe minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::DensityAltitude>
{
    using type = data_filter::DensityAltitude;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "density_altitude",
            /* .docs          = */ "m",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::density_altitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::DensityAltitude",
        /* .title       = */ "density_altitude",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AntennaOffsetCorrection>
{
    using type = data_filter::AntennaOffsetCorrection;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "offset",
            /* .docs          = */ "(x,y,z) [meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AntennaOffsetCorrection",
        /* .title       = */ "antenna_offset_correction",
        /* .docs        = */ "Filter reported GNSS antenna offset in vehicle frame.\n\nThis offset added to any previously stored offset vector to compensate for errors in definition.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AntennaOffsetCorrectionUncertainty>
{
    using type = data_filter::AntennaOffsetCorrectionUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "offset_uncert",
            /* .docs          = */ "(x,y,z) [meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AntennaOffsetCorrectionUncertainty",
        /* .title       = */ "antenna_offset_correction_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MultiAntennaOffsetCorrection>
{
    using type = data_filter::MultiAntennaOffsetCorrection;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "Receiver ID for the receiver to which the antenna is attached",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "offset",
            /* .docs          = */ "(x,y,z) [meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MultiAntennaOffsetCorrection",
        /* .title       = */ "multi_antenna_offset_correction",
        /* .docs        = */ "Filter reported GNSS antenna offset in vehicle frame.\n\nThis offset added to any previously stored offset vector to compensate for errors in definition.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MultiAntennaOffsetCorrectionUncertainty>
{
    using type = data_filter::MultiAntennaOffsetCorrectionUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "Receiver ID for the receiver to which the antenna is attached",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "offset_uncert",
            /* .docs          = */ "(x,y,z) [meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::offset_uncert>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MultiAntennaOffsetCorrectionUncertainty",
        /* .title       = */ "multi_antenna_offset_correction_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagnetometerOffset>
{
    using type = data_filter::MagnetometerOffset;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "hard_iron",
            /* .docs          = */ "(x,y,z) [Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::hard_iron>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagnetometerOffset",
        /* .title       = */ "magnetometer_offset",
        /* .docs        = */ "Filter reported magnetometer hard iron offset in sensor frame.\n\nThis offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagnetometerMatrix>
{
    using type = data_filter::MagnetometerMatrix;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "soft_iron",
            /* .docs          = */ "Row-major [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::soft_iron>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagnetometerMatrix",
        /* .title       = */ "magnetometer_matrix",
        /* .docs        = */ "Filter reported magnetometer soft iron matrix in sensor frame.\n\nThis matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagnetometerOffsetUncertainty>
{
    using type = data_filter::MagnetometerOffsetUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "hard_iron_uncertainty",
            /* .docs          = */ "(x,y,z) [Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::hard_iron_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagnetometerOffsetUncertainty",
        /* .title       = */ "magnetometer_offset_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagnetometerMatrixUncertainty>
{
    using type = data_filter::MagnetometerMatrixUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "soft_iron_uncertainty",
            /* .docs          = */ "Row-major [dimensionless]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::soft_iron_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagnetometerMatrixUncertainty",
        /* .title       = */ "magnetometer_matrix_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagnetometerCovarianceMatrix>
{
    using type = data_filter::MagnetometerCovarianceMatrix;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "covariance",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::covariance>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagnetometerCovarianceMatrix",
        /* .title       = */ "magnetometer_covariance_matrix",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::MagnetometerResidualVector>
{
    using type = data_filter::MagnetometerResidualVector;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "residual",
            /* .docs          = */ "(x,y,z) [Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::residual>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::MagnetometerResidualVector",
        /* .title       = */ "magnetometer_residual_vector",
        /* .docs        = */ "Filter reported magnetometer measurement residuals in vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::ClockCorrection>
{
    using type = data_filter::ClockCorrection;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "1, 2, etc.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "bias",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::bias>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "bias_drift",
            /* .docs          = */ "[seconds/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::bias_drift>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::ClockCorrection",
        /* .title       = */ "clock_correction",
        /* .docs        = */ "Filter reported GNSS receiver clock error parameters.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::ClockCorrectionUncertainty>
{
    using type = data_filter::ClockCorrectionUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "1, 2, etc.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "bias_uncertainty",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::bias_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "bias_drift_uncertainty",
            /* .docs          = */ "[seconds/second]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::bias_drift_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::ClockCorrectionUncertainty",
        /* .title       = */ "clock_correction_uncertainty",
        /* .docs        = */ "Filter reported 1-sigma GNSS receiver clock error parameters.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GnssAidStatusFlags>
{
    using type = data_filter::GnssAidStatusFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "tight_coupling", "If 1, the Kalman filter is processing raw range information from this GNSS module" },
        { uint32_t(2), "differential", "If 1, the Kalman filter is processing RTK corrections from this GNSS module" },
        { uint32_t(4), "integer_fix", "If 1, the Kalman filter has an RTK integer fix from this GNSS module, indicating the best position performance possible" },
        { uint32_t(8), "GPS_L1", "If 1, the Kalman filter is using GPS L1 measurements" },
        { uint32_t(16), "GPS_L2", "If 1, the Kalman filter is using GPS L2 measurements" },
        { uint32_t(32), "GPS_L5", "If 1, the Kalman filter is using GPS L5 measurements (not available on the GQ7)" },
        { uint32_t(64), "GLO_L1", "If 1, the Kalman filter is using GLONASS L1 measurements" },
        { uint32_t(128), "GLO_L2", "If 1, the Kalman filter is using GLONASS L2 measurements" },
        { uint32_t(256), "GAL_E1", "If 1, the Kalman filter is using Galileo E1 measurements" },
        { uint32_t(512), "GAL_E5", "If 1, the Kalman filter is using Galileo E5 measurements" },
        { uint32_t(1024), "GAL_E6", "If 1, the Kalman filter is using Galileo E6 measurements" },
        { uint32_t(2048), "BEI_B1", "If 1, the Kalman filter is using Beidou B1 measurements (not enabled on GQ7 currently)" },
        { uint32_t(4096), "BEI_B2", "If 1, the Kalman filter is using Beidou B2 measurements (not enabled on GQ7 currently)" },
        { uint32_t(8192), "BEI_B3", "If 1, the Kalman filter is using Beidou B3 measurements (not available on the GQ7)" },
        { uint32_t(16384), "no_fix", "If 1, this GNSS module is reporting no position fix" },
        { uint32_t(32768), "config_error", "If 1, there is likely an issue with the antenna offset for this GNSS module" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "GnssAidStatusFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::GnssPosAidStatus>
{
    using type = data_filter::GnssPosAidStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "Last GNSS aiding measurement time of week [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "status",
            /* .docs          = */ "Aiding measurement status bitfield",
            /* .type          = */ {Type::BITS, &MetadataFor<data_filter::GnssAidStatusFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::GnssAidStatusFlags, &type::status>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 8,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GnssPosAidStatus",
        /* .title       = */ "GNSS Position Aiding Status",
        /* .docs        = */ "Filter reported GNSS position aiding status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GnssAttAidStatus>
{
    using type = data_filter::GnssAttAidStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "Last valid aiding measurement time of week [seconds] [processed instead of measured?]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "status",
            /* .docs          = */ "Last valid aiding measurement status bitfield",
            /* .type          = */ {Type::BITS, &MetadataFor<data_filter::GnssAidStatusFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::GnssAidStatusFlags, &type::status>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 8,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GnssAttAidStatus",
        /* .title       = */ "GNSS Attitude Aiding Status",
        /* .docs        = */ "Filter reported dual antenna GNSS attitude aiding status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::HeadAidStatus::HeadingAidType>
{
    using type = data_filter::HeadAidStatus::HeadingAidType;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "DUAL_ANTENNA", "" },
        { uint32_t(2), "EXTERNAL_MESSAGE", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "HeadingAidType",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::HeadAidStatus>
{
    using type = data_filter::HeadAidStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "Last valid aiding measurement time of week [seconds] [processed instead of measured?]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "type",
            /* .docs          = */ "1 - Dual antenna, 2 - External heading message (user supplied)",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_filter::HeadAidStatus::HeadingAidType>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::HeadAidStatus::HeadingAidType, &type::type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 2,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::HeadAidStatus",
        /* .title       = */ "head_aid_status",
        /* .docs        = */ "Filter reported GNSS heading aiding status",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::RelPosNed>
{
    using type = data_filter::RelPosNed;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "relative_position",
            /* .docs          = */ "[meters, NED]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::relative_position>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::RelPosNed",
        /* .title       = */ "NED Relative Position",
        /* .docs        = */ "Filter reported relative position, with respect to configured reference position",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::EcefPos>
{
    using type = data_filter::EcefPos;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "position_ecef",
            /* .docs          = */ "[meters, ECEF]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3d>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3d, &type::position_ecef>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::EcefPos",
        /* .title       = */ "ECEF Position",
        /* .docs        = */ "Filter reported ECEF position",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::EcefVel>
{
    using type = data_filter::EcefVel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "velocity_ecef",
            /* .docs          = */ "[meters/second, ECEF]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::velocity_ecef>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::EcefVel",
        /* .title       = */ "ECEF Velocity",
        /* .docs        = */ "Filter reported ECEF velocity",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::EcefPosUncertainty>
{
    using type = data_filter::EcefPosUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "pos_uncertainty",
            /* .docs          = */ "[meters]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::pos_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::EcefPosUncertainty",
        /* .title       = */ "ECEF Position Uncertainty",
        /* .docs        = */ "Filter reported 1-sigma position uncertainty in the ECEF frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::EcefVelUncertainty>
{
    using type = data_filter::EcefVelUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "vel_uncertainty",
            /* .docs          = */ "[meters/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::vel_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::EcefVelUncertainty",
        /* .title       = */ "ECEF Velocity Uncertainty",
        /* .docs        = */ "Filter reported 1-sigma velocity uncertainties in the ECEF frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::FilterAidingMeasurementType>
{
    using type = data_filter::FilterAidingMeasurementType;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "GNSS", "" },
        { uint32_t(2), "DUAL_ANTENNA", "" },
        { uint32_t(3), "HEADING", "" },
        { uint32_t(4), "PRESSURE", "" },
        { uint32_t(5), "MAGNETOMETER", "" },
        { uint32_t(6), "SPEED", "" },
        { uint32_t(33), "AIDING_POS_ECEF", "" },
        { uint32_t(34), "AIDING_POS_LLH", "" },
        { uint32_t(35), "AIDING_HEIGHT_ABOVE_ELLIPSOID", "" },
        { uint32_t(40), "AIDING_VEL_ECEF", "" },
        { uint32_t(41), "AIDING_VEL_NED", "" },
        { uint32_t(42), "AIDING_VEL_BODY_FRAME", "" },
        { uint32_t(49), "AIDING_HEADING_TRUE", "" },
        { uint32_t(50), "AIDING_MAGNETIC_FIELD", "" },
        { uint32_t(51), "AIDING_PRESSURE", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FilterAidingMeasurementType",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::FilterMeasurementIndicator>
{
    using type = data_filter::FilterMeasurementIndicator;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "enabled", "" },
        { uint32_t(2), "used", "" },
        { uint32_t(4), "residual_high_warning", "" },
        { uint32_t(8), "sample_time_warning", "" },
        { uint32_t(16), "configuration_error", "" },
        { uint32_t(32), "max_num_meas_exceeded", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "FilterMeasurementIndicator",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::AidingMeasurementSummary>
{
    using type = data_filter::AidingMeasurementSummary;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "[seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::time_of_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "source",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::source>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "type",
            /* .docs          = */ "(see product manual for supported types) Note: values 0x20 and above correspond to commanded aiding measurements in the 0x13 Aiding command set.",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_filter::FilterAidingMeasurementType>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::FilterAidingMeasurementType, &type::type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "indicator",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_filter::FilterMeasurementIndicator>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::FilterMeasurementIndicator, &type::indicator>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AidingMeasurementSummary",
        /* .title       = */ "aiding_measurement_summary",
        /* .docs        = */ "Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::OdometerScaleFactorError>
{
    using type = data_filter::OdometerScaleFactorError;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scale_factor_error",
            /* .docs          = */ "[dimensionless]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::scale_factor_error>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::OdometerScaleFactorError",
        /* .title       = */ "Odometer Scale Factor Error",
        /* .docs        = */ "Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::OdometerScaleFactorErrorUncertainty>
{
    using type = data_filter::OdometerScaleFactorErrorUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scale_factor_error_uncertainty",
            /* .docs          = */ "[dimensionless]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::scale_factor_error_uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::OdometerScaleFactorErrorUncertainty",
        /* .title       = */ "Odometer Scale Factor Error Uncertainty",
        /* .docs        = */ "Filter reported odometer scale factor error uncertainty.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::GnssDualAntennaStatus::FixType>
{
    using type = data_filter::GnssDualAntennaStatus::FixType;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "FIX_NONE", "" },
        { uint32_t(1), "FIX_DA_FLOAT", "" },
        { uint32_t(2), "FIX_DA_FIXED", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "FixType",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags>
{
    using type = data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "rcv_1_data_valid", "" },
        { uint32_t(2), "rcv_2_data_valid", "" },
        { uint32_t(4), "antenna_offsets_valid", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "DualAntennaStatusFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_filter::GnssDualAntennaStatus>
{
    using type = data_filter::GnssDualAntennaStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_of_week",
            /* .docs          = */ "Last dual-antenna GNSS aiding measurement time of week [seconds]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::time_of_week>,
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
        {
            /* .name          = */ "heading_unc",
            /* .docs          = */ "[radians]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::heading_unc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "fix_type",
            /* .docs          = */ "Fix type indicator",
            /* .type          = */ {Type::ENUM, &MetadataFor<data_filter::GnssDualAntennaStatus::FixType>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::GnssDualAntennaStatus::FixType, &type::fix_type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "status_flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags, &type::status_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "0 - invalid, 1 - valid",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::GnssDualAntennaStatus",
        /* .title       = */ "GNSS Dual Antenna Status",
        /* .docs        = */ "Summary information for status of GNSS dual antenna heading estimate.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AidingFrameConfigError>
{
    using type = data_filter::AidingFrameConfigError;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Frame ID for the receiver to which the antenna is attached",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "translation",
            /* .docs          = */ "Translation config X, Y, and Z (m).",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::translation>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "attitude",
            /* .docs          = */ "Attitude quaternion",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Quatf>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Quatf, &type::attitude>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AidingFrameConfigError",
        /* .title       = */ "Aiding Frame Configuration Error",
        /* .docs        = */ "Filter reported aiding source frame configuration error\n\nThese estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_filter::AidingFrameConfigErrorUncertainty>
{
    using type = data_filter::AidingFrameConfigErrorUncertainty;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "frame_id",
            /* .docs          = */ "Frame ID for the receiver to which the antenna is attached",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::frame_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "translation_unc",
            /* .docs          = */ "Translation uncertaint X, Y, and Z (m).",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::translation_unc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "attitude_unc",
            /* .docs          = */ "Attitude uncertainty, X, Y, and Z (radians).",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::attitude_unc>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_filter::AidingFrameConfigErrorUncertainty",
        /* .title       = */ "Aiding Frame Configuration Error Uncertainty",
        /* .docs        = */ "Filter reported aiding source frame configuration error uncertainty\n\nThese estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* DATA_FILTER_FIELDS[] = {
    &MetadataFor<data_filter::PositionLlh>::value,
    &MetadataFor<data_filter::VelocityNed>::value,
    &MetadataFor<data_filter::AttitudeQuaternion>::value,
    &MetadataFor<data_filter::AttitudeDcm>::value,
    &MetadataFor<data_filter::EulerAngles>::value,
    &MetadataFor<data_filter::GyroBias>::value,
    &MetadataFor<data_filter::AccelBias>::value,
    &MetadataFor<data_filter::PositionLlhUncertainty>::value,
    &MetadataFor<data_filter::VelocityNedUncertainty>::value,
    &MetadataFor<data_filter::EulerAnglesUncertainty>::value,
    &MetadataFor<data_filter::GyroBiasUncertainty>::value,
    &MetadataFor<data_filter::AccelBiasUncertainty>::value,
    &MetadataFor<data_filter::LinearAccel>::value,
    &MetadataFor<data_filter::CompAngularRate>::value,
    &MetadataFor<data_filter::Wgs84GravityMag>::value,
    &MetadataFor<data_filter::Status>::value,
    &MetadataFor<data_filter::Timestamp>::value,
    &MetadataFor<data_filter::QuaternionAttitudeUncertainty>::value,
    &MetadataFor<data_filter::GravityVector>::value,
    &MetadataFor<data_filter::HeadingUpdateState>::value,
    &MetadataFor<data_filter::MagneticModel>::value,
    &MetadataFor<data_filter::GyroScaleFactor>::value,
    &MetadataFor<data_filter::AccelScaleFactor>::value,
    &MetadataFor<data_filter::GyroScaleFactorUncertainty>::value,
    &MetadataFor<data_filter::AccelScaleFactorUncertainty>::value,
    &MetadataFor<data_filter::MagBias>::value,
    &MetadataFor<data_filter::MagBiasUncertainty>::value,
    &MetadataFor<data_filter::CompAccel>::value,
    &MetadataFor<data_filter::StandardAtmosphere>::value,
    &MetadataFor<data_filter::PressureAltitude>::value,
    &MetadataFor<data_filter::DensityAltitude>::value,
    &MetadataFor<data_filter::MagnetometerOffset>::value,
    &MetadataFor<data_filter::MagnetometerMatrix>::value,
    &MetadataFor<data_filter::MagnetometerOffsetUncertainty>::value,
    &MetadataFor<data_filter::MagnetometerMatrixUncertainty>::value,
    &MetadataFor<data_filter::MagnetometerCovarianceMatrix>::value,
    &MetadataFor<data_filter::MagnetometerResidualVector>::value,
    &MetadataFor<data_filter::AntennaOffsetCorrection>::value,
    &MetadataFor<data_filter::AntennaOffsetCorrectionUncertainty>::value,
    &MetadataFor<data_filter::ClockCorrection>::value,
    &MetadataFor<data_filter::ClockCorrectionUncertainty>::value,
    &MetadataFor<data_filter::MultiAntennaOffsetCorrection>::value,
    &MetadataFor<data_filter::MultiAntennaOffsetCorrectionUncertainty>::value,
    &MetadataFor<data_filter::EcefPosUncertainty>::value,
    &MetadataFor<data_filter::EcefVelUncertainty>::value,
    &MetadataFor<data_filter::EcefPos>::value,
    &MetadataFor<data_filter::EcefVel>::value,
    &MetadataFor<data_filter::RelPosNed>::value,
    &MetadataFor<data_filter::GnssPosAidStatus>::value,
    &MetadataFor<data_filter::GnssAttAidStatus>::value,
    &MetadataFor<data_filter::HeadAidStatus>::value,
    &MetadataFor<data_filter::AidingMeasurementSummary>::value,
    &MetadataFor<data_filter::OdometerScaleFactorError>::value,
    &MetadataFor<data_filter::OdometerScaleFactorErrorUncertainty>::value,
    &MetadataFor<data_filter::GnssDualAntennaStatus>::value,
    &MetadataFor<data_filter::AidingFrameConfigError>::value,
    &MetadataFor<data_filter::AidingFrameConfigErrorUncertainty>::value,
};

static constexpr DescriptorSetInfo DATA_FILTER = {
    /*.descriptor =*/ mip::data_filter::DESCRIPTOR_SET,
    /*.name       =*/ "Filter Data",
    /*.fields     =*/ DATA_FILTER_FIELDS,
};

} // namespace mip::metadata

