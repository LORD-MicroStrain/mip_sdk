#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/data_sensor.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<data_sensor::RawAccel>
{
    using type = data_sensor::RawAccel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "raw_accel",
            /* .docs          = */ "Native sensor counts",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::raw_accel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::RawAccel",
        /* .title       = */ "raw_accel",
        /* .docs        = */ "Three element vector representing the sensed acceleration.\nThis quantity is temperature compensated and expressed in the sensor body frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::RawGyro>
{
    using type = data_sensor::RawGyro;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "raw_gyro",
            /* .docs          = */ "Native sensor counts",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::raw_gyro>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::RawGyro",
        /* .title       = */ "raw_gyro",
        /* .docs        = */ "Three element vector representing the sensed angular rate.\nThis quantity is temperature compensated and expressed in the sensor body frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::RawMag>
{
    using type = data_sensor::RawMag;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "raw_mag",
            /* .docs          = */ "Native sensor counts",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::raw_mag>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::RawMag",
        /* .title       = */ "raw_mag",
        /* .docs        = */ "Three element vector representing the sensed magnetic field.\nThis quantity is temperature compensated and expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::RawPressure>
{
    using type = data_sensor::RawPressure;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "raw_pressure",
            /* .docs          = */ "Native sensor counts",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::raw_pressure>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::RawPressure",
        /* .title       = */ "raw_pressure",
        /* .docs        = */ "Scalar value representing the sensed ambient pressure.\nThis quantity is temperature compensated.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::ScaledAccel>
{
    using type = data_sensor::ScaledAccel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scaled_accel",
            /* .docs          = */ "(x, y, z)[g]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scaled_accel>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::ScaledAccel",
        /* .title       = */ "scaled_accel",
        /* .docs        = */ "3-element vector representing the sensed acceleration.\nThis quantity is temperature compensated and expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::ScaledGyro>
{
    using type = data_sensor::ScaledGyro;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scaled_gyro",
            /* .docs          = */ "(x, y, z) [radians/second]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scaled_gyro>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::ScaledGyro",
        /* .title       = */ "scaled_gyro",
        /* .docs        = */ "3-element vector representing the sensed angular rate.\nThis quantity is temperature compensated and expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::ScaledMag>
{
    using type = data_sensor::ScaledMag;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scaled_mag",
            /* .docs          = */ "(x, y, z) [Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::scaled_mag>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::ScaledMag",
        /* .title       = */ "scaled_mag",
        /* .docs        = */ "3-element vector representing the sensed magnetic field.\nThis quantity is temperature compensated and expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::ScaledPressure>
{
    using type = data_sensor::ScaledPressure;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scaled_pressure",
            /* .docs          = */ "[mBar]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::scaled_pressure>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::ScaledPressure",
        /* .title       = */ "scaled_pressure",
        /* .docs        = */ "Scalar value representing the sensed ambient pressure.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::DeltaTheta>
{
    using type = data_sensor::DeltaTheta;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "delta_theta",
            /* .docs          = */ "(x, y, z) [radians]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::delta_theta>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::DeltaTheta",
        /* .title       = */ "delta_theta",
        /* .docs        = */ "3-element vector representing the time integral of angular rate.\nThis quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::DeltaVelocity>
{
    using type = data_sensor::DeltaVelocity;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "delta_velocity",
            /* .docs          = */ "(x, y, z) [g*sec]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::delta_velocity>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::DeltaVelocity",
        /* .title       = */ "delta_velocity",
        /* .docs        = */ "3-element vector representing the time integral of acceleration.\nThis quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::CompOrientationMatrix>
{
    using type = data_sensor::CompOrientationMatrix;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "m",
            /* .docs          = */ "Matrix elements in row-major order.",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::m>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::CompOrientationMatrix",
        /* .title       = */ "Complementary Filter Orientation Matrix",
        /* .docs        = */ "3x3 Direction Cosine Matrix EQSTART M_{ned}^{veh} EQEND describing the orientation of the device with respect to the NED local-level frame.\nThis matrix satisfies the following relationship:\n\nEQSTART v^{veh} = M_{ned}^{veh} v^{ned} EQEND<br/>\n\nWhere:<br/>\n\nEQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame. <br/>\nEQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>\n<br/>\nThe matrix elements are stored is row-major order: EQSTART M = \\begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \\end{bmatrix} EQEND",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::CompQuaternion>
{
    using type = data_sensor::CompQuaternion;

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
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::CompQuaternion",
        /* .title       = */ "Complementary Filter Quaternion",
        /* .docs        = */ "4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.\nThis quaternion satisfies the following relationship:\n\nEQSTART p^{veh} = q^{-1} p^{ned} q EQEND<br/>\n\nWhere:<br/>\nEQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the rotation. <br/>\nEQSTART p^ned = (0, v^{ned}_x, v^{ned}_y, v^{ned}_z) EQEND and EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame.<br/>\nEQSTART p^veh = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::CompEulerAngles>
{
    using type = data_sensor::CompEulerAngles;

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
        /* .name        = */ "data_sensor::CompEulerAngles",
        /* .title       = */ "Complementary Filter Euler Angles",
        /* .docs        = */ "Euler angles describing the orientation of the device with respect to the NED local-level frame.\nThe Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::CompOrientationUpdateMatrix>
{
    using type = data_sensor::CompOrientationUpdateMatrix;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "m",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Matrix3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Matrix3f, &type::m>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::CompOrientationUpdateMatrix",
        /* .title       = */ "Complementary Filter Orientation Update Matrix",
        /* .docs        = */ "DEPRECATED!",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::OrientationRawTemp>
{
    using type = data_sensor::OrientationRawTemp;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "raw_temp",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::raw_temp>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::OrientationRawTemp",
        /* .title       = */ "orientation_raw_temp",
        /* .docs        = */ "DEPRECATED!",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::InternalTimestamp>
{
    using type = data_sensor::InternalTimestamp;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "counts",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::counts>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::InternalTimestamp",
        /* .title       = */ "internal_timestamp",
        /* .docs        = */ "DEPRECATED!",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::PpsTimestamp>
{
    using type = data_sensor::PpsTimestamp;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "seconds",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::seconds>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "useconds",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::useconds>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::PpsTimestamp",
        /* .title       = */ "PPS Timestamp",
        /* .docs        = */ "DEPRECATED!",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::GpsTimestamp::ValidFlags>
{
    using type = data_sensor::GpsTimestamp::ValidFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "pps_valid", "True when the PPS signal is present." },
        { uint32_t(2), "time_refresh", "Toggles each time the time is updated via internal GPS or the GPS Time Update command (0x01, 0x72)." },
        { uint32_t(4), "time_initialized", "True if the time has ever been set." },
        { uint32_t(8), "tow_valid", "True if the time of week is valid." },
        { uint32_t(16), "week_number_valid", "True if the week number is valid." },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ValidFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_sensor::GpsTimestamp>
{
    using type = data_sensor::GpsTimestamp;

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
            /* .type          = */ {Type::BITS, &MetadataFor<data_sensor::GpsTimestamp::ValidFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_sensor::GpsTimestamp::ValidFlags, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::GpsTimestamp",
        /* .title       = */ "gps_timestamp",
        /* .docs        = */ "GPS timestamp of the SENSOR data\n\nShould the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.\nUpon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.\nIf synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.\n\nNote: this data field may be deprecated in the future. The more flexible shared data field (0x80, 0xD3) should be used instead.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::TemperatureAbs>
{
    using type = data_sensor::TemperatureAbs;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "min_temp",
            /* .docs          = */ "[degC]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::min_temp>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "max_temp",
            /* .docs          = */ "[degC]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::max_temp>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mean_temp",
            /* .docs          = */ "[degC]",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::mean_temp>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::TemperatureAbs",
        /* .title       = */ "Temperature Statistics",
        /* .docs        = */ "SENSOR reported temperature statistics\n\nTemperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.\nAll quantities are calculated with respect to the last power on or reset, whichever is later.\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::UpVector>
{
    using type = data_sensor::UpVector;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "up",
            /* .docs          = */ "[Gs]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::up>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::UpVector",
        /* .title       = */ "up_vector",
        /* .docs        = */ "Gyro-stabilized 3-element vector representing the complementary filter's estimated vertical direction.\nThis quantity is expressed in the vehicle frame.\n\nThis quantity is sensitive to non-gravitational accelerations, which may cause notable deviations from the true vertical direction.\n\nFor legacy reasons, this vector is the inverse of the gravity vector.\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::NorthVector>
{
    using type = data_sensor::NorthVector;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "north",
            /* .docs          = */ "[Gauss]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ nullptr, //utils::access<type, Vector3f, &type::north>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::NorthVector",
        /* .title       = */ "north_vector",
        /* .docs        = */ "Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.\nThis quantity is expressed in the vehicle frame.\n\nThis quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::OverrangeStatus::Status>
{
    using type = data_sensor::OverrangeStatus::Status;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "accel_x", "" },
        { uint32_t(2), "accel_y", "" },
        { uint32_t(4), "accel_z", "" },
        { uint32_t(16), "gyro_x", "" },
        { uint32_t(32), "gyro_y", "" },
        { uint32_t(64), "gyro_z", "" },
        { uint32_t(256), "mag_x", "" },
        { uint32_t(512), "mag_y", "" },
        { uint32_t(1024), "mag_z", "" },
        { uint32_t(4096), "press", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "Status",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<data_sensor::OverrangeStatus>
{
    using type = data_sensor::OverrangeStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "status",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<data_sensor::OverrangeStatus::Status>::value},
            /* .accessor      = */ nullptr, //utils::access<type, data_sensor::OverrangeStatus::Status, &type::status>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::OverrangeStatus",
        /* .title       = */ "overrange_status",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::OdometerData>
{
    using type = data_sensor::OdometerData;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "speed",
            /* .docs          = */ "Average speed over the time interval [m/s]. Can be negative for quadrature encoders.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::speed>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "uncertainty",
            /* .docs          = */ "Uncertainty of velocity [m/s].",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::uncertainty>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "valid_flags",
            /* .docs          = */ "If odometer is configured, bit 0 will be set to 1.",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::valid_flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_sensor::OdometerData",
        /* .title       = */ "odometer_data",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* DATA_SENSOR_FIELDS[] = {
    &MetadataFor<data_sensor::RawAccel>::value,
    &MetadataFor<data_sensor::RawGyro>::value,
    &MetadataFor<data_sensor::RawMag>::value,
    &MetadataFor<data_sensor::ScaledAccel>::value,
    &MetadataFor<data_sensor::ScaledGyro>::value,
    &MetadataFor<data_sensor::ScaledMag>::value,
    &MetadataFor<data_sensor::DeltaTheta>::value,
    &MetadataFor<data_sensor::DeltaVelocity>::value,
    &MetadataFor<data_sensor::CompOrientationMatrix>::value,
    &MetadataFor<data_sensor::CompQuaternion>::value,
    &MetadataFor<data_sensor::CompOrientationUpdateMatrix>::value,
    &MetadataFor<data_sensor::CompEulerAngles>::value,
    &MetadataFor<data_sensor::OrientationRawTemp>::value,
    &MetadataFor<data_sensor::InternalTimestamp>::value,
    &MetadataFor<data_sensor::PpsTimestamp>::value,
    &MetadataFor<data_sensor::NorthVector>::value,
    &MetadataFor<data_sensor::UpVector>::value,
    &MetadataFor<data_sensor::GpsTimestamp>::value,
    &MetadataFor<data_sensor::TemperatureAbs>::value,
    &MetadataFor<data_sensor::RawPressure>::value,
    &MetadataFor<data_sensor::ScaledPressure>::value,
    &MetadataFor<data_sensor::OverrangeStatus>::value,
    &MetadataFor<data_sensor::OdometerData>::value,
};

static constexpr DescriptorSetInfo DATA_SENSOR = {
    /*.descriptor =*/ mip::data_sensor::DESCRIPTOR_SET,
    /*.name       =*/ "Sensor Data",
    /*.fields     =*/ DATA_SENSOR_FIELDS,
};

} // namespace mip::metadata

