#pragma once

#include <mip/definitions/data_sensor.hpp>
#include "common.hpp"

#include <mip/metadata/mip_metadata.hpp>
#include <functional>

namespace mip::metadata
{

template<>
struct MetadataFor<data_sensor::ScaledAccel>
{
    using Field = data_sensor::ScaledAccel;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scaled_accel",
            /* .docs          = */ "Scaled acceleration in [g]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<Field, Vector3f, &Field::scaled_accel>,
            ///* .byte_offset   = */ 0,
            ///* .functions     = */ {false, false, false, false, false},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "ScaledAccel",
        /* .title       = */ "Scaled Accel",
        /* .docs        = */ "Acceleration",
        /* .parameters  = */ parameters,
        /* .descriptor    = */ Field::DESCRIPTOR,
        ///* .functions   = */ {false,false,false,false,false},
        ///* .proprietary = */ false,
        ///* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::ScaledGyro>
{
    using Field = data_sensor::ScaledGyro;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "scaled_gyro",
            /* .docs          = */ "Scaled angular rate in [rad/s]",
            /* .type          = */ {Type::STRUCT, &MetadataFor<Vector3f>::value},
            /* .accessor      = */ utils::access<Field, Vector3f, &Field::scaled_gyro>,
            ///* .byte_offset   = */ 0,
            ///* .functions     = */ {false, false, false, false, false},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "ScaledGyro",
        /* .title       = */ "Scaled Gyro",
        /* .docs        = */ "Angular rate of rotation.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ Field::DESCRIPTOR,
        ///* .functions   = */ {false,false,false,false,false},
        ///* .proprietary = */ false,
        ///* .response    = */ nullptr,
    };
};


static constexpr inline std::initializer_list<const FieldInfo*> ALL_SENSOR_DATA = {
    &MetadataFor<data_sensor::ScaledAccel>::value,
    &MetadataFor<data_sensor::ScaledGyro>::value,
};

} // namespace mip
