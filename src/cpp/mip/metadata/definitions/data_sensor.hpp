#pragma once

#include <mip/definitions/data_sensor.hpp>

#include <mip/metadata/mip_metadata.hpp>
#include <functional>

namespace mip::metadata
{

template<>
struct MetadataFor<data_sensor::ScaledAccel>
{
    using Field = data_sensor::ScaledAccel;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "CommSpeed",
        /* .title       = */ "Comm Port Speed",
        /* .docs        = */ "Changes the comm port speed.",
        /* .parameters  = */ {
            {
                /* .name          = */ "scaled_accel",
                /* .docs          = */ "Scaled acceleration in [g]",
                /* .type          = */ {Type::STRUCT},
                /* .accessor      = */ utils::access<Field, Vector3f, &Field::scaled_accel>,
                ///* .byte_offset   = */ 0,
                ///* .functions     = */ {false, false, false, false, false},
            },
        },
        ///* .functions   = */ {false,false,false,false,false},
        ///* .proprietary = */ false,
        ///* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_sensor::ScaledGyro>
{
    using Field = data_sensor::ScaledGyro;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "CommSpeed",
        /* .title       = */ "Comm Port Speed",
        /* .docs        = */ "Changes the comm port speed.",
        /* .parameters  = */ {
            {
                /* .name          = */ "scaled_gyro",
                /* .docs          = */ "Scaled angular rate in [rad/s]",
                /* .type          = */ {Type::STRUCT},
                /* .accessor      = */ utils::access<Field, Vector3f, &Field::scaled_gyro>,
                ///* .byte_offset   = */ 0,
                ///* .functions     = */ {false, false, false, false, false},
            },
        },
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
