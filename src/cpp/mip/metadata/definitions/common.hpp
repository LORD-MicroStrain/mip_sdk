#pragma once

#include <mip/metadata/mip_metadata.hpp>
#include <mip/metadata/utils.hpp>

#include <mip/definitions/common.hpp>


namespace mip::metadata
{

template<>
struct MetadataFor<DescriptorRate>
{
    using type = DescriptorRate;

    static constexpr inline ParameterInfo parameters[] = {
        {
            .name = "descriptor",
            .docs = "MIP data descriptor",
            .type = {Type::U8},
            .accessor = utils::access<type, uint8_t, &type::descriptor>,
        },
        {
            .name = "decimation",
            .docs = "Decimation from the base rate",
            .type = {Type::U16},
            .accessor = utils::access<type, uint16_t, &type::decimation>,
        },
    };

    static constexpr inline StructInfo value = {
        .name  = "DescriptorRate",
        .title = "Descriptor Rate",
        .docs  = "Descriptor rate information",
        .parameters = parameters,
    };
};

template<>
struct MetadataFor<Vector3f>
{
    using type = Vector3f;

    static constexpr inline ParameterInfo parameters[] = {
        {
            .name = "x",
            .docs = "X axis",
            .type = {Type::FLOAT},
            .accessor = nullptr,
        },
        {
            .name = "y",
            .docs = "Y axis",
            .type = {Type::FLOAT},
            .accessor = nullptr,
        },
        {
            .name = "z",
            .docs = "Z axis",
            .type = {Type::FLOAT},
            .accessor = nullptr,
        },
    };

    static constexpr inline StructInfo value = {
        .name = "Vector3f",
        .title = "3D Vector",
        .docs = "Represents a 3D vector of floats.",
        .parameters = parameters,
    };
};

} // namespace mip::metadata
