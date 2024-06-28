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

//template<>
//struct MetadataFor<Vector3f>
//{
//    using type = Vector3f;
//
//    static constexpr inline ParameterInfo parameters[] = {
//        {
//            .name = "x",
//            .docs = "X axis",
//            .type = {Type::FLOAT},
//            .accessor = nullptr,
//        },
//        {
//            .name = "y",
//            .docs = "Y axis",
//            .type = {Type::FLOAT},
//            .accessor = nullptr,
//        },
//        {
//            .name = "z",
//            .docs = "Z axis",
//            .type = {Type::FLOAT},
//            .accessor = nullptr,
//        },
//    };
//
//    static constexpr inline StructInfo value = {
//        .name = "Vector3f",
//        .title = "3D Vector",
//        .docs = "Represents a 3D vector of floats.",
//        .parameters = parameters,
//    };
//};


template<typename T, size_t N>
struct MetadataFor<Vector<T,N>>
{
    using type = Vector<T,N>;

    static constexpr inline ParameterInfo parameters[] = {
        {
            .name = "x",
            .docs = "X axis",
            .type = {utils::ParamType<T>::value},
            .accessor = nullptr,
        },
        {
            .name = "y",
            .docs = "Y axis",
            .type = {utils::ParamType<T>::value},
            .accessor = nullptr,
        },
        {
            .name = "z",
            .docs = "Z axis",
            .type = {utils::ParamType<T>::value},
            .accessor = nullptr,
        },
        {
            .name = "w",
            .docs = "W axis",
            .type = {utils::ParamType<T>::value},
            .accessor = nullptr,
        },
    };


    static constexpr inline StructInfo values_f[] = {
        {
            .name = "Vector2f",
            .title = "Vector2f",
            .docs = "2-dimensional vector of floats",
            .parameters = {parameters, 2}
        },
        {
            .name = "Vector3f",
            .title = "Vector3f",
            .docs = "3-dimensional vector of floats",
            .parameters = {parameters, 3}
        },
        {
            .name = "Vector4f",
            .title = "Vector4f",
            .docs = "4-dimensional vector of floats",
            .parameters = {parameters, 4}
        },
    };
    static constexpr inline StructInfo values_d[] = {
        {
            .name = "Vector2d",
            .title = "Vector2d",
            .docs = "2-dimensional vector of doubles",
            .parameters = {parameters, 2}
        },
        {
            .name = "Vector3d",
            .title = "Vector3d",
            .docs = "3-dimensional vector of doubles",
            .parameters = {parameters, 3}
        },
        {
            .name = "Vector4d",
            .title = "Vector4d",
            .docs = "4-dimensional vector of doubles",
            .parameters = {parameters, 4}
        },
    };

    static_assert(std::is_floating_point_v<T>, "Expected either float or double");
    static_assert(N >= 2 && N <= 4, "N should be in the range [2,4].");

    static constexpr inline const StructInfo& value = std::is_same_v<T,double> ? values_d[N-2] : values_f[N-2];
};


template<>
struct MetadataFor<Matrix3f>
{
    using type = Matrix3f;

    static constexpr inline ParameterInfo parameters[] = {
        {
            .name = "m",
            .docs = "Matrix data",
            .type = {Type::FLOAT},
            .accessor = nullptr,
            .count = 3,
        },
    };

    static constexpr inline StructInfo value = {
        .name = "Matrix3f",
        .title = "3x3 Float Matrix",
        .docs = "Represents a 3D matrix of floats.",
        .parameters = parameters,
    };
};

template<>
struct MetadataFor<Matrix3d>
{
    using type = Matrix3d;

    static constexpr inline ParameterInfo parameters[] = {
        {
            .name = "m",
            .docs = "Matrix data",
            .type = {Type::DOUBLE},
            .accessor = nullptr,
            .count = 3,
        },
    };

    static constexpr inline StructInfo value = {
        .name = "Matrix3f",
        .title = "3x3 Double Matrix",
        .docs = "Represents a 3D matrix of doubles.",
        .parameters = parameters,
    };
};

} // namespace mip::metadata
