#pragma once

#include <mip/metadata/mip_metadata.hpp>
#include <mip/metadata/mip_meta_utils.hpp>

#include <mip/definitions/common.hpp>

namespace mip::metadata
{

template<>
struct MetadataFor<CmdResult>
{
    using type = CmdResult;

    static constexpr inline EnumInfo::Entry entries[] = {
        {
            /*.value =*/ CmdResult::ACK_OK,
            /*.name  =*/ "OK",
            /*.docs  =*/ "Command completed successfully",
        },
        {
            /*.value =*/ CmdResult::NACK_COMMAND_UNKNOWN,
            /*.name  =*/ "Unknown Command",
            /*.docs  =*/ "The device did not recognize the command",
        },
        {
            /*.value =*/ CmdResult::NACK_INVALID_CHECKSUM,
            /*.name  =*/ "Invalid Checksum",
            /*.docs  =*/ "An packet with an invalid checksum was received by the device",
        },
        {
            /*.value =*/ CmdResult::NACK_INVALID_PARAM,
            /*.name  =*/ "Invalid Parameter",
            /*.docs  =*/ "One or more parameters to the command were not valid",
        },
        {
            /*.value =*/ CmdResult::NACK_COMMAND_FAILED,
            /*.name  =*/ "Command Failed",
            /*.docs  =*/ "The device could not complete the command",
        },
        {
            /*.value =*/ CmdResult::NACK_COMMAND_TIMEOUT,
            /*.name  =*/ "Device Timeout",
            /*.docs  =*/ "The device reported a timeout condition",
        },
        // Status codes not represented here as they don't come from the device.
    };

    static constexpr inline EnumInfo value = {
        /*.name    =*/ "CmdResult",
        /*.docs    =*/ "Acknowledgement/reply code from the device after a command is issued",
        /*.type    =*/ Type::U8,
        /*.entries =*/ entries,
    };
};

struct ReplyField
{
    static constexpr inline uint8_t FIELD_DESCRIPTOR = 0xF1;
    static constexpr inline CompositeDescriptor DESCRIPTOR = {INVALID_DESCRIPTOR_SET, FIELD_DESCRIPTOR};

    uint8_t   cmd_field_desc;
    CmdResult result;

    size_t insert(Serializer& buffer) const { return buffer.insert(cmd_field_desc, result.value); }
    size_t extract(Serializer& buffer) { return buffer.extract(cmd_field_desc, result.value); }
};

template<>
struct MetadataFor<ReplyField>
{
    using type = ReplyField;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /*.name       =*/ "cmd_field_desc",
            /*.docs       =*/ "The field descriptor of the command this field acknowledges.",
            /*.type       =*/ {Type::U8},
            /*.accessor   =*/ nullptr, //utils::access<type, uint8_t, &type::cmd_field_desc>,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "result",
            /*.docs       =*/ "Result of the command.",
            /*.type       =*/ {Type::ENUM, &MetadataFor<CmdResult>::value},
            /*.accessor   =*/ nullptr, //utils::access<type, CmdResult, &type::result>,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /*.name         = */ "ReplyField",
        /* .title       = */ "Command Reply",
        /* .docs        = */ "Sent by the device to indicate the result of a command.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<DescriptorRate>
{
    using type = DescriptorRate;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /*.name       =*/ "descriptor",
            /*.docs       =*/ "MIP data descriptor",
            /*.type       =*/ {Type::U8},
            /*.accessor   =*/ nullptr, //utils::access<type, uint8_t, &type::descriptor>,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "decimation",
            /*.docs       =*/ "Decimation from the base rate",
            /*.type       =*/ {Type::U16},
            /*.accessor   =*/ nullptr, //utils::access<type, uint16_t, &type::decimation>,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
    };

    static constexpr inline StructInfo value = {
        /*.name       =*/ "DescriptorRate",
        /*.title      =*/ "Descriptor Rate",
        /*.docs       =*/ "Descriptor rate information",
        /*.parameters =*/ parameters,
    };
};


template<typename T, size_t N>
struct MetadataFor<Vector<T,N>>
{
    using type = Vector<T,N>;

    static constexpr inline ParameterInfo parameters_v[] = {
        {
            /*.name       =*/ "x",
            /*.docs       =*/ "X axis",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "y",
            /*.docs       =*/ "Y axis",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "z",
            /*.docs       =*/ "Z axis",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "w",
            /*.docs       =*/ "W axis",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
    };

    static constexpr inline StructInfo vector_f[3] = {
        {
            /*.name       =*/ "Vector2f",
            /*.title      =*/ "Vector2f",
            /*.docs       =*/ "2-dimensional vector of floats",
            /*.parameters =*/ {parameters_v, 2}
        },
        {
            /*.name       =*/ "Vector3f",
            /*.title      =*/ "Vector3f",
            /*.docs       =*/ "3-dimensional vector of floats",
            /*.parameters =*/ {parameters_v, 3}
        },
        {
            /*.name       =*/ "Vector4f",
            /*.title      =*/ "Vector4f",
            /*.docs       =*/ "4-dimensional vector of floats",
            /*.parameters =*/ {parameters_v, 4}
        },
    };

    static constexpr inline StructInfo vector_d[3] = {
        {
            /*.name       =*/ "Vector2d",
            /*.title      =*/ "Vector2d",
            /*.docs       =*/ "2-dimensional vector of doubles",
            /*.parameters =*/ {parameters_v, 2}
        },
        {
            /*.name       =*/ "Vector3d",
            /*.title      =*/ "Vector3d",
            /*.docs       =*/ "3-dimensional vector of doubles",
            /*.parameters =*/ {parameters_v, 3}
        },
        {
            /*.name       =*/ "Vector4d",
            /*.title      =*/ "Vector4d",
            /*.docs       =*/ "4-dimensional vector of doubles",
            /*.parameters =*/ {parameters_v, 4}
        },
    };

    static constexpr inline ParameterInfo parameters_m[] = {
        {
            /*.name       =*/ "m[0]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[1]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[2]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[3]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[4]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[5]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[6]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[7]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
        {
            /*.name       =*/ "m[8]",
            /*.docs       =*/ "Matrix data",
            /*.type       =*/ {utils::ParamType<T>::value},
            /*.accessor   =*/ nullptr,
            /*.attributes =*/ NO_FUNCTIONS,
            /*.count      =*/ 1,
            /*.condition  =*/ {},
        },
    };

    static constexpr inline StructInfo matrix_f = {
        /*.name       =*/ "Matrix3d",
        /*.title      =*/ "3x3 Double Matrix",
        /*.docs       =*/ "Represents a 3D matrix of doubles.",
        /*.parameters =*/ parameters_m,
    };

    static constexpr inline StructInfo matrix_d = {
        /*.name       =*/ "Matrix3f",
        /*.title      =*/ "3x3 Float Matrix",
        /*.docs       =*/ "Represents a 3D matrix of floats.",
        /*.parameters =*/ parameters_m,
    };

    static_assert(std::is_floating_point<T>::value, "Expected either float or double");
    static_assert(N == 9 || (N >= 2 && N <= 4), "N should be in the range [2,4] or exactly 9.");

    static constexpr inline const StructInfo& value =
        N == 9 ? (std::is_same<T,double>::value ? matrix_d        : matrix_f) :
                 (std::is_same<T,double>::value ? vector_d[N - 2] : vector_f[N - 2]);
};

} // namespace mip::metadata
