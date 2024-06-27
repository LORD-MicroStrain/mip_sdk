

#include <mip/metadata/mip_definitions.hpp>

#include <mip/mip_field.hpp>

#include <bit>
#include <string>
#include <cstdio>
#include <cstdarg>
#include <cinttypes>
#include <sstream>
#include <ostream>


using namespace mip::metadata;

extern Definitions mipdefs;


////////////////////////////////////////////////////////////////////////////////
///@brief Format an arithmetic type (including bool) from buffer to a string.
///
///@param ss      Output stream
///@param buffer  Data buffer
///@param length  Length of data buffer
///@param offset  Offset into data buffer where value is stored (may be out of range)
///
///@returns ss
///
template<class T>
std::ostream& formatBasicType(std::ostream& ss, const uint8_t* buffer, size_t length, size_t offset)
{
    if(offset+sizeof(T) > length)
        return ss << "?";

    return ss << microstrain::read<T>(buffer);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Read an integral value from the raw buffer.
///
///@param type   Real type of the data in the buffer. Used for size.
///@param buffer  Data buffer
///@param length  Length of data buffer
///@param offset  Offset into data buffer where value is stored (may be out of range)
///
///@returns A uint64_t containing the value. All smaller integers are converted to this type.
///@returns std::nullopt (no value) if the offset/size is beyond the end of the buffer.
///
static std::optional<uint64_t> readIntegralValue(Type type, const uint8_t* buffer, size_t length, size_t offset)
{
    switch(type)
    {
    case Type::U8:  return microstrain::extract<uint8_t >(buffer, length, offset, false);
    case Type::S8:  return microstrain::extract< int8_t >(buffer, length, offset, false);
    case Type::U16: return microstrain::extract<uint16_t>(buffer, length, offset, false);
    case Type::S16: return microstrain::extract< int32_t>(buffer, length, offset, false);
    case Type::U32: return microstrain::extract<uint32_t>(buffer, length, offset, false);
    case Type::S32: return microstrain::extract< int64_t>(buffer, length, offset, false);
    case Type::U64: return microstrain::extract<uint64_t>(buffer, length, offset, false);
    case Type::S64: return microstrain::extract< int64_t>(buffer, length, offset, false);
    default: return std::nullopt;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format enum from buffer to a string.
///
///@param ss      Output stream
///@param info    Enum metadata (may be NULL)
///@param buffer  Data buffer
///@param length  Length of data buffer
///@param offset  Offset into data buffer where value is stored
///
///@returns ss
///
std::ostream& formatEnum(std::ostream& ss, const EnumInfo* info, const uint8_t* buffer, size_t length, size_t offset= 0)
{
    if(!info)
        return ss << "<enum>";

    std::optional<uint64_t> value = readIntegralValue(info->type, buffer, length, offset);

    if(!value)
        return ss << "?";

    auto it = std::find_if(
        info->entries.begin(), info->entries.end(),
        [value](const auto& entry){ return entry.value == value; }
    );
    const char* name = (it != info->entries.end()) ? it->name : "?";

    return ss << *value << '(' << name << ')';
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format bitfield from buffer to a string.
///
///@param ss      Output stream
///@param info    Bitfield metadata (may be NULL)
///@param buffer  Data buffer
///@param length  Length of data buffer
///@param offset  Offset into data buffer where value is stored (may be out of range)
///
///@returns ss
///
std::ostream& formatBitfield(std::ostream& ss, const BitfieldInfo* info, const uint8_t* buffer, size_t length, size_t offset)
{
    if(!info)
        return ss << "<bitfield>";

    std::optional<uint64_t> value = readIntegralValue(info->type, buffer, length, offset);

    if(!value)
        return ss << "?";

    ss << '{';
    size_t i=0;
    for(const auto& entry : info->entries)
    {
        if(i > 0)
            ss << ", ";

        ss << entry.name << '=';

        const size_t firstBit = std::countl_zero(entry.value);
        ss << ((*value & entry.value) >> firstBit);

        i++;
    }
    ss << '}';
    return ss;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format union from buffer to a string.
///
///@param ss      Output stream
///@param info    Union metadata (may be NULL)
///@param parent  Struct metadata of the parent object (for parameter access).
///@param buffer  Data buffer
///@param length  Length of data buffer
///@param offset  Offset into data buffer where value is stored (may be out of range)
///
///@returns ss
///
std::ostream& formatUnion(std::ostream& ss, const UnionInfo* info, const StructInfo& parent, const uint8_t* buffer, size_t length, size_t offset=0)
{
    if(!info)
        return ss << "<union>";
    if(offset > length)
        return ss << "?";

    ss << info->name << '{';

    // Todo
    // const ParameterInfo* param = nullptr;
    //
    // for(const auto& p : info->parameters)
    // {
    //     if(p.union_index < parent.parameters.size())
    //     {
    //
    //     }
    // }

    ss << '}';
    return ss;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format struct from buffer to a string.
///
///@param ss      Output stream
///@param info    Struct metadata (may be NULL)
///@param buffer  Data buffer
///@param length  Length of data buffer
///@param offset  Offset into data buffer where value is stored (may be out of range)
///
///@returns ss
///
std::ostream& formatStruct(std::ostream& ss, const StructInfo* info, const uint8_t* buffer, size_t length, size_t offset=0)
{
    if(!info)
        return ss << "<struct>";
    if(offset > length)
        return ss << "?";

    ss << '{';

    size_t i=0;
    for(const auto& param : info->parameters)
    {
        if(i > 0)
            ss << ", ";

        ss << param.name << '=';

        switch(param.type.type)
        {
        case Type::NONE:
            return ss << "-";

        case Type::BOOL:   formatBasicType<bool    >(ss, buffer, length, param.byte_offset); break;
        case Type::U8:     formatBasicType<uint8_t >(ss, buffer, length, param.byte_offset); break;
        case Type::S8:     formatBasicType< int8_t >(ss, buffer, length, param.byte_offset); break;
        case Type::U16:    formatBasicType<uint16_t>(ss, buffer, length, param.byte_offset); break;
        case Type::S16:    formatBasicType< int16_t>(ss, buffer, length, param.byte_offset); break;
        case Type::U32:    formatBasicType<uint32_t>(ss, buffer, length, param.byte_offset); break;
        case Type::S32:    formatBasicType< int32_t>(ss, buffer, length, param.byte_offset); break;
        case Type::U64:    formatBasicType<uint64_t>(ss, buffer, length, param.byte_offset); break;
        case Type::S64:    formatBasicType< int64_t>(ss, buffer, length, param.byte_offset); break;
        case Type::FLOAT:  formatBasicType<float   >(ss, buffer, length, param.byte_offset); break;
        case Type::DOUBLE: formatBasicType<double  >(ss, buffer, length, param.byte_offset); break;

        case Type::ENUM:
            formatEnum(
                ss, static_cast<const EnumInfo *>(param.type.infoPtr),
                buffer, length, param.byte_offset
            );
            break;

        case Type::BITFIELD:
            formatBitfield(
                ss, static_cast<const BitfieldInfo *>(param.type.infoPtr),
                buffer, length, param.byte_offset
            );
            break;

        case Type::STRUCT:
            formatStruct(
                ss, static_cast<const StructInfo *>(param.type.infoPtr),
                buffer, length, param.byte_offset
            );
            break;

        case Type::UNION:
            formatUnion(
                ss, static_cast<const UnionInfo *>(param.type.infoPtr), *info,
                buffer, length, param.byte_offset
            );
            break;
        }

        i++;
    }
    ss << '}';

    return ss;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format mip field from buffer to a string.
///
///@param ss      Output stream
///@param field   FieldView containing data to print
///
///@returns ss
///
std::ostream& formatField(std::ostream& ss, const mip::FieldView& field)
{
    const FieldInfo* info = mipdefs.findField(field.descriptor());

    if(!info)
    {
        ss << "Unknown(%02X,%02x)[";
        for(uint8_t i=0; i<field.payloadLength(); i++)
            ss << std::hex << field.payload(i);
        ss << ']';
    }
    else
    {
        ss << info->name;
        formatStruct(ss, static_cast<const StructInfo *>(info), field.payload(), field.payloadLength());
    }
    return ss;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Format bitfield to a string.
///
///@param field FieldView holding the buffer containing the field data.
///
///@returns A human and machine-readable string.
///
std::string formatField(const mip::FieldView& field)
{
    std::stringstream ss;

    formatField(ss, field);

    return ss.str();
}
