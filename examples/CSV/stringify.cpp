

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


// std::string& appendFmt(std::string& base, const char* fmt, ...)
// {
//     std::va_list args;
//     va_start(args, fmt);
//     int length = std::vsnprintf(nullptr, 0, fmt, args);
//     va_end(args);
//
//     if(length > 0)
//     {
//         size_t oldSize = base.length();
//         base.resize(oldSize + length);
//
//         va_start(args, fmt);
//         std::vsnprintf(base.data()+oldSize, length, fmt, args);
//         va_end(args);
//
//         base.resize(base.size()-1);  // Remove extra terminating '\0'
//     }
// }


// template<class T>
// std::string formatBasicType(const uint8_t* buffer, size_t length, size_t offset)
// {
//     if(offset+sizeof(T) > length)
//         return "?";
//
//     T value = microstrain::read<T>(buffer);
//
//     const char* fmt = nullptr;
//     switch(utils::ParamType<T>::value)
//     {
//     case Type::BOOL:
//         return value ? "true" : "false";
//
//     case Type::U8:     fmt = "%" PRIu8;  break;
//     case Type::S8:     fmt = "%" PRIi8; break;
//     case Type::U16:    fmt = "%" PRIu16; break;
//     case Type::S16:    fmt = "%" PRIi16; break;
//     case Type::U32:    fmt = "%" PRIu32; break;
//     case Type::S32:    fmt = "%" PRIi32; break;
//     case Type::U64:    fmt = "%" PRIu64; break;
//     case Type::S64:    fmt = "%" PRIi64; break;
//
//     case Type::FLOAT:  fmt = "%f"; break;
//     case Type::DOUBLE: fmt = "%f"; break;
//
//     default: return "?";
//     }
//
//     std::string tmp;
//     return appendFmt(tmp, fmt, value);
// }




template<class T>
std::ostream& formatBasicType(std::ostream& ss, const uint8_t* buffer, size_t length, size_t offset)
{
    if(offset+sizeof(T) > length)
        return ss << "?";

    return ss << microstrain::read<T>(buffer);
}

static std::optional<uint64_t> read(Type type, const uint8_t* buffer, size_t length, size_t offset)
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

std::ostream& format(std::ostream& ss, const EnumInfo* info, const uint8_t* buffer, size_t length, size_t offset=0)
{
    if(!info)
        return ss << "<enum>";

    std::optional<uint64_t> value = read(info->type, buffer, length, offset);

    if(!value)
        return ss << "?";

    auto it = std::find_if(
        info->entries.begin(), info->entries.end(),
        [value](const auto& entry){ return entry.value == value; }
    );
    const char* name = (it != info->entries.end()) ? it->name : "?";

    return ss << *value << '(' << name << ')';
}

std::ostream& format(std::ostream& ss, const BitfieldInfo* info, const uint8_t* buffer, size_t length, size_t offset)
{
    if(!info)
        return ss << "<bitfield>";

    std::optional<uint64_t> value = read(info->type, buffer, length, offset);
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

std::ostream& format(std::ostream& ss, const StructInfo* info, const uint8_t* buffer, size_t length, size_t offset=0)
{
    if(!info)
        return ss << "<struct>";

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
            format(
                ss, static_cast<const EnumInfo *>(param.type.infoPtr),
                buffer, length, param.byte_offset
            );
            break;

        case Type::BITFIELD:
            if(!param.type.infoPtr)
                ss << "<bits>";
            else if(param.byte_offset > length)
                ss << "?";
            else
            {
                format(
                    ss, static_cast<const BitfieldInfo *>(param.type.infoPtr),
                    buffer, length, param.byte_offset
                );
            }
            break;

        case Type::STRUCT:
            if(!param.type.infoPtr)
                ss << "<struct>";
            else if(param.byte_offset > length)
                ss << "{?}";
            else
            {
                format(
                    ss, static_cast<const StructInfo *>(param.type.infoPtr),
                    buffer, length, param.byte_offset
                );
            }
            break;

        case Type::UNION:
            ss << "<union>";
            break;
        }

        i++;
    }
    ss << '}';

    return ss;
}

std::ostream& format(std::ostream& ss, const mip::FieldView& field)
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
        format(ss, static_cast<const StructInfo*>(info), field.payload(), field.payloadLength());
    }
    return ss;
}

std::string formatField(const mip::FieldView& field)
{
    std::stringstream ss;

    format(ss, field);

    return ss.str();
}
