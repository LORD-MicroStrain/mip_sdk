
#include "basic_formatter.hpp"

#include <mip/mip_packet.hpp>
#include <mip/metadata/mip_all_definitions.hpp>

#include <algorithm>
#include <bit>
#include <iomanip>
#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Skips the "descriptor_set::" prefix on field names, if it exists.
///
static const char* fieldName(const char* qualifiedFieldName)
{
    const char* p=qualifiedFieldName;

    // Search forward for ':'
    while(*p != ':')
    {
        if(*p == '\0')
            return qualifiedFieldName;
        p++;
    }

    // Continuing search for NOT ':'
    while(*(++p) == ':')
    {
        if(*p == '\0')
            return qualifiedFieldName;
    }

    return p;
}


namespace mip::printer
{

using namespace mip::metadata;


////////////////////////////////////////////////////////////////////////////////
///@brief Constructs a formatter which writes to the give ostream.
///
///@param stream All formatted text is written to this stream.
///
BasicFormatter::BasicFormatter(std::ostream &stream) : mStream(stream)
{
    stream << std::boolalpha << std::noshowbase;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Begins formatting a packet by printing the descriptor set.
///
///@param info   Descriptor set definition (may be NULL).
///@param packet Mip packet details.
///
///@returns True to continue formatting the packet's contents.
///
bool BasicFormatter::formatPacketBegin(const DescriptorSetInfo* info, const PacketView& packet)
{
    if(info)
        mStream << info->name << " [ ";
    else
        mStream << "Unknown(0x" << std::hex << (int)packet.descriptorSet() << std::dec << ')';

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Begins formatting a field by printing its name and descriptors.
///
///@param info Field metadata (may be NULL).
///
///@returns true to also format the contents.
///
bool BasicFormatter::formatFieldBegin(const FieldInfo* info)
{
    // Todo: print descriptors (even if NULL?)
    if(info)
        mStream << fieldName(info->name) << '{';
    else
        mStream << "<field>";

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Begins formatting a struct by printing its name and opening brace.
///
///@param info    Struct metadata (may be NULL).
///
///@returns true to also format the contents.
///
bool BasicFormatter::formatStructBegin(const StructInfo* info)
{
    if(info)
        mStream << info->name << '(';
    else
        mStream << "<struct>";

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Begins formatting a union by printing its name and opening brace.
///
///@param info Union metadata (may be NULL).
///
///@returns true to also format the contents.
///
bool BasicFormatter::formatUnionBegin(const UnionInfo* info)
{
    if(info)
        mStream << info->name << '(';
    else
        mStream << "<union>";

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Begins formatting an array by printing the opening bracket.
///
///@param count Number of array elements to follow.
///
///@returns true to also format the contents.
///
bool BasicFormatter::formatArrayBegin(unsigned int count)
{
    (void)count;

    mStream << '[';
    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Formats a parameter name, excluding its value
///
///@param info Parameter metadata such as name.
///
///@returns true to also format the value.
///
bool BasicFormatter::formatParam(const ParameterInfo* info)
{
    if(info)
        mStream << info->name << '=';  // Value to follow via formatXXX
    else
        mStream << "?=";

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Indicates the end of content for a given field/struct/union/array.
///
/// This is only called if the corresponding formatXxxBegin function has
/// returned true and the metadata for this element is not NULL.
///
///@param type Type of the element that has been formatted. NULL for arrays and fields.
///
void BasicFormatter::formatEnd(Type type)
{
    // info is NULL when ending an array.
    switch(type)
    {
    case Type::FIELD:
        mStream << '}';
        break;

    case Type::STRUCT:
    case Type::UNION:
    case Type::BITS:
        mStream << ')';
        break;

    default:  // Fields, Arrays
        mStream << " ]";
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Adds a separator between array or field elements.
///
void BasicFormatter::formatSeparator()
{
    mStream << ", ";
}

////////////////////////////////////////////////////////////////////////////////
///@brief Formats an arithmetic value (including bool and char).
///
///@param type Type of the value. Always a basic type.
///@param value Value of the corresponding type.
///
void BasicFormatter::formatValue(Type type, BasicValue value)
{
    switch(type)
    {
    case Type::CHAR:   mStream << value.c;       break;
    case Type::BOOL:   mStream << value.b;       break;
    case Type::U8:     mStream << (int)value.u8; break;
    case Type::S8:     mStream << (int)value.s8; break;
    case Type::U16:    mStream << value.u16;     break;
    case Type::S16:    mStream << value.s16;     break;
    case Type::U32:    mStream << value.u32;     break;
    case Type::S32:    mStream << value.s32;     break;
    case Type::U64:    mStream << value.u64;     break;
    case Type::S64:    mStream << value.s64;     break;
    case Type::FLOAT:  mStream << value.f;       break;
    case Type::DOUBLE: mStream << value.d;       break;
    default: assert(false); mStream << '?';      break;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Formats an enumerated value, including the name and numeric value.
///
///@param info     Enum metadata (may be NULL).
///@param rawValue Numeric value of the enumeration.
///
void BasicFormatter::formatEnum(const EnumInfo* info, uint64_t rawValue)
{
    if(!info)
    {
        mStream << "<enum>";
        return;
    }

    auto it = std::find_if(
        info->entries.begin(), info->entries.end(),
        [rawValue](const auto& entry){ return entry.value == rawValue; }
    );
    const char* name = (it != info->entries.end()) ? it->name : "?";

    mStream << info->name << '{' << rawValue << '(' << name << ")}";
}


////////////////////////////////////////////////////////////////////////////////
///@brief Formats a bitfield, including the numeric value and all named fields.
///
///@param info     Bitfield metadata (may be NULL).
///@param rawValue Numeric value of the bitfield.
///
void BasicFormatter::formatBitfield(const BitfieldInfo* info, uint64_t rawValue)
{
    if(!info)
    {
        mStream << "<bitfield>";
        return;
    }

    mStream << "(0x";

    auto mode = mStream.flags() & std::ios_base::basefield;
    mStream.setf(std::ios_base::hex, std::ios_base::basefield);
    mStream << rawValue;
    mStream.setf(mode, std::ios_base::basefield);

    formatSeparator();

    size_t i=0;
    for(const auto& entry : info->entries)
    {
        if(i > 0)
            formatSeparator();

        mStream << entry.name << '=';

        const size_t firstBit = std::countl_zero(entry.value);
        mStream << ((rawValue & entry.value) >> firstBit);

        i++;
    }

    mStream << ')';
}

////////////////////////////////////////////////////////////////////////////////
///@brief Called when the corresponding parameter cannot be read or accessed.
///
///@param info Metadata for the parameter that couldn't be read (may be NULL).
///
void BasicFormatter::formatError(const ParameterInfo* info)
{
    if(!info)
        mStream << "<unknown>";
    else
        mStream << '<' << typeName(info->type.type) << '>';
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the name of a (basic) type.
///
///@param type
///
const char* BasicFormatter::typeName(Type type)
{
    switch(type)
    {
    default: assert(false); // Fallthrough
    case Type::NONE:     return "none";

    case Type::CHAR:     return "char";
    case Type::BOOL:     return "bool";

    case Type::U8:       return "u8";
    case Type::S8:       return "s8";
    case Type::U16:      return "u16";
    case Type::S16:      return "s16";
    case Type::U32:      return "u32";
    case Type::S32:      return "s32";
    case Type::U64:      return "u64";
    case Type::S64:      return "s64";
    case Type::FLOAT:    return "float";
    case Type::DOUBLE:   return "double";

    case Type::ENUM:     return "enum";
    case Type::BITS:     return "bitfield";
    case Type::STRUCT:   return "struct";
    case Type::UNION:    return "union";
    }
}


} // namespace mip::printer
