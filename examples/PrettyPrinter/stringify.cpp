

#include <mip/metadata/mip_all_definitions.hpp>

#include <mip/mip_field.hpp>

#include <array>
#include <bit>
#include <string>
#include <sstream>
#include <ostream>
#include <iomanip>
#include <optional>
#include <assert.h>

#if __cpp_lib_optional < 201606L
#error "Needs optional support"
#endif


struct Formatter
{
    static constexpr size_t MAX_PARAMETERS = 20;

    mip::Serializer serializer;
    std::ostream& ss;
    std::array<uint8_t, MAX_PARAMETERS> offsets;

    std::ostream& formatEnum(const mip::metadata::EnumInfo* info);
    std::ostream& formatBitfield(const mip::metadata::BitfieldInfo* info);
    std::ostream& formatUnion(const mip::metadata::UnionInfo* info, const mip::metadata::StructInfo& parent, size_t offset_index);
    std::ostream& formatStruct(const mip::metadata::StructInfo* info, size_t offset_index=0);

    void formatParameter(const mip::metadata::ParameterInfo& param, const mip::metadata::StructInfo& parent, size_t offset_index=0);

    template<class T>
    std::ostream& formatBasicType();
};


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
std::ostream& Formatter::formatBasicType()
{
    T value;
    if(!serializer.extract(value))
        return ss << "?";
    if constexpr(std::is_same_v<T,char>)
        return ss << value;
    else if constexpr(std::is_same_v<T,bool>)
        return ss << std::boolalpha << value;
    else
        return ss << (int)value;
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
static std::optional<uint64_t> readIntegralValue(mip::metadata::Type type, mip::Serializer& serializer)
{
    switch(type)
    {
    case mip::metadata::Type::U8:  return microstrain::extract< uint8_t>(serializer);
    case mip::metadata::Type::S8:  return microstrain::extract<  int8_t>(serializer);
    case mip::metadata::Type::U16: return microstrain::extract<uint16_t>(serializer);
    case mip::metadata::Type::S16: return microstrain::extract< int32_t>(serializer);
    case mip::metadata::Type::U32: return microstrain::extract<uint32_t>(serializer);
    case mip::metadata::Type::S32: return microstrain::extract< int64_t>(serializer);
    case mip::metadata::Type::U64: return microstrain::extract<uint64_t>(serializer);
    case mip::metadata::Type::S64: return microstrain::extract< int64_t>(serializer);
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
std::ostream& Formatter::formatEnum(const mip::metadata::EnumInfo* info)
{
    if(!info)
        return ss << "<enum>";

    std::optional<uint64_t> value = readIntegralValue(info->type, serializer);

    if(!value)
        return ss << "?";

    auto it = std::find_if(
        info->entries.begin(), info->entries.end(),
        [value](const mip::metadata::EnumInfo::Entry& entry){ return entry.value == value; }
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
std::ostream& Formatter::formatBitfield(const mip::metadata::BitfieldInfo* info)
{
    if(!info)
        return ss << "<bitfield>";

    std::optional<uint64_t> value = readIntegralValue(info->type, serializer);

    if(!value)
        return ss << "?";

    ss << '{';
    size_t i=0;
    for(const mip::metadata::EnumInfo::Entry& entry : info->entries)
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

void Formatter::formatParameter(const mip::metadata::ParameterInfo &param, const mip::metadata::StructInfo& parent, size_t offset_index)
{
    switch(param.type.type)
    {
    case mip::metadata::Type::NONE:
        ss << "-";
        break;

    case mip::metadata::Type::CHAR:   formatBasicType<char    >(); break;
    case mip::metadata::Type::BOOL:   formatBasicType<bool    >(); break;
    case mip::metadata::Type::U8:     formatBasicType<uint8_t >(); break;
    case mip::metadata::Type::S8:     formatBasicType< int8_t >(); break;
    case mip::metadata::Type::U16:    formatBasicType<uint16_t>(); break;
    case mip::metadata::Type::S16:    formatBasicType< int16_t>(); break;
    case mip::metadata::Type::U32:    formatBasicType<uint32_t>(); break;
    case mip::metadata::Type::S32:    formatBasicType< int32_t>(); break;
    case mip::metadata::Type::U64:    formatBasicType<uint64_t>(); break;
    case mip::metadata::Type::S64:    formatBasicType< int64_t>(); break;
    case mip::metadata::Type::FLOAT:  formatBasicType<float   >(); break;
    case mip::metadata::Type::DOUBLE: formatBasicType<double  >(); break;

    case mip::metadata::Type::ENUM:
        formatEnum(static_cast<const mip::metadata::EnumInfo *>(param.type.infoPtr));
        break;

    case mip::metadata::Type::BITS:
        formatBitfield(static_cast<const mip::metadata::BitfieldInfo *>(param.type.infoPtr));
        break;

    case mip::metadata::Type::STRUCT:
        formatStruct(static_cast<const mip::metadata::StructInfo *>(param.type.infoPtr), offset_index);
        break;

    case mip::metadata::Type::UNION:
        formatUnion(static_cast<const mip::metadata::UnionInfo *>(param.type.infoPtr), parent, offset_index);
        break;
    }
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
std::ostream& Formatter::formatUnion(const mip::metadata::UnionInfo* info, const mip::metadata::StructInfo& parent, size_t offset_index)
{
    if(!info)
        return ss << "<union>";
    if(serializer.isOverrun())
        return ss << "?";

    ss << info->name << '{';

    for(const mip::metadata::ParameterInfo& param : info->parameters)
    {
        // Enum condition, where the active union member depends on a previous parameter's value.
        if(param.condition.type == mip::metadata::ParameterInfo::Condition::Type::ENUM)
        {
            // Parameter index is within the parent's parameter array.
            assert(param.condition.paramIdx.isValid(parent.parameters.size()));
            const mip::metadata::ParameterInfo &discriminant = parent.parameters[param.condition.paramIdx.index()];

            // Index is within the offset array relative to the parent.
            assert(offset_index+param.condition.paramIdx.index() < MAX_PARAMETERS);
            const uint8_t offset = offsets[offset_index+param.condition.paramIdx.index()];

            // Read value from serializer (jump back to the correct parameter offset, then back).
            const size_t oldOffset = serializer.setOffset(offset);
            std::optional<uint64_t> value = readIntegralValue(discriminant.type.type, serializer);
            serializer.setOffset(oldOffset);

            // If value was read successfully and it matches the condition value then display it.
            if(value.has_value() && *value == param.condition.value)
            {
                formatParameter(param, parent, offset_index);
            }
        }
    }

    ss << '}';
    return ss;
}

//static constexpr inline size_t MAX_PARAM_REFS = 5;
//struct Offset
//{
//    microstrain::Index paramId = {};
//    size_t             offset  = 0;
//};

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
std::ostream& Formatter::formatStruct(const mip::metadata::StructInfo* info, size_t offset_index)
{
    if(!info)
        return ss << "<struct>";
    if(serializer.isOverrun())
        return ss << "?";

    // Keep track of the offset of each parameter so we can go back and look up array sizes, etc.
    assert(info->parameters.size() <= MAX_PARAMETERS);  // Increase MAX_PARAMETERS if this trips.

    //std::array<Offset, MAX_PARAM_REFS> offsets;
    //size_t nextOffsetIndex = 0;
    //microstrain::Index firstVariadicParam;

    //const size_t baseOffset = serializer.offset();

    // Entry in 'offsets' representing the first parameter of this struct.
    assert(offset_index + info->parameters.size() <= MAX_PARAMETERS);
    // size_t base_index = offset_index;
    // num_offsets += info->parameters.size();
    // Ensure there are enough entries in the offsets array.

    ss << '{';

    for(size_t i=0; i<info->parameters.size(); i++)
    {
        const mip::metadata::ParameterInfo& param = info->parameters[i];
        if(i > 0)
            ss << ", ";

        ss << param.name << '=';

        // Save offset of each parameter in case it's an array count or union discriminator.
        // This will clobber the offsets of nested structs (within this one), but those
        // offsets won't be needed again. Only basic types can be counters or discriminators
        // so only those parameters might need to be read later on.
        offsets[offset_index+i] = (uint8_t)serializer.offset();

        uint8_t count = param.count.count;
        if(param.count.hasCounter())
        {
            assert(param.count.paramIdx.isValid(i));  // Array size can't come after the array

            const mip::metadata::ParameterInfo& counter = info->parameters[param.count.paramIdx.index()];
            const size_t counterOffset = offsets[offset_index+param.count.paramIdx.index()];

            // Counters must be arithmetic types and can't be arrays.
            assert(counter.type.isBasicType());
            assert(counter.count.count == 1 && !counter.count.hasCounter());

            const size_t oldOffset = serializer.setOffset(counterOffset);
            std::optional<uint64_t> counterValue = readIntegralValue(counter.type.type, serializer);
            serializer.setOffset(oldOffset);

            if(!counterValue)
            {
                serializer.invalidate();
                ss << "[?]";
                continue;
            }

            count = (uint8_t)*counterValue;
        }
        if(param.count.count != 1)
            ss << (param.type.type == mip::metadata::Type::CHAR ? '"' : '[');

        for(uint8_t j=0; j<count || (count==0 && serializer.isOk()); j++)
        {
            if(j > 0 && param.type.type != mip::metadata::Type::CHAR)
                ss << ", ";

            formatParameter(param, *info, offset_index+i);
        }
        if(param.count.count != 1)
            ss << (param.type.type == mip::metadata::Type::CHAR ? '"' : ']');
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
    const mip::metadata::FieldInfo* info = mip::metadata::findField(mip::metadata::ALL_DESCRIPTOR_SETS, field.descriptor());

    if(!info)
    {
        ss << std::showbase << std::hex << std::setfill('0');
        ss << "Unknown(" << std::setw(2) << (int)field.descriptorSet() << "," << std::setw(2) << (int)field.fieldDescriptor() << ")[" << std::noshowbase;
        for(uint8_t i=0; i<field.payloadLength(); i++)
            ss << std::setw(2) << (int)field.payload(i);
        ss << ']';
    }
    else
    {
        ss << info->name;

        Formatter formatter = {
            mip::Serializer(field.payload(), field.payloadLength()),
            ss,
        };

        formatter.formatStruct(static_cast<const mip::metadata::StructInfo *>(info));
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
