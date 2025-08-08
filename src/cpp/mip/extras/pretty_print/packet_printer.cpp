
#include "packet_printer.hpp"

#include "formatter.hpp"

#include <mip/mip_packet.hpp>
#include <mip/mip_field.hpp>
#include <mip/metadata/definitions/data_shared.hpp>
#include <mip/metadata/mip_all_definitions_runtime.hpp>

#include <cstdio>
#include <optional>
#include <assert.h>


namespace mip::printer
{

using namespace mip::metadata;

static std::optional<Formatter::BasicValue> readBasicValue(Serializer& serializer, Type type);
static std::optional<uint64_t> readIntegralValue(Serializer& serializer, Type type);


void PacketPrinter::formatPacket(const mip::PacketView& packet)
{
    return formatPacket(packet, mip::metadata::allDescriptorSets());
}

void PacketPrinter::formatField(const mip::FieldView& field)
{
    return formatField(field, mip::metadata::allDescriptorSets());
}

void PacketPrinter::formatPacket(const PacketView& packet, const DescriptorSetSpan& mipdefs)
{
    const DescriptorSetInfo* info = findDescriptorSet(mipdefs, packet.descriptorSet());

    if(!mFormatter.formatPacketBegin(info, packet) || !info)
        return;

    size_t i=0;
    for(mip::FieldView field : packet)
    {
        if(i>0)
            mFormatter.formatSeparator();

        formatField(field, info);

        i++;
    }

    mFormatter.formatEnd(Type::NONE);
}

void PacketPrinter::formatField(const mip::FieldView &field, const DescriptorSetSpan &mipdefs)
{
    const FieldInfo* info = findField(mipdefs, field.descriptor());

    formatField(info, field);
}

void PacketPrinter::formatField(const mip::FieldView &field, const DescriptorSetInfo* descriptorSet)
{
    const FieldInfo* info = nullptr;

    if(descriptorSet)
        info = findField(*descriptorSet, field.fieldDescriptor());

    if(!info && isDataDescriptorSet(field.descriptorSet()))
        info = findField(DATA_SHARED, field.fieldDescriptor());

    formatField(info, field);
}

void PacketPrinter::formatField(const FieldInfo* info, const FieldView& field)
{
    if( mFormatter.formatFieldBegin(info) && info )
    {
        mSerializer = Serializer(field.payload(), field.payloadLength());

        formatStructContents(info, 0);

        mFormatter.formatEnd(Type::FIELD);
    }
}

void PacketPrinter::formatStruct(const StructInfo* info, size_t offset_index)
{
    if( mFormatter.formatStructBegin(info) && info )
    {
        formatStructContents(info, offset_index);

        mFormatter.formatEnd(Type::STRUCT);
    }
}

void PacketPrinter::formatStructContents(const StructInfo* info, size_t offset_index)
{
    assert(info);

    // Keep track of the offset of each parameter, so we can go back and look up array sizes, etc.
    assert(info->parameters.size() <= MAX_PARAMETERS);  // Increase MAX_PARAMETERS if this trips.

    // Entry in 'offsets' representing the first parameter of this struct.
    assert(offset_index + info->parameters.size() <= MAX_PARAMETERS);

    for(size_t i=0; i<info->parameters.size(); i++)
    {
        const auto& param = info->parameters[i];
        if(i > 0)
            mFormatter.formatSeparator();

        mFormatter.formatParam(&param);

        // Save offset of each parameter in case it's an array count or union discriminator.
        // This will clobber the offsets of nested structs (within this one), but those
        // offsets won't be needed again. Only basic types can be counters or discriminators
        // so only those parameters might need to be read later on.
        mOffsets[offset_index+i] = (uint8_t)mSerializer.offset();

        uint8_t count = param.count.count;
        if(param.count.hasCounter())
        {
            assert(param.count.count == 0);  // This code assumes count=0 <==> has_counter.

            assert(param.count.paramIdx.isValid(i));  // Array size can't come after the array

            const ParameterInfo& counter = info->parameters[param.count.paramIdx.index()];
            const size_t counterOffset = mOffsets[offset_index+param.count.paramIdx.index()];

            // Counters must be arithmetic types and can't be arrays.
            assert(counter.type.isBasicType());
            assert(counter.count.count == 1 && !counter.count.hasCounter());

            const size_t oldOffset = mSerializer.setOffset(counterOffset);
            std::optional<uint64_t> counterValue = readIntegralValue(mSerializer, counter.type.type);
            mSerializer.setOffset(oldOffset);

            if(!counterValue)
            {
                mSerializer.invalidate();
                mFormatter.formatError(&counter);
            }

            count = (uint8_t)*counterValue;
        }

        // Check if this is a character string (no NULL terminator!).
        if(param.type.type == Type::CHAR && count != 1)
        {
            const uint8_t* ptr = mSerializer.getPointer(count);
            if(ptr)
                mFormatter.formatString(reinterpret_cast<const char*>(ptr), count);
            else
                mFormatter.formatError(&param);
        }
        else  // Not a string (either an array of non-characters or not an array)
        {
            if(param.count.count != 1)
                mFormatter.formatArrayBegin(param.count.count);

            for(uint8_t j=0; j<count || (count==0 && mSerializer.isOk()); j++)
            {
                if(j > 0)
                    mFormatter.formatSeparator();

                formatParameter(param, *info, offset_index+i);
            }

            // End of array?
            if(param.count.count != 1)
                mFormatter.formatEnd(Type::NONE);
        }
    }
}

void PacketPrinter::formatUnion(const UnionInfo* info, const StructInfo& parent, size_t offset_index)
{
    if( mFormatter.formatUnionBegin(info) && info )
    {
        for(const auto& param : info->parameters)
        {
            // Enum condition, where the active union member depends on a previous parameter's value.
            if(param.condition.type == ParameterInfo::Condition::Type::ENUM)
            {
                // Parameter index is within the parent's parameter array.
                assert(param.condition.paramIdx.isValid(parent.parameters.size()));
                const ParameterInfo &discriminator = parent.parameters[param.condition.paramIdx.index()];

                // Index is within the offset array relative to the parent.
                assert(offset_index+param.condition.paramIdx.index() < MAX_PARAMETERS);
                const uint8_t offset = mOffsets[offset_index+param.condition.paramIdx.index()];

                // Read value from serializer (jump back to the correct parameter offset, then back).
                const size_t oldOffset = mSerializer.setOffset(offset);
                std::optional<uint64_t> value = readIntegralValue(mSerializer, discriminator.type.type);
                mSerializer.setOffset(oldOffset);

                // If value was read successfully, and it matches the condition value, then display it.
                if(value.has_value() && *value == param.condition.value)
                {
                    formatParameter(param, parent, offset_index);
                }
            }
        }

        mFormatter.formatEnd(Type::UNION);
    }
}

void PacketPrinter::formatParameter(const ParameterInfo& param, const StructInfo& parent, size_t offset_index)
{
    switch(param.type.type)
    {
    case Type::NONE:
    case Type::CHAR:
    case Type::BOOL:
    case Type::U8:
    case Type::S8:
    case Type::U16:
    case Type::S16:
    case Type::U32:
    case Type::S32:
    case Type::U64:
    case Type::S64:
    case Type::FLOAT:
    case Type::DOUBLE:
        if(auto value = readBasicValue(mSerializer, param.type.type))
            mFormatter.formatValue(param.type.type, *value);
        else
            mFormatter.formatError(&param);
        break;

    case Type::ENUM:
        if(auto* info = static_cast<const EnumInfo *>(param.type.infoPtr))
        {
            if(auto value = readIntegralValue(mSerializer, info->type))
                mFormatter.formatEnum(info, *value);
            else
                mFormatter.formatError(&param);
        }
        else
            mFormatter.formatEnum(nullptr, 0);
        break;

    case Type::BITS:
        if(auto* info = static_cast<const BitfieldInfo*>(param.type.infoPtr))
        {
            if(auto value = readIntegralValue(mSerializer, info->type))
                mFormatter.formatBitfield(info, *value);
            else
                mFormatter.formatError(&param);
        }
        else
            mFormatter.formatBitfield(nullptr, 0);
        break;

    case Type::STRUCT:
        if(auto* info = static_cast<const StructInfo*>(param.type.infoPtr))
            formatStruct(info, offset_index);
        else
            mFormatter.formatStructBegin(nullptr);
        break;

    case Type::UNION:
        if(auto* info = static_cast<const UnionInfo*>(param.type.infoPtr))
        formatUnion(info, parent, offset_index);
        break;

    case Type::FIELD:  // Will never happen
        break;
    }
}


static std::optional<Formatter::BasicValue> readBasicValue(Serializer& serializer, Type type)
{
    Formatter::BasicValue value;
    bool valid;

    switch(type)
    {
    case Type::NONE:   valid = true; break;
    case Type::CHAR:   valid = serializer.extract(value.c); break;
    case Type::BOOL:   valid = serializer.extract(value.b); break;
    case Type::U8:     valid = serializer.extract(value.u8); break;
    case Type::S8:     valid = serializer.extract(value.s8); break;
    case Type::U16:    valid = serializer.extract(value.u16); break;
    case Type::S16:    valid = serializer.extract(value.s16); break;
    case Type::U32:    valid = serializer.extract(value.u32); break;
    case Type::S32:    valid = serializer.extract(value.s32); break;
    case Type::U64:    valid = serializer.extract(value.u64); break;
    case Type::S64:    valid = serializer.extract(value.s64); break;
    case Type::FLOAT:  valid = serializer.extract(value.f); break;
    case Type::DOUBLE: valid = serializer.extract(value.d); break;
    default:           valid = false; break;
    }

    return valid ? std::make_optional(value) : std::nullopt;
}

static std::optional<uint64_t> readIntegralValue(Serializer& serializer, Type type)
{
    switch(type)
    {
    case Type::U8:  return microstrain::extract< uint8_t>(serializer);
    case Type::S8:  return microstrain::extract<  int8_t>(serializer);
    case Type::U16: return microstrain::extract<uint16_t>(serializer);
    case Type::S16: return microstrain::extract< int32_t>(serializer);
    case Type::U32: return microstrain::extract<uint32_t>(serializer);
    case Type::S32: return microstrain::extract< int64_t>(serializer);
    case Type::U64: return microstrain::extract<uint64_t>(serializer);
    case Type::S64: return microstrain::extract< int64_t>(serializer);
    default: return std::nullopt;
    }
}

} // namespace mip::metadata
