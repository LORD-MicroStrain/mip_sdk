
#include "mip_format_struct.hpp"

#include "mip_meta_utils.hpp"

#include <assert.h>


namespace mip::metadata
{

static uint64_t readIntegralValue(const void* ptr, Type type);


void StructFormatter::format(const FieldInfo &info, const void *ptr)
{
    if( !mFormatter.formatFieldBegin(&info) )
        return;

    formatStructContents(info, ptr);

    mFormatter.formatEnd(Type::STRUCT);
}

void StructFormatter::formatStructContents(const StructInfo &info, const void *ptr)
{
    size_t p=0;
    for(const ParameterInfo& param : info.parameters)
    {
        if(p>0)
            mFormatter.formatSeparator();

        size_t count = param.count.count;
        if(param.count.hasCounter())
        {
            assert(param.count.count == 0);  // This code assumes count=0 <=> has_counter.

            assert(param.count.paramIdx.isValid(p));  // Array size can't come after the array

            const ParameterInfo& counter = info.parameters[param.count.paramIdx.index()];

            const void* counterPtr = counter.accessor(const_cast<void*>(ptr));
            count = readIntegralValue(counterPtr, counter.type.type);
        }

        const void* value = param.accessor(const_cast<void*>(ptr));

        if(param.type.type == Type::CHAR && count != 1)
        {
            mFormatter.formatString(reinterpret_cast<const char*>(value), count);
        }
        else // Not a string (either an array of non-characeters or not an array)
        {
            if(count != 1)
            {
                if( !mFormatter.formatArrayBegin(count) )
                    count = 0;
            }

            for(size_t j=0; j<count; j++)
            {
                if(j>0)
                    mFormatter.formatSeparator();

                formatParameter(param, value);
            }

            if(count != 1)
                mFormatter.formatEnd(Type::NONE);
        }

        p++;
    }
}


void StructFormatter::formatParameter(const ParameterInfo& param, const void *ptr)
{
    switch(param.type.type)
    {
    case Type::NONE: break;
    case Type::CHAR:   mFormatter.formatValue(param.type.type, {.c   = *reinterpret_cast<const char*    >(ptr)}); break;
    case Type::BOOL:   mFormatter.formatValue(param.type.type, {.b   = *reinterpret_cast<const bool*    >(ptr)}); break;
    case Type::U8:     mFormatter.formatValue(param.type.type, {.u8  = *reinterpret_cast<const uint8_t *>(ptr)}); break;
    case Type::S8:     mFormatter.formatValue(param.type.type, {.s8  = *reinterpret_cast<const  int8_t *>(ptr)}); break;
    case Type::U16:    mFormatter.formatValue(param.type.type, {.u16 = *reinterpret_cast<const uint16_t*>(ptr)}); break;
    case Type::S16:    mFormatter.formatValue(param.type.type, {.s16 = *reinterpret_cast<const  int16_t*>(ptr)}); break;
    case Type::U32:    mFormatter.formatValue(param.type.type, {.u32 = *reinterpret_cast<const uint32_t*>(ptr)}); break;
    case Type::S32:    mFormatter.formatValue(param.type.type, {.s32 = *reinterpret_cast<const  int32_t*>(ptr)}); break;
    case Type::U64:    mFormatter.formatValue(param.type.type, {.u64 = *reinterpret_cast<const uint64_t*>(ptr)}); break;
    case Type::S64:    mFormatter.formatValue(param.type.type, {.s64 = *reinterpret_cast<const  int64_t*>(ptr)}); break;
    case Type::FLOAT:  mFormatter.formatValue(param.type.type, {.f   = *reinterpret_cast<const float*   >(ptr)}); break;
    case Type::DOUBLE: mFormatter.formatValue(param.type.type, {.d   = *reinterpret_cast<const double*  >(ptr)}); break;

    case Type::ENUM:
        if(auto info = static_cast<const EnumInfo*>(param.type.infoPtr))
            mFormatter.formatEnum(info, readIntegralValue(ptr, info->type));
        else
            mFormatter.formatEnum(nullptr, 0);
        break;

    case Type::BITFIELD:
        if(auto info = static_cast<const BitfieldInfo*>(param.type.infoPtr))
            mFormatter.formatBitfield(info, readIntegralValue(ptr, info->type));
        else
            mFormatter.formatBitfield(nullptr, 0);
        break;

    case Type::STRUCT:
        if(auto* info = static_cast<const StructInfo*>(param.type.infoPtr))
            format(*info, ptr);
        else
            mFormatter.formatStructBegin(nullptr);
        break;

    case Type::UNION:
        if(auto* info = static_cast<const UnionInfo*>(param.type.infoPtr))
            format(*info, parent, ptr);
        else
            mFormatter.formatUnionBegin(nullptr);
        break;
    }
}


static uint64_t readIntegralValue(const void* ptr, Type type)
{
    switch(type)
    {
    case Type::U8:  return *reinterpret_cast<const uint8_t *>(ptr);
    case Type::S8:  return *reinterpret_cast<const  int8_t *>(ptr);
    case Type::U16: return *reinterpret_cast<const uint16_t*>(ptr);
    case Type::S16: return *reinterpret_cast<const  int16_t*>(ptr);
    case Type::U32: return *reinterpret_cast<const uint32_t*>(ptr);
    case Type::S32: return *reinterpret_cast<const  int32_t*>(ptr);
    case Type::U64: return *reinterpret_cast<const uint64_t*>(ptr);
    case Type::S64: return *reinterpret_cast<const  int64_t*>(ptr);
    default: return 0;
    }
}

} // namespace mip::metadata
