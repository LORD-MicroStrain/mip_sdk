#pragma once

#include <mip/metadata/mip_structures.hpp>

#include <stdint.h>


namespace mip { class PacketView; }


namespace mip::printer
{

class Formatter
{
public:
    union BasicValue
    {
        char     c;
        bool     b;
        uint8_t  u8;
        int8_t   s8;
        uint16_t u16;
        int16_t  s16;
        uint32_t u32;
        int32_t  s32;
        uint64_t u64;
        int64_t  s64;
        float    f;
        double   d;
    };

    // Called when iterating into a field/struct/union.
    // Info pointer may be NULL if not recognized.
    // Return true to step into, false to skip over.
    virtual bool formatPacketBegin(const metadata::DescriptorSetInfo* info, const PacketView& packet) = 0;
    virtual bool formatFieldBegin(const metadata::FieldInfo* info) = 0;
    virtual bool formatStructBegin(const metadata::StructInfo* info) = 0;
    virtual bool formatUnionBegin(const metadata::UnionInfo* info) = 0;
    virtual bool formatArrayBegin(unsigned int count) = 0;

    virtual void formatEnd(metadata::Type type) { (void)type; }

    virtual bool formatParam(const metadata::ParameterInfo* info) = 0;
    virtual void formatValue(metadata::Type type, BasicValue value) { (void)type; (void)value; }
    virtual void formatString(const char* ptr, unsigned int length) { (void)ptr; (void)length; }
    virtual void formatEnum(const metadata::EnumInfo* info, uint64_t rawValue) { (void)info; (void)rawValue; }
    virtual void formatBitfield(const metadata::BitfieldInfo* info, uint64_t rawValue) { (void)info; (void)rawValue; }

    virtual void formatSeparator() {}

    virtual void formatError(const metadata::ParameterInfo* info) { (void)info; }
};


} // namespace mip::printer
