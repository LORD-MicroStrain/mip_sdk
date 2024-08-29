#pragma once

#include "mip_structures.hpp"

#include <ostream>

namespace mip { class PacketView; }

namespace mip::metadata
{
class DescriptorSet;

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
    virtual bool formatPacketBegin(const DescriptorSet* info, const PacketView& packet) = 0;
    virtual bool formatFieldBegin(const FieldInfo* info) = 0;
    virtual bool formatStructBegin(const StructInfo* info) = 0;
    virtual bool formatUnionBegin(const UnionInfo* info) = 0;
    virtual bool formatArrayBegin(unsigned int count) = 0;

    virtual void formatEnd(Type type) { (void)type; }

    virtual bool formatParam(const ParameterInfo* info) = 0;
    virtual void formatValue(Type type, BasicValue value) { (void)type; (void)value; }
    virtual void formatString(const char* ptr, unsigned int length) { (void)ptr; (void)length; }
    virtual void formatEnum(const EnumInfo* info, uint64_t rawValue) { (void)info; (void)rawValue; }
    virtual void formatBitfield(const BitfieldInfo* info, uint64_t rawValue) { (void)info; (void)rawValue; }

    virtual void formatSeparator() {}

    virtual void formatError(const ParameterInfo* info) { (void)info; }
};


class BasicFormatter : public Formatter
{
public:
    BasicFormatter(std::ostream& stream);

    bool formatPacketBegin(const DescriptorSet* info, const PacketView& packet) override;
    bool formatFieldBegin(const FieldInfo* info) override;
    bool formatStructBegin(const StructInfo* info) override;
    bool formatUnionBegin(const UnionInfo* info) override;
    bool formatArrayBegin(unsigned int count) override;

    void formatEnd(Type type) override;

    bool formatParam(const ParameterInfo* info) override;
    void formatValue(Type type, BasicValue value) override;
    void formatEnum(const EnumInfo* info, uint64_t rawValue) override;
    void formatBitfield(const BitfieldInfo* info, uint64_t rawValue) override;

    void formatSeparator() override;

    void formatError(const ParameterInfo* info) override;

    static const char* typeName(Type type);

private:
    std::ostream& mStream;
};



} // namespace mip::metadata
