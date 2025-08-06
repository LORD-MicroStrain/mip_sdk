#pragma once

#include "formatter.hpp"

#include <ostream>


namespace mip::printer
{

class BasicFormatter : public Formatter
{
public:
    BasicFormatter(std::ostream& stream);

    BasicFormatter(const BasicFormatter&) = delete;
    BasicFormatter& operator=(const BasicFormatter&) = delete;

    const std::ostream& stream() const { return mStream; }
    std::ostream& stream() { return mStream; }

    bool formatPacketBegin(const metadata::DescriptorSetInfo* info, const PacketView& packet) override;
    bool formatFieldBegin(const metadata::FieldInfo* info) override;
    bool formatStructBegin(const metadata::StructInfo* info) override;
    bool formatUnionBegin(const metadata::UnionInfo* info) override;
    bool formatArrayBegin(unsigned int count) override;

    void formatEnd(metadata::Type type) override;

    bool formatParam(const metadata::ParameterInfo* info) override;
    void formatValue(metadata::Type type, BasicValue value) override;
    void formatEnum(const metadata::EnumInfo* info, uint64_t rawValue) override;
    void formatBitfield(const metadata::BitfieldInfo* info, uint64_t rawValue) override;

    void formatSeparator() override;

    void formatError(const metadata::ParameterInfo* info) override;

    static const char* typeName(metadata::Type type);

private:
    std::ostream& mStream;
};


} // namespace mip::printer
