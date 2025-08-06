#pragma once

#include <mip/mip_serialization.hpp>
#include <mip/metadata/mip_definitions.hpp>


namespace mip
{
class PacketView;
class FieldView;
}

namespace mip::metadata
{
struct FieldInfo;
struct StructInfo;
struct UnionInfo;
struct ParameterInfo;
}

namespace mip::printer
{
class Formatter;
class DescriptorSet;


class PacketPrinter
{
public:
    PacketPrinter(Formatter& format) : mFormatter(format) {}

    PacketPrinter(const PacketPrinter&) = delete;
    PacketPrinter& operator=(const PacketPrinter&) = delete;

    Formatter& formatter() const { return mFormatter; }

    void formatPacket(const PacketView& packet);
    void formatField(const FieldView& field);

    void formatPacket(const PacketView& packet, const metadata::DescriptorSetSpan& mipdefs);
    void formatField(const FieldView& field, const metadata::DescriptorSetSpan& mipdefs);
    void formatField(const FieldView& field, const metadata::DescriptorSetInfo* descriptorSet);

private:
    void formatField(const metadata::FieldInfo* info, const FieldView& field);
    void formatStruct(const metadata::StructInfo* info, size_t offset_index);
    void formatStructContents(const metadata::StructInfo* info, size_t offset_index);
    void formatUnion(const metadata::UnionInfo* info, const metadata::StructInfo& parent, size_t offset_index);
    void formatParameter(const metadata::ParameterInfo& info, const metadata::StructInfo& parent, size_t offset_index);

private:
    mip::Serializer mSerializer;

    static inline constexpr size_t MAX_PARAMETERS = 20;
    std::array<uint8_t, MAX_PARAMETERS> mOffsets;

    Formatter& mFormatter;
};


} // namespace mip::metadata
