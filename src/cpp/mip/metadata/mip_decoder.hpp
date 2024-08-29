#pragma once

#include "mip_structures.hpp"

#include <mip/mip_serialization.hpp>

#include <ostream>

namespace mip
{
class PacketView;
class FieldView;
}

namespace mip::metadata
{
class Formatter;
class Definitions;
class DescriptorSet;


class FieldByteFormatter // : public FieldVisitor
{
public:
    FieldByteFormatter(Formatter& format) : mFormatter(format) {}

    void formatPacket(const PacketView& packet, const Definitions& mipdefs);
    void formatField(const FieldView& field, const Definitions& mipdefs);
    void formatField(const FieldView& field, const DescriptorSet* descriptorSet);

private:
    void formatField(const FieldInfo* info, const FieldView& field);
    void formatStruct(const StructInfo* info, size_t offset_index);
    void formatStructContents(const StructInfo* info, size_t offset_index);
    void formatUnion(const UnionInfo* info, const StructInfo& parent, size_t offset_index);
    void formatParameter(const ParameterInfo& info, const StructInfo& parent, size_t offset_index);

private:
    mip::Serializer mSerializer;

    static inline constexpr size_t MAX_PARAMETERS = 20;
    std::array<uint8_t, MAX_PARAMETERS> mOffsets;

    Formatter& mFormatter;
};


} // namespace mip::metadata
