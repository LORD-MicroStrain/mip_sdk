#pragma once

#include <microstrain/common/span.hpp>
#include <mip/mip_descriptors.hpp>


namespace mip::metadata
{

struct FieldInfo;
struct DescriptorSetInfo;

using DescriptorSetSpan = microstrain::Span<const DescriptorSetInfo* const>;

const DescriptorSetInfo* findDescriptorSet(const DescriptorSetSpan& descriptorSets, uint8_t descriptor);
const FieldInfo* findField(const DescriptorSetSpan& descriptorSets, mip::CompositeDescriptor descriptor);
const FieldInfo* findField(const DescriptorSetInfo& ds_info, uint8_t field_desc);

} // namespace mip::metadata
