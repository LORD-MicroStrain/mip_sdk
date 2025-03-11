#pragma once

#include <microstrain/span.hpp>
#include <mip/mip_descriptors.hpp>


namespace mip::metadata
{

struct FieldInfo;
struct DescriptorSetInfo;

// A list of const DescriptorSetInfo pointers.
// This is a view of an array of const pointers to const DescriptorSetInfos.
using DescriptorSetSpan = microstrain::Span<DescriptorSetInfo const* const>;

const DescriptorSetInfo* findDescriptorSet(const DescriptorSetSpan& descriptorSets, uint8_t descriptor);
const FieldInfo* findField(const DescriptorSetSpan& descriptorSets, mip::CompositeDescriptor descriptor);
const FieldInfo* findField(const DescriptorSetInfo& ds_info, uint8_t field_desc, bool check_shared_data=true);

} // namespace mip::metadata
