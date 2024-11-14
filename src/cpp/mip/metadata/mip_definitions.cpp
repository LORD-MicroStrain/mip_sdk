
#include "mip_definitions.hpp"

#include "mip_metadata.hpp"

#include <mip/metadata/definitions/data_shared.hpp>
#include <mip/metadata/definitions/common.hpp>


namespace mip::metadata
{

const DescriptorSetInfo* findDescriptorSet(const DescriptorSetSpan& descriptorSets, uint8_t descriptor)
{
    for(const DescriptorSetInfo* desc_set_info : descriptorSets)
    {
        if(desc_set_info->descriptor == descriptor)
            return desc_set_info;
    }
    return nullptr;
}

const FieldInfo* findField(const DescriptorSetSpan& descriptorSets, mip::CompositeDescriptor descriptor)
{
    if(mip::isDataDescriptorSet(descriptor.descriptorSet))
    {
        // Special case for shared data
        if(mip::isSharedDataFieldDescriptor(descriptor.fieldDescriptor))
        {
            return findField(DATA_SHARED, descriptor.fieldDescriptor);
        }
    }
    else if(isCommandDescriptorSet(descriptor.descriptorSet))
    {
        // Special case for reply fields
        if(isCommandDescriptorSet(descriptor.descriptorSet) && isReplyFieldDescriptor(descriptor.fieldDescriptor))
            return &MetadataFor<ReplyField>::value;
    }

    const DescriptorSetInfo* ds_info = findDescriptorSet(descriptorSets, descriptor.descriptorSet);
    if(!ds_info)
        return nullptr;

    if(const FieldInfo* info = findField(*ds_info, descriptor.fieldDescriptor))
        return info;

    return nullptr;
}

const FieldInfo* findField(const DescriptorSetInfo& ds_info, uint8_t field_desc)
{
    // Binary search assumes fields are sorted by descriptor.
    auto it = std::lower_bound(
        ds_info.fields.begin(), ds_info.fields.end(), field_desc,
        [](const FieldInfo* a, uint8_t b){ return a->descriptor.fieldDescriptor < b; }
    );

    return (it != ds_info.fields.end()) && ((*it)->descriptor.fieldDescriptor==field_desc) ? *it : nullptr;
}


} // namespace mip::metadata
