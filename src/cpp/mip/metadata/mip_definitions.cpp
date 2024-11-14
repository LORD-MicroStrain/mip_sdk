
#include "mip_definitions.hpp"

#include "mip_metadata.hpp"

#include <mip/metadata/definitions/data_shared.hpp>
#include <mip/metadata/definitions/common.hpp>


namespace mip::metadata
{

////////////////////////////////////////////////////////////////////////////////
///@brief Searches for a descriptor set in a list of descriptor sets.
///
///@param descriptorSets List of descriptor sets to search.
///@param descriptor     Descriptor set ID number.
///
///@returns A pointer to the descriptor set metadata if found.
///@returns NULL if the specified ID isn't in the list.
///
const DescriptorSetInfo* findDescriptorSet(const DescriptorSetSpan& descriptorSets, uint8_t descriptor)
{
    for(const DescriptorSetInfo* desc_set_info : descriptorSets)
    {
        if(desc_set_info->descriptor == descriptor)
            return desc_set_info;
    }
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Searches for field metadata info by descriptor ID.
///
///@param descriptorSets List of descriptor sets to search.
///@param descriptor     Descriptor set and field descriptor to search for.
///
///@returns A pointer to the field metadata if found.
///@returns NULL if the specified descriptor isn't in the list.
///
const FieldInfo* findField(const DescriptorSetSpan& descriptorSets, mip::CompositeDescriptor descriptor)
{
    if(mip::isDataDescriptorSet(descriptor.descriptorSet))
    {
        // Special case for shared data
        if(mip::isSharedDataFieldDescriptor(descriptor.fieldDescriptor))
            return findField(DATA_SHARED, descriptor.fieldDescriptor, false);
    }
    else if(isCommandDescriptorSet(descriptor.descriptorSet))
    {
        // Special case for reply fields
        if(isReplyFieldDescriptor(descriptor.fieldDescriptor))
            return &MetadataFor<ReplyField>::value;
    }

    const DescriptorSetInfo* ds_info = findDescriptorSet(descriptorSets, descriptor.descriptorSet);
    if(!ds_info)
        return nullptr;

    if(const FieldInfo* info = findField(*ds_info, descriptor.fieldDescriptor, false))
        return info;

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Searches for field metadata info by descriptor ID.
///
/// You can use this function with a specific descriptor set constant, or after
/// searching for one with findDescriptorSet.
///
///@param descriptorSets    Specific descriptor set to search.
///@param descriptor        Field descriptor to search for.
///@param check_shared_data If true and the descriptor is a shared data field,
///                         searches the shared data set instead.
///
///@returns A pointer to the field metadata if found.
///@returns NULL if the specified descriptor isn't in the list.
///
const FieldInfo* findField(const DescriptorSetInfo& ds_info, uint8_t field_desc, bool check_shared_data)
{
    if(check_shared_data && mip::isDataDescriptorSet(ds_info.descriptor))
    {
        if(mip::isSharedDataFieldDescriptor(field_desc))
            return findField(DATA_SHARED, field_desc, false);
    }

    // Binary search assumes fields are sorted by descriptor.
    auto it = std::lower_bound(
        ds_info.fields.begin(), ds_info.fields.end(), field_desc,
        [](const FieldInfo* a, uint8_t b){ return a->descriptor.fieldDescriptor < b; }
    );

    return (it != ds_info.fields.end()) && ((*it)->descriptor.fieldDescriptor==field_desc) ? *it : nullptr;
}


} // namespace mip::metadata
