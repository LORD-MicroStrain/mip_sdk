
#include "mip_definitions.hpp"

#include <mip/definitions/data_shared.hpp>
#include <mip/definitions/data_gnss.hpp>
#include <mip/metadata/definitions/common.hpp>

#include <algorithm>

namespace mip::metadata
{

std::span<const FieldInfo* const>::iterator DescriptorSet::findIter(uint8_t fieldDescriptor) const
{
    return std::lower_bound(
        mFields.begin(), mFields.end(),
        fieldDescriptor,
        [](const FieldInfo* a, uint8_t fieldDescriptor) -> bool {
            return a->descriptor.fieldDescriptor < fieldDescriptor;
        }
    );
}

const FieldInfo* DescriptorSet::findField(uint8_t fieldDescriptor) const
{
    auto it = findIter(fieldDescriptor);

    return (it != mFields.end()) ? *it : nullptr;
}

std::span<const FieldInfo* const> DescriptorSet::allCommands() const
{
    auto it = findIter(mip::C::MIP_RESPONSE_DESCRIPTOR_START);

    return {mFields.begin(), it};
}

std::span<const FieldInfo* const> DescriptorSet::allResponses() const
{
    auto it = findIter(mip::C::MIP_RESPONSE_DESCRIPTOR_START);

    return {it, mFields.end()};
}

// void Definitions::DescriptorSet::add(const FieldInfo* field)
// {
//     // Maintain the order of fields by descriptor.
//     auto it = find(field->descriptor.fieldDescriptor);
//
//     mFields.insert(it, field);
// }
//
// void Definitions::DescriptorSet::add(std::span<const FieldInfo*> fields)
// {
//     for(auto field : fields)
//         add(field);
// }
//
// auto Definitions::DescriptorSet::find(uint8_t descriptor) const -> std::vector<const FieldInfo*>::const_iterator
// {
//     return std::lower_bound(
//         mFields.begin(), mFields.end(),
//         descriptor,
//         [](const FieldInfo* a, uint8_t fieldDescriptor) -> bool {
//             return a->descriptor.fieldDescriptor < fieldDescriptor;
//         }
//     );
// }
//
// const FieldInfo* Definitions::DescriptorSet::findField(uint8_t descriptor) const
// {
//     auto it = find(descriptor);
//
//     return (it != mFields.end()) ? *it : nullptr;
// }
//
// void Definitions::registerDescriptorSet(uint8_t descriptorSet, const char* name, std::span<const FieldInfo*> fields)
// {
//     // Insert in order if not already present.
//     auto it = find(descriptorSet);
//
//     if(it == mDescriptorSets.end())
//         it = mDescriptorSets.emplace(it, descriptorSet, name);
//
//     // const_cast because findDescriptorSetIter gives vector::const_iterator.
//     const_cast<DescriptorSet&>(*it).add(fields);
// }

// auto Definitions::find(uint8_t descriptorSet) const -> DescSetList::const_iterator
// {
//     return std::lower_bound(
//         mDescriptorSets.begin(), mDescriptorSets.end(),
//         descriptorSet,
//         [](const DescriptorSet& set, uint8_t desc) -> bool {
//             return set.value() < desc;
//         }
//     );
// }
//
// auto Definitions::findDescriptorSet(uint8_t descriptorSet) const -> const DescriptorSet*
// {
//     auto it = find(descriptorSet);
//
//     return (it != mDescriptorSets.end()) ? &*it : nullptr;
// }

const DescriptorSet* Definitions::findDescriptorSet(uint8_t descriptorSet) const
{
    auto it = std::lower_bound(
        mDescriptorSets.begin(), mDescriptorSets.end(),
        descriptorSet,
        [](const DescriptorSet* set, uint8_t desc) -> bool {
            return set->value() < desc;
        }
    );

    return (it != mDescriptorSets.end()) ? *it : nullptr;
}

const FieldInfo* Definitions::findField(mip::CompositeDescriptor descriptor) const
{
    const DescriptorSet* ds = nullptr;

    // if(isDataDescriptorSet(descriptor.descriptorSet) && isSharedDataFieldDescriptor(descriptor.fieldDescriptor))
    //     ds = findDescriptorSet(data_shared::DESCRIPTOR_SET);
    // else if(isDataDescriptorSet(descriptor.descriptorSet) && isGnssDataDescriptorSet(descriptor.fieldDescriptor))
    //     ds = findDescriptorSet(data_gnss::DESCRIPTOR_SET);
    // else
        ds = findDescriptorSet(descriptor.descriptorSet);;

    return ds ? ds->findField(descriptor.fieldDescriptor) : nullptr;
}

// const FieldInfo* Definitions::findField(mip::CompositeDescriptor descriptor) const
// {
//     //auto it = findFieldIter(descriptor);
//     auto it = mFields.find(descriptor);
//
//     if(it != mFields.end())
//         return *it;
//
//     // Check for shared data fields.
//     if(isDataDescriptorSet(descriptor.descriptorSet) && isSharedDataFieldDescriptor(descriptor.fieldDescriptor))
//     {
//         it = mFields.find(CompositeDescriptor{data_shared::DESCRIPTOR_SET, descriptor.fieldDescriptor});
//
//         if(it != mFields.end())
//             return *it;
//     }
//     // Check for GNSS data (shared among 6 descriptor sets)
//     else if(isGnssDataDescriptorSet(descriptor.descriptorSet))
//     {
//         it = mFields.find(CompositeDescriptor{data_gnss::DESCRIPTOR_SET, descriptor.fieldDescriptor});
//
//         if(it != mFields.end())
//             return *it;
//     }
//     // Reply descriptor?
//     else if(isCommandDescriptorSet(descriptor.descriptorSet) && isReplyFieldDescriptor(descriptor.fieldDescriptor))
//         return &MetadataFor<ReplyField>::value;
//
//     return nullptr;
// }

} // namespace mip::metadata
