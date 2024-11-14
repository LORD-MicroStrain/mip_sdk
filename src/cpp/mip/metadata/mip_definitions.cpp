
#include "mip_definitions.hpp"

#include <mip/definitions/data_shared.hpp>
#include <mip/metadata/definitions/common.hpp>

namespace mip::metadata
{

void Definitions::registerField(const mip::metadata::FieldInfo* field)
{
    mFields.insert(field);
}

void Definitions::registerDefinitions(const FieldInfoSpan &fields)
{
    for (const FieldInfo *field : fields)
        mFields.insert(field);
}

void Definitions::registerDefinitions(const FieldInfoSpans &fields)
{
    for (const auto* sublist : fields)
        registerDefinitions(*sublist);
}

//std::vector<const FieldInfo *>::const_iterator Definitions::findFieldIter(mip::CompositeDescriptor descriptor) const
//{
//    return std::lower_bound(
//        mFields.begin(), mFields.end(), descriptor,
//        [](const FieldInfo* info, CompositeDescriptor desc)->bool
//        {
//            return info->descriptor < desc;
//        }
//    );
//}

const FieldInfo* Definitions::findField(mip::CompositeDescriptor descriptor) const
{
    //auto it = findFieldIter(descriptor);
    auto it = mFields.find(descriptor);

    if(it != mFields.end())
        return *it;

    // Check for shared data fields.
    if(isDataDescriptorSet(descriptor.descriptorSet) && isSharedDataFieldDescriptor(descriptor.fieldDescriptor))
    {
        it = mFields.find(CompositeDescriptor{data_shared::DESCRIPTOR_SET, descriptor.fieldDescriptor});

        if(it != mFields.end())
            return *it;
    }
    // Reply descriptor?
    else if(isCommandDescriptorSet(descriptor.descriptorSet) && isReplyFieldDescriptor(descriptor.fieldDescriptor))
        return &MetadataFor<ReplyField>::value;

    return nullptr;
}

} // namespace mip::metadata
