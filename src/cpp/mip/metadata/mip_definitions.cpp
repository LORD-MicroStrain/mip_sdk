
#include "mip_definitions.hpp"

namespace mip::metadata
{

void Definitions::registerField(const mip::metadata::FieldInfo* field)
{
    mFields.insert(field);
}

void Definitions::registerDefinitions(std::initializer_list<const FieldInfo *> fields)
{
    mFields.insert(fields);
}

void Definitions::registerDefinitions(
    const std::initializer_list< const std::initializer_list<const FieldInfo* >* >& fields)
{
    for(const auto* sublist : fields)
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

    if(it == mFields.end())
        return nullptr;

    return *it;
}

} // namespace mip::metadata
