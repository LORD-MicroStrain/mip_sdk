#pragma once

#include "mip_metadata.hpp"
#include "../mip_descriptors.hpp"

#include <set>
#include <initializer_list>


namespace mip::metadata
{

class Definitions
{
public:
    void registerField(const FieldInfo* field);
    void registerDefinitions(std::initializer_list<const FieldInfo*> fields);

    const FieldInfo* findField(mip::CompositeDescriptor descriptor) const;

    //std::span<const FieldInfo*> findDescriptorSet(uint8_t descriptorSet) const;

private:
    //std::vector<const FieldInfo*>::const_iterator findFieldIter(mip::CompositeDescriptor desc) const;

    struct Less
    {
        inline bool operator()(const FieldInfo *a, const FieldInfo *b) const
        {
            return a->descriptor < b->descriptor;
        }
        inline bool operator()(const FieldInfo *a, CompositeDescriptor desc) const
        {
            return a->descriptor < desc;
        }
        inline bool operator()(CompositeDescriptor desc, const FieldInfo *a) const
        {
            return desc < a->descriptor;
        }
        using is_transparent = void;
    };

private:
    std::set<const FieldInfo *, Less> mFields;
};




} // namespace mip::metadata
