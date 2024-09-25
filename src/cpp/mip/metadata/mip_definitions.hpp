#pragma once

#include "mip_metadata.hpp"

#include <mip/mip_descriptors.hpp>

#include <set>
#include <initializer_list>


namespace mip::metadata
{

class Definitions
{
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

    using Container = std::set<const FieldInfo *, Less>;

public:
    Definitions() = default;
    Definitions(const std::initializer_list<const std::initializer_list<const FieldInfo*>*>& fields) { registerDefinitions(fields); }

    void registerField(const FieldInfo* field);
    void registerDefinitions(std::initializer_list<const FieldInfo*> fields);
    void registerDefinitions(const std::initializer_list<const std::initializer_list<const FieldInfo*>*>& fields);

    const FieldInfo* findField(mip::CompositeDescriptor descriptor) const;


private:
    Container mFields;
};




} // namespace mip::metadata
