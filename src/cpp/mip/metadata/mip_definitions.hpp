#pragma once

#include <microstrain/common/span.hpp>
#include <mip/mip_descriptors.hpp>

//#include <set>
//#include <initializer_list>


namespace mip::metadata
{

struct FieldInfo;
struct DescriptorSetInfo;

using FieldSpan = microstrain::Span<const FieldInfo* const>;
using DescriptorSetSpan = microstrain::Span<const DescriptorSetInfo* const>;

const DescriptorSetInfo* findDescriptorSet(const DescriptorSetSpan& descriptorSets, uint8_t descriptor);
const FieldInfo* findField(const DescriptorSetSpan& descriptorSets, mip::CompositeDescriptor descriptor);
const FieldInfo* findField(const DescriptorSetInfo& ds_info, uint8_t field_desc);


//class Definitions
//{
//    //struct Less
//    //{
//    //    inline bool operator()(const FieldInfo *a, const FieldInfo *b) const
//    //    {
//    //        return a->descriptor < b->descriptor;
//    //    }
//    //    inline bool operator()(const FieldInfo *a, CompositeDescriptor desc) const
//    //    {
//    //        return a->descriptor < desc;
//    //    }
//    //    inline bool operator()(CompositeDescriptor desc, const FieldInfo *a) const
//    //    {
//    //        return desc < a->descriptor;
//    //    }
//    //    using is_transparent = void;
//    //};
//    //
//    //using Container = std::set<const FieldInfo *, Less>;
//
//public:
//    Definitions() = default;
//    Definitions(const DescriptorSetSpan& fields) { registerDefinitions(fields); }
//
//    //void registerField(const FieldInfo* field);
//    //void registerDefinitions(const FieldSpan& fields);
//    //void registerDefinitions(const DescriptorSetSpan& fields);
//
//    const DescriptorSetInfo* findDescriptorSet(uint8_t desc_set) const;
//    const FieldInfo* findField(mip::CompositeDescriptor descriptor) const;
//
//private:
//    //Container mFields;
//    DescriptorSetSpan mDescriptorSets;
//};




} // namespace mip::metadata
