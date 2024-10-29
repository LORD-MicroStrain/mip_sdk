#pragma once

#include "mip_metadata.hpp"

#include <mip/mip_descriptors.hpp>

#include <span>


namespace mip::metadata
{


class Definitions
{
public:
    // class DescriptorSet
    // {
    // public:
    //     DescriptorSet(uint8_t descriptor, const char* name) : mDescriptor(descriptor, mip::INVALID_FIELD_DESCRIPTOR), mName(name) {}
    //
    //     // Not copyable
    //     DescriptorSet(const DescriptorSet&) = delete;
    //     DescriptorSet& operator=(const DescriptorSet&) = delete;
    //     // Movable
    //     DescriptorSet(DescriptorSet&& other) : mDescriptor(other.mDescriptor), mName(other.mName), mFields(std::move(other.mFields)) {}
    //     DescriptorSet& operator=(DescriptorSet&& other) = default;// { mDescriptor=other.mDescriptor; mName=other.mName; mFields=std::move(other.mFields); }
    //
    //     const char*         name()           const { return mName; }
    //     uint8_t             value()          const { return mDescriptor.descriptorSet; }
    //     CompositeDescriptor fullDescriptor() const { return mDescriptor; }
    //
    //     void add(const FieldInfo* field);
    //     void add(std::span<const FieldInfo*> fields);
    //
    //     const FieldInfo* findField(uint8_t descriptor) const;
    //
    // private:
    //     std::vector<const FieldInfo*>::const_iterator find(uint8_t descriptor) const;
    //
    // private:
    //     CompositeDescriptor           mDescriptor = 0x0000;
    //     const char*                   mName = nullptr;
    //     std::vector<const FieldInfo*> mFields;
    // };

    // void registerDescriptorSet(uint8_t desc_set, const char* name, std::span<const FieldInfo*> fields);

    using DsPtr = const DescriptorSet*;

    template<size_t N>
    constexpr Definitions(const DsPtr (&descriptorSets)[N]) : mDescriptorSets(descriptorSets) {}

    const DescriptorSet* findDescriptorSet(uint8_t descriptorSet) const;
    const FieldInfo* findField(mip::CompositeDescriptor descriptor) const;

private:
    std::span<const DsPtr> mDescriptorSets;

// private:
//     using DescSetList = std::vector<DescriptorSet>;
//
//     DescSetList::const_iterator find(uint8_t descriptorSet) const;
//
// private:
//     // DescSetList mDescriptorSets;
};




} // namespace mip::metadata
