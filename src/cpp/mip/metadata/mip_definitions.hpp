#pragma once

#include "mip_metadata.hpp"
#include "../mip_descriptors.hpp"

#include <vector>
#include <span>


namespace mip::metadata
{

class Definitions
{
public:
    const metadata::FieldInfo* findField(mip::CompositeDescriptor descriptor);

    std::span<const metadata::FieldInfo*> findDescriptorSet(uint8_t descriptorSet);

private:
    //std::vector<std::span<const metadata::FieldInfo*>> mFields;

};




} // namespace mip::metadata
