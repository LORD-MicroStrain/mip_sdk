#pragma once

#include <mip/metadata/mip_structures.hpp>
#include <microstrain/span.hpp>


namespace mip::metadata
{
    extern microstrain::Span<const DescriptorSetInfo* const> allDescriptorSets();
}
