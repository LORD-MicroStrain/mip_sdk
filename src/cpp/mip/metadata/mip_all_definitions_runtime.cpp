
#include "mip_all_definitions_runtime.hpp"
#include "mip_all_definitions.hpp"


namespace mip::metadata
{

microstrain::Span<const DescriptorSetInfo* const> allDescriptorSets()
{
    return ALL_DESCRIPTOR_SETS;
}

} // namespace mip::metadata
