
#include "mip_all_definitions_runtime.hpp"
#include "mip_all_definitions.hpp"

#include <mip/mip_metadata.h>


namespace mip::metadata
{

microstrain::Span<const DescriptorSetInfo* const> allDescriptorSets()
{
    return ALL_DESCRIPTOR_SETS;
}

} // namespace mip::metadata


namespace mip::C
{

const struct mip_meta_descriptor_set_info* convert_to_c(const metadata::DescriptorSetInfo* info) { return reinterpret_cast<const struct mip_meta_descriptor_set_info*>(info); }
const struct mip_meta_field_info*          convert_to_c(const metadata::FieldInfo*         info) { return reinterpret_cast<const struct mip_meta_field_info*         >(info); }
const struct mip_meta_struct_info*         convert_to_c(const metadata::StructInfo*        info) { return reinterpret_cast<const struct mip_meta_struct_info*        >(info); }
const struct mip_meta_param_info*          convert_to_c(const metadata::ParameterInfo*     info) { return reinterpret_cast<const struct mip_meta_param_info*         >(info); }

const metadata::DescriptorSetInfo* convert_to_cpp(const struct mip_meta_descriptor_set_info* info) { return reinterpret_cast<const metadata::DescriptorSetInfo*>(info); }
const metadata::FieldInfo*         convert_to_cpp(const struct mip_meta_field_info*          info) { return reinterpret_cast<const metadata::FieldInfo*        >(info); }
const metadata::StructInfo*        convert_to_cpp(const struct mip_meta_struct_info*         info) { return reinterpret_cast<const metadata::StructInfo*       >(info); }
const metadata::ParameterInfo*     convert_to_cpp(const struct mip_meta_param_info*          info) { return reinterpret_cast<const metadata::ParameterInfo*    >(info); }

//
// Top-level
//

int mip_meta_iterate_descriptor_sets(void* user, mip_meta_descriptor_set_callback callback)
{
    for(const metadata::DescriptorSetInfo* ds : metadata::ALL_DESCRIPTOR_SETS)
    {
        int result = callback(user, convert_to_c(ds));

        if(result != 0)
            return result;
    }

    return 0;
}

const struct mip_meta_descriptor_set_info* mip_meta_find_descriptor_set(uint8_t descriptor)
{
    const metadata::DescriptorSetInfo* info = metadata::findDescriptorSet(metadata::ALL_DESCRIPTOR_SETS, descriptor);
    return convert_to_c(info);
}

//
// Descriptor sets
//

uint8_t mip_meta_descset_value(const struct mip_meta_descriptor_set_info* info)
{
    assert(info);

    return convert_to_cpp(info)->descriptor;
}

const char* mip_meta_descset_nameE(const struct mip_meta_descriptor_set_info* info)
{
    assert(info);

    return convert_to_cpp(info)->name;
}

int mip_meta_iterate_fields(void* user, const struct mip_meta_descriptor_set_info* ds_info, mip_meta_field_callback callback)
{
    assert(ds_info);

    for(const metadata::FieldInfo* info : convert_to_cpp(ds_info)->fields)
    {
        int result = callback(user, convert_to_c(info));

        if(result != 0)
            return result;
    }

    return 0;
}

const struct mip_meta_field_info* mip_meta_find_field_in_descriptor_set(const struct mip_meta_descriptor_set_info* ds_info, uint8_t field_descriptor, bool check_shared)
{
    assert(ds_info);

    return convert_to_c( metadata::findField(*convert_to_cpp(ds_info), field_descriptor, check_shared) );
}

//
// Fields
//

uint16_t mip_meta_field_descriptor(const struct mip_meta_field_info* info)
{
    assert(info);

    return convert_to_cpp(info)->descriptor.as_u16();
}

const struct mip_meta_field_info* mip_meta_field_response(const struct mip_meta_field_info* info)
{
    assert(info);

    return convert_to_c( convert_to_cpp(info)->response );
}

uint8_t mip_meta_field_supported_function_selectors(const struct mip_meta_field_info* info)
{
    assert(info);

    return convert_to_cpp(info)->functions.bits;
}

const char* mip_meta_struct_name(const struct mip_meta_struct_info* info)
{
    assert(info);

    return convert_to_cpp(info)->name;
}

const char* mip_meta_struct_title(const struct mip_meta_struct_info* info)
{
    assert(info);

    return convert_to_cpp(info)->title;
}

const char* mip_meta_struct_docs(const struct mip_meta_struct_info* info)
{
    assert(info);

    return convert_to_cpp(info)->docs;
}

int mip_meta_iterate_struct_parameters(void* user, const struct mip_meta_struct_info* info, mip_meta_param_callback callback)
{
    assert(info);

    for(const metadata::ParameterInfo& param : convert_to_cpp(info)->parameters)
    {
        int result = callback(user, convert_to_c(&param));

        if(result != 0)
            return result;
    }

    return 0;
}

//
// Parameters
//

const char*  mip_meta_param_name(const struct mip_meta_param_info* info)
{
    assert(info);

    return convert_to_cpp(info)->name;
}

const char* mip_meta_param_docs(const struct mip_meta_param_info* info)
{
    assert(info);

    return convert_to_cpp(info)->docs;
}

unsigned int mip_meta_param_type(const struct mip_meta_param_info* info)
{
    assert(info);

    return static_cast<unsigned int>(convert_to_cpp(info)->type.type);
}

uint8_t mip_meta_param_required_functions(const struct mip_meta_param_info* info)
{
    assert(info);

    return convert_to_cpp(info)->attributes.bits;
}

int mip_meta_param_count(const struct mip_meta_param_info* info)
{
    assert(info);

    const metadata::ParameterInfo::Count& count = convert_to_cpp(info)->count;

    return (count.hasCounter()) ? -(int)count.paramIdx.id() : (int)count.count;
}


} // namespace mip::C
