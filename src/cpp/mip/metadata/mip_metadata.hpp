#pragma once

#include "structures.hpp"
#include "utils.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_descriptors.hpp>


namespace mip::metadata
{

// Type trait class to be specialized for each field type.
template<class FieldType>
struct MetadataForField;



// template<ParameterInfo::Type ParamType> struct ParamEnum;
// template<> struct ParamEnum<ParameterInfo::Type::U8> :


///@brief Gets a reference to parameter 'I' in the struct for the given field.
///
///@tparam I     Index indicating which parameter to access. 0-based.
///@tparam FieldType Type of the MIP field struct.
///
///@param field An instance of the field struct whose member variable will be accessed.
///
///@returns A reference to member I of the given field.
///
template<size_t I, class FieldType>
auto& get(typename EnableForFieldTypes<FieldType>::type& field)
{
    constexpr ParameterInfo& paramInfo = MetadataForField<FieldType>::PARAMETERS[I].type;
    using T = typename utils::ParamEnum<paramInfo.type.type>::type;
    return *static_cast<T*>(paramInfo.accessor(&field));
}


namespace utils
{
}



template<class FieldType, size_t ParamIndex, class ParamType>
ParamType get(FieldType& field)
{
}

template<size_t ParameterIndex, class FieldType>
auto get(FieldType& field)
{
    const ParameterInfo& paramInfo = MetadataForField<FieldType>::PARAMETERS[ParameterIndex];


}


} // namespace mip
