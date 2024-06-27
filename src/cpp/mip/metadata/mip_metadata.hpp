#pragma once

#include "structures.hpp"
#include "utils.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_descriptors.hpp>


namespace mip::metadata
{

// Type trait class to be specialized for each field/struct/etc.
template<class FieldType>
struct MetadataFor;


template<>
struct MetadataFor<FunctionSelector>
{
    using type = FunctionSelector;

    static constexpr EnumInfo::Entry entries[] = {
        { "WRITE",   (uint8_t)FunctionSelector::WRITE },
        { "READ",    (uint8_t)FunctionSelector::READ  },
        { "SAVE",    (uint8_t)FunctionSelector::SAVE  },
        { "LOAD",    (uint8_t)FunctionSelector::LOAD  },
        { "DEFAULT", (uint8_t)FunctionSelector::RESET },
    };

    static constexpr inline EnumInfo value = {
        .name = "FunctionSelector",
        .docs = "",
        .type = Type::U8,
        .entries = entries,
    };
};

inline void* accessFunctionSelector(void* p) { return static_cast<FunctionSelector*>(p); }

static constexpr inline ParameterInfo FUNCTION_SELECTOR_PARAM = {
    /* .name          = */ "function",
    /* .docs          = */ "Standard MIP function selector",
    /* .type          = */ {Type::ENUM, &MetadataFor<FunctionSelector>::value},
    /* .accessor      = */ accessFunctionSelector,
    /* .byte_offset   = */ 0,
    /* .functions     = */ {true,true,true,true,true},
    /* .count         = default */
    /* .counter_idx   = default */
    /* .union_index   = default */
    /* .union_value   = default */
};



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
    constexpr ParameterInfo& paramInfo = MetadataFor<FieldType>::PARAMETERS[I].type;
    using T = typename utils::ParamEnum<paramInfo.type.type>::type;
    return *static_cast<T*>(paramInfo.accessor(&field));
}


template<class FieldType, size_t ParamIndex, class ParamType>
ParamType get(FieldType& field)
{
}

template<size_t ParameterIndex, class FieldType>
auto get(FieldType& field)
{
    const ParameterInfo& paramInfo = MetadataFor<FieldType>::PARAMETERS[ParameterIndex];


}


} // namespace mip
