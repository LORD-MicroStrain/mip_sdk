#pragma once

#include "mip_structures.hpp"
#include "mip_meta_utils.hpp"

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
        { (uint8_t)FunctionSelector::WRITE, "WRITE",   "Applies a new setting." },
        { (uint8_t)FunctionSelector::READ,  "READ",    "Reads the current setting." },
        { (uint8_t)FunctionSelector::SAVE,  "SAVE",    "Saves the current setting as power-on setting." },
        { (uint8_t)FunctionSelector::LOAD,  "LOAD",    "Loads the power-on setting." },
        { (uint8_t)FunctionSelector::RESET, "DEFAULT", "Resets to the factory default setting." },
    };

    static constexpr inline EnumInfo value = {
        /*.name    =*/ "FunctionSelector",
        /*.docs    =*/ "",
        /*.type    =*/ Type::U8,
        /*.entries =*/ entries,
    };
};

inline void* accessFunctionSelector(void* p) { return static_cast<FunctionSelector*>(p); }

static constexpr inline ParameterInfo FUNCTION_SELECTOR_PARAM = {
    /* .name      = */ "function",
    /* .docs      = */ "Standard MIP function selector",
    /* .type      = */ {Type::ENUM, &MetadataFor<FunctionSelector>::value},
    /* .accessor  = */ accessFunctionSelector,
    /* .functions = */ {true,true,true,true,true},
    /* .count     = */ 1,
    /* .condition = */ {},
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


//template<class FieldType, size_t ParamIndex, class ParamType>
//ParamType get(FieldType& field)
//{
//}
//
//template<size_t ParameterIndex, class FieldType>
//auto get(FieldType& field)
//{
//    const ParameterInfo& paramInfo = MetadataFor<FieldType>::PARAMETERS[ParameterIndex];
//
//
//}

class DescriptorSet
{
public:
    using FieldPtr = const FieldInfo*;

    template<size_t N>
    constexpr DescriptorSet(uint8_t descriptor, const char* name, const FieldPtr (&fields)[N]) :
        mDescriptor(descriptor,0x00), mName(name), mFields(fields)
    {
    }

    constexpr DescriptorSet(uint8_t descriptor, const char* name, std::span<const FieldPtr> fields) :
        mDescriptor(descriptor, 0x00), mName(name), mFields(fields)
    {
    }

    constexpr const char*         name()           const { return mName; }
    constexpr uint8_t             value()          const { return mDescriptor.descriptorSet; }
    constexpr CompositeDescriptor fullDescriptor() const { return mDescriptor; }

    constexpr auto& allFields() const { return mFields; }

    auto allCommands() const -> std::span<const FieldPtr>;
    auto allResponses() const -> std::span<const FieldPtr>;

    const FieldInfo* findField(uint8_t fieldDescriptor) const;

private:
    std::span<const FieldPtr>::iterator findIter(uint8_t fieldDescriptor) const;

private:
    CompositeDescriptor       mDescriptor = 0x0000;
    const char*               mName = nullptr;
    std::span<const FieldPtr> mFields;
};


} // namespace mip::metadata
