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


//class Analyzer
//{
//public:
//    Analyzer(microstrain::Serializer serializer) : m_serializer(serializer) {}
//
//    bool analyze(const StructInfo& info)
//    {
//        size_t nextOffsetEntry = 0;
//
//        for(const ParameterInfo& param : info.parameters)
//        {
//            if(param.count.paramIdx.isValid(info.parameters.size()))
//            {
//
//            }
//        }
//    }
//private:
//    size_t getOffsetForParameter(const ParameterInfo* parameters, size_t index) const
//    {
//        for(size_t i=0; i<index; i++)
//        {
//            const ParameterInfo& param = parameters[i];
//            size_t size = sizeForBasicType(param.type);
//            if(size == 0)
//            {
//
//            }
//        }
//        for(const auto& param : parameters)
//    }
//
//private:
//    struct Offset
//    {
//        microstrain::Index parameterId = {};
//        size_t             offset      = 0;
//    };
//
//private:
//    // Maximum number of parameters in a struct that can reference other parameters,
//    // i.e. counters for arrays or union discriminators.
//    static constexpr inline size_t MAX_PARAM_REFS = 5;
//
//    microstrain::Serializer m_sserializer;
//    std::array<Offset, MAX_PARAM_REFS> m_offsets;
//};
//
//
//
//constexpr size_t getParameterOffset(microstrain::Serializer serializer, microstrain::Index parameterId)
//{
//    const auto& parameters = MetadataFor<FieldType>::parameters;
//    if(parameterId.isValid(parameters.size()))
//        return -1;
//
//    size_t offset = 0;
//
//    for(auto i=microstrain::Index(0); i<parameterId; i++)
//    {
//        const ParameterInfo& parameter = parameters[i.index()];
//
//        size_t size = sizeForBasicType(parameter.type);
//        if(size == 0)
//        {
//            if(parameter.type.type == Type::STRUCT)
//            {
//                // TODO
//            }
//        }
//
//        if(parameter.count.isFixed())
//            offset += size;
//        else
//        {
//            assert(parameter.count.paramIdx.isValid(parameters.size()));
//
//            uint8_t count = readCount(serializer, parameters, parameter.count.paramIdx);
//
//        }
//    }
//
//    return offset;
//}

} // namespace mip
