#pragma once

#include "mip_metadata.hpp"
#include "mip_structures.hpp"
#include "mip_formatter.hpp"

#include <mip/mip_descriptors.hpp>


namespace mip::metadata
{

class StructFormatter
{
public:
    StructFormatter(Formatter& formatter) : mFormatter(formatter) {}

    // template<class MipType>
    // void format(const MipType& field)
    // {
    //     const auto* info = MetadataFor<MipType>::value;
    //
    //     format((void*)&field, info);
    // }

    template<class MipType>
    void format(const std::enable_if_t<std::is_class_v<MipType>, MipType>& field)
    {
        constexpr auto& info = &MetadataFor<MipType>::value;

        auto formatAll = [this](auto... args)
        {
            size_t i=0;
            ( (formatParam(args, info.parameters[i]), i++) + ... );
        };

        if constexpr(isField<MipType>::value)
            mFormatter.formatFieldBegin(&info);
        else
            mFormatter.formatStructBegin(&info);

        auto tuple = field.asTuple();
        std::apply(formatAll, tuple);

        mFormatter.formatEnd(utils::ParamType<MipType>::value);
    }

    template<class T>
    void formatParam(const ParameterInfo& info, const T& value)
    {

    }

    template<class EnumType>
    void format(std::enable_if<std::is_enum_v<EnumType>, EnumType> value)
    {
        auto& info = MetadataFor<EnumType>::value;
        mFormatter.formatEnum(&info, value);
    }

    template<class BitfieldType>
    void format(const mip::Bitfield<BitfieldType>& bf)
    {
        auto& info = MetadataFor<BitfieldType>::value;
        mFormatter.formatBitfield(&info, static_cast<const BitfieldType&>(bf).value);
    }

    template<class T>
    void format(std::enable_if_t<std::is_arithmetic_v<T>, T> value)
    {
        mFormatter.formatValue(utils::ParamType<T>::value, value);
    }

private:
    void format(const FieldInfo& info, const void* ptr);
    void formatStructContents(const StructInfo& info, const void* ptr);
    void formatParameter(const ParameterInfo& info, const void* ptr);
    void format(const StructInfo& info, const void* ptr);

private:
    Formatter& mFormatter;
};



} // namespace mip::metadata
