#pragma once

namespace mscl
{
    namespace detail
    {
        ////////////////////////////////////////////////////////////////////////////////
        ///@brief Internal wrapper for converting C enums to their C++ versions.
        ///
        template<class BaseEnum>
        struct EnumWrapper
        {
            using CType = BaseEnum;

            BaseEnum _value;

            EnumWrapper() = default;
            EnumWrapper(const EnumWrapper<BaseEnum>&) = default;
            EnumWrapper(BaseEnum val) : _value(val) {}

            EnumWrapper& operator=(const EnumWrapper<BaseEnum>&) = default;
            EnumWrapper& operator=(BaseEnum val) { _value=val; return *this; }

            operator BaseEnum() const { return _value; }
            operator BaseEnum&() { return _value; }
        };
    }
}
