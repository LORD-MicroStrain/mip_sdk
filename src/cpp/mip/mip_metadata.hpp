#pragma once

#include <microstrain/common/platform.hpp>

#include "mip_serialization.hpp"
#include "mip_descriptors.hpp"

namespace mip
{



struct FuncBits
{
    constexpr FuncBits() = default;
    constexpr FuncBits(bool w, bool r, bool s, bool l, bool d, bool e=false) : bits(0x00) { write(w); read(r); save(s); load(l); reset(d); echo(e); }

    constexpr bool any() const { return bits > 0; }

    constexpr bool write() const { return bits & 0b00000001; }
    constexpr bool read()  const { return bits & 0b00000010; }
    constexpr bool save()  const { return bits & 0b00000100; }
    constexpr bool load()  const { return bits & 0b00001000; }
    constexpr bool reset() const { return bits & 0b00010000; }
    constexpr bool echo()  const { return bits & 0b10000000; }

    constexpr FuncBits& write(bool w) { bits = (bits & 0b11111110) | (uint8_t(w)<<0); return *this; }
    constexpr FuncBits& read (bool r) { bits = (bits & 0b11111101) | (uint8_t(r)<<1); return *this; }
    constexpr FuncBits& save (bool s) { bits = (bits & 0b11111011) | (uint8_t(s)<<2); return *this; }
    constexpr FuncBits& load (bool l) { bits = (bits & 0b11110111) | (uint8_t(l)<<3); return *this; }
    constexpr FuncBits& reset(bool d) { bits = (bits & 0b11101111) | (uint8_t(d)<<4); return *this; }
    constexpr FuncBits& echo (bool e) { bits = (bits & 0b01111111) | (uint8_t(e)<<7); return *this; }

    constexpr bool has(mip::FunctionSelector function) const { return (bits >> (static_cast<uint8_t>(function) - static_cast<uint8_t>(mip::FunctionSelector::WRITE))) & 1; }

    uint8_t bits = 0x00;
};


struct StructInfo;

// Type trait class to be specialized for each field type.
template<class FieldType>
struct FieldInfo;


struct ParameterInfo
{
    enum class Type : uint8_t
    {
        NONE = 0,  ///< Invalid/unknown.

        BOOL,
        U8,
        S8,
        U16,
        S16,
        U32,
        S32,
        U64,
        S64,
        FLOAT,
        DOUBLE,

        ENUM,
        BITFIELD,
        STRUCT,
        UNION,
    };

    // struct TypeInfo
    // {
    //     template<class Field, class T>
    //     TypeInfo(T Field::*member);
    //
    //     Type type;
    //     uint8_t struct_offset = 0;
    //     StructInfo*
    // };

    using Accessor = void* (*)(void*);

    const char* name = nullptr;     ///< Programmatic name (e.g. for printing or language bindings).
    const char* docs = nullptr;     ///< Human-readable documentation.
    Type        type = Type::NONE;  ///< Data type.
    Accessor    accessor = nullptr; ///< Obtains a reference to the member variable.
    uint8_t     byte_offset   = 0;  ///< Offset "on the wire" or in a serialized buffer, in bytes.
    FuncBits    functions;          ///< This parameter is required for the specified function selectors.
    uint8_t     count = 1;          ///< Number of elements. 0 if size is specified at runtime.
    int8_t      counter_idx = 0;    ///< When count == 0, this specifies the parameter holding the runtime count, relative to this parameter's index.
    int8_t      union_index = 0;    ///< When type is UNION, this specifies the parameter that controls which union member is active (the discriminator).
    uint16_t    union_value = 0;    ///< When in a union, this specifies the value of the discriminator parameter that selects this member.
};

template<class T, class=void> struct ParamType { static INLINE_VAR constexpr auto value = ParameterInfo::Type::NONE; };

template<> struct ParamType<bool,     void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::BOOL;   };
template<> struct ParamType<uint8_t,  void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::U8;     };
template<> struct ParamType< int8_t,  void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::S8;     };
template<> struct ParamType<uint16_t, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::U16;    };
template<> struct ParamType< int16_t, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::S16;    };
template<> struct ParamType<uint32_t, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::U32;    };
template<> struct ParamType< int32_t, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::S32;    };
template<> struct ParamType<uint64_t, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::U64;    };
template<> struct ParamType< int64_t, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::S64;    };
template<> struct ParamType<float,    void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::FLOAT;  };
template<> struct ParamType<double,   void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::DOUBLE; };
template<class T> struct ParamType<Bitfield<T>, void> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::BITFIELD; };
template<class T> struct ParamType<T, typename std::enable_if<std::is_enum<T>::value, T>::type>  { static constexpr INLINE_VAR auto value = ParameterInfo::Type::ENUM;   };
template<class T> struct ParamType<T, typename EnableForFieldTypes<T>::type>       { static constexpr INLINE_VAR auto value = ParameterInfo::Type::STRUCT; };
template<class T> struct ParamType<T, typename std::enable_if<std::is_union<T>::value, T>::type> { static constexpr INLINE_VAR auto value = ParameterInfo::Type::UNION;  };

template<ParameterInfo::Type Kind>
struct ParamEnum { using type = void; };

template<> struct ParamEnum<ParameterInfo::Type::U8    > { using type = uint8_t;  };
template<> struct ParamEnum<ParameterInfo::Type::S8    > { using type = int8_t;   };
template<> struct ParamEnum<ParameterInfo::Type::U16   > { using type = uint16_t; };
template<> struct ParamEnum<ParameterInfo::Type::S16   > { using type = int16_t;  };
template<> struct ParamEnum<ParameterInfo::Type::U32   > { using type = uint32_t; };
template<> struct ParamEnum<ParameterInfo::Type::S32   > { using type = int32_t;  };
template<> struct ParamEnum<ParameterInfo::Type::U64   > { using type = uint64_t; };
template<> struct ParamEnum<ParameterInfo::Type::S64   > { using type = int64_t;  };
template<> struct ParamEnum<ParameterInfo::Type::FLOAT > { using type = float;    };
template<> struct ParamEnum<ParameterInfo::Type::DOUBLE> { using type = double;   };

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
    constexpr ParameterInfo& paramInfo = FieldInfo<FieldType>::PARAMETERS[I].type;
    using T = ParamEnum<paramInfo.type>::type;
    return *static_cast<T*>(paramInfo.accessor(&field));
}


namespace utils
{
    // Gets a void pointer to the member identified by Ptr, given an
    // instance of field passed by void pointer.
    template<class Field, class T, T Field::*Ptr>
    void* access(void* p)
    {
        return &(static_cast<Field*>(p)->*Ptr);
    }

    // Converts a pointer to a struct to a pointer to FunctionSelector.
    // Note: this assumes the function selector is the very first parameter,
    // no non-empty base classes, etc. and is legal c++.
    inline void* accessFunctionSelector(void* p)
    {
        return static_cast<FunctionSelector*>(p);
    }
}

// Function selector MUST be the first struct member (or not a member at all)!
static constexpr INLINE_VAR ParameterInfo FUNCTION_PARAMETER = {
    /* .name          = */ "function",
    /* .docs          = */ "Standard MIP function selector",
    /* .type          = */ ParameterInfo::Type::ENUM,
    /* .accessor      = */ utils::accessFunctionSelector,
    /* .byte_offset   = */ 0,
    /* .functions     = */ {true,true,true,true,true},
    /* .count         = default */
    /* .counter_idx   = default */
    /* .union_index   = default */
    /* .union_value   = default */
};


struct EnumInfo
{
    struct Entry
    {
        const char* name  = nullptr;
        uint32_t    value = 0;
    };

    const char*                  name    = nullptr;
    const char*                  docs    = nullptr;
    ParameterInfo::Type          type    = ParameterInfo::Type::NONE;
    std::initializer_list<Entry> entries;
};

struct StructInfo
{
    const char* name  = nullptr;
    const char* title = nullptr;
    const char* docs  = nullptr;
    std::initializer_list<ParameterInfo> parameters;
};




template<class FieldType, size_t ParamIndex, class ParamType>
ParamType get(FieldType& field)
{
}

template<size_t ParameterIndex, class FieldType>
auto get(FieldType& field)
{
    const ParameterInfo& paramInfo = FieldInfo<FieldType>::PARAMETERS[ParameterIndex];


}



struct Example : public FieldStruct
{
    static constexpr INLINE_VAR std::initializer_list<ParameterInfo> PARAMETERS = {
        {
            /*.name          = */ "Enable",
            /*.docs          = */ "Enables the thing",
            /*.type          = */ ParameterInfo::Type::BOOL,
            /*.accessor      = */ nullptr,//[](void*)->void*{return nullptr;},
            /*.byte_offset   = */ 0,
            /*.functions     = */ {true, false, false, false, false},
        },
        {
            /*.name          = */ "Count",
            /*.docs          = */ "Number of things",
            /*.type          = */ ParameterInfo::Type::U8,
            /*.accessor      = */ nullptr, //[](void*)->void*{return nullptr;},
            /*.struct_offset = */ 1,
            /*.functions     = */ {true, true, false, false, false},
        }
    };

    bool    enable = false;
    uint8_t count  = 0;

    auto asTuple() { return std::make_tuple(std::ref(enable), std::ref(count)); }
    auto asTuple() const { return std::make_tuple(enable, count); }
};



} // namespace mip
