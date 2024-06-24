#pragma once

#include "structures.hpp"


namespace mip::metadata::utils
{

//
// Gets the "Type" enum value given a C++ template type.
//
template<class T, class=void> struct ParamType { static inline constexpr auto value = Type::NONE; };

template<> struct ParamType<bool,     void> { static constexpr inline auto value = Type::BOOL;   };
template<> struct ParamType<uint8_t,  void> { static constexpr inline auto value = Type::U8;     };
template<> struct ParamType< int8_t,  void> { static constexpr inline auto value = Type::S8;     };
template<> struct ParamType<uint16_t, void> { static constexpr inline auto value = Type::U16;    };
template<> struct ParamType< int16_t, void> { static constexpr inline auto value = Type::S16;    };
template<> struct ParamType<uint32_t, void> { static constexpr inline auto value = Type::U32;    };
template<> struct ParamType< int32_t, void> { static constexpr inline auto value = Type::S32;    };
template<> struct ParamType<uint64_t, void> { static constexpr inline auto value = Type::U64;    };
template<> struct ParamType< int64_t, void> { static constexpr inline auto value = Type::S64;    };
template<> struct ParamType<float,    void> { static constexpr inline auto value = Type::FLOAT;  };
template<> struct ParamType<double,   void> { static constexpr inline auto value = Type::DOUBLE; };
template<class T> struct ParamType<Bitfield<T>, void> { static constexpr inline auto value = Type::BITFIELD; };
template<class T> struct ParamType<T, typename std::enable_if<std::is_enum<T>::value, T>::type>  { static constexpr inline auto value = Type::ENUM;   };
template<class T> struct ParamType<T, typename EnableForFieldTypes<T>::type>                     { static constexpr inline auto value = Type::STRUCT; };
template<class T> struct ParamType<T, typename std::enable_if<std::is_union<T>::value, T>::type> { static constexpr inline auto value = Type::UNION;  };


//
// Gets the template type given a compile-time "Type" enum value.
// Note: this only works with basic/built-in/arithmetic types, not structs/enums/etc.
//
template<Type Kind>
struct ParamEnum { using type = void; };

template<> struct ParamEnum<Type::BOOL  > { using type = bool;     };
template<> struct ParamEnum<Type::U8    > { using type = uint8_t;  };
template<> struct ParamEnum<Type::S8    > { using type = int8_t;   };
template<> struct ParamEnum<Type::U16   > { using type = uint16_t; };
template<> struct ParamEnum<Type::S16   > { using type = int16_t;  };
template<> struct ParamEnum<Type::U32   > { using type = uint32_t; };
template<> struct ParamEnum<Type::S32   > { using type = int32_t;  };
template<> struct ParamEnum<Type::U64   > { using type = uint64_t; };
template<> struct ParamEnum<Type::S64   > { using type = int64_t;  };
template<> struct ParamEnum<Type::FLOAT > { using type = float;    };
template<> struct ParamEnum<Type::DOUBLE> { using type = double;   };


// Gets a void pointer to the member identified by Ptr, given an
// instance of field passed by void pointer.
template<class Field, class T, T Field::*Ptr>
void* access(void* p)
{
    return &(static_cast<Field*>(p)->*Ptr);
}


static constexpr inline EnumInfo FUNCTION_SELECTOR_ENUM = {
    .name = "FunctionSelector",
    .docs = "",
    .type = Type::U8,
    .entries = {
        { "WRITE",   (uint8_t)FunctionSelector::WRITE },
        { "READ",    (uint8_t)FunctionSelector::READ  },
        { "SAVE",    (uint8_t)FunctionSelector::SAVE  },
        { "LOAD",    (uint8_t)FunctionSelector::LOAD  },
        { "DEFAULT", (uint8_t)FunctionSelector::RESET },
    }
};

inline void* accessFunctionSelector(void* p) { return static_cast<FunctionSelector*>(p); }

static constexpr inline ParameterInfo FUNCTION_SELECTOR_PARAM = {
    /* .name          = */ "function",
    /* .docs          = */ "Standard MIP function selector",
    /* .type          = */ {Type::ENUM, (const void*)&FUNCTION_SELECTOR_ENUM},
    /* .accessor      = */ accessFunctionSelector,
    /* .byte_offset   = */ 0,
    /* .functions     = */ {true,true,true,true,true},
    /* .count         = default */
    /* .counter_idx   = default */
    /* .union_index   = default */
    /* .union_value   = default */
};

//template<class Field>
//constexpr ParameterInfo makeFunctionSelectorParam()
//{
//    return {
//        /* .name          = */ "function",
//        /* .docs          = */ "Standard MIP function selector",
//        /* .type          = */ {Type::ENUM, &FUNCTION_SELECTOR_ENUM_INFO},
//        /* .accessor      = */ utils::access<Field,FunctionSelector,&Field::function>,
//        /* .byte_offset   = */ 0,
//        /* .functions     = */ {true,true,true,true,true},
//        /* .count         = default */
//        /* .counter_idx   = default */
//        /* .union_index   = default */
//        /* .union_value   = default */
//    };
//}

//template<class Field>
//struct FunctionSelectorParam
//{
//    static constexpr inline ParameterInfo value = {
//        /* .name          = */ "function",
//        /* .docs          = */ "Standard MIP function selector",
//        /* .type          = */ Type::ENUM,
//        /* .accessor      = */ utils::access<Field,FunctionSelector,&Field::function>,
//        /* .byte_offset   = */ 0,  // Always the first parameter.
//        /* .functions     = */ {true,true,true,true,true},
//        /* .count         = default */
//        /* .counter_idx   = default */
//        /* .union_index   = default */
//        /* .union_value   = default */
//    };
//};

} // namespace mip::metadata
