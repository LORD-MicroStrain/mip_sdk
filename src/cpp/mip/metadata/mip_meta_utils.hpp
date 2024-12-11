#pragma once

#include "mip_structures.hpp"


namespace mip::metadata::utils
{

//
// Gets the "Type" enum value given a C++ template type.
//
template<class T, class=void> struct ParamType { static inline constexpr auto value = Type::NONE; };

template<> struct ParamType<char,     void> { static constexpr inline auto value = Type::CHAR;   };
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
template<class T> struct ParamType<Bitfield<T>, void> { static constexpr inline auto value = Type::BITS; };
template<class T> struct ParamType<T, typename std::enable_if<std::is_enum<T>::value, T>::type>  { static constexpr inline auto value = Type::ENUM;   };
template<class T> struct ParamType<T, typename EnableForFieldTypes<T>::type>                     { static constexpr inline auto value = Type::STRUCT; };
template<class T> struct ParamType<T, typename std::enable_if<std::is_union<T>::value, T>::type> { static constexpr inline auto value = Type::UNION;  };


//
// Gets the template type given a compile-time "Type" enum value.
// Note: this only works with basic/built-in/arithmetic types, not structs/enums/etc.
//
template<Type Kind>
struct ParamEnum { using type = void; };

template<> struct ParamEnum<Type::CHAR  > { using type = char;     };
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
/*template<class Field, class T, T Field::*Ptr>
void* access(void* p)
{
    return &(static_cast<Field*>(p)->*Ptr);
}

template<class Field, class T, auto Ptr>
void* access(void* p)
{
    Field* f = static_cast<Field*>(p);
    auto& param = f->*Ptr;
    return &param;
}*/

template<class Field, class T, auto Ptr>
void* access(void* p);


#ifdef MICROSTRAIN_HAS_OPTIONAL
///@brief Reads a value of the given type from a buffer.
///
inline std::optional<uint64_t> extractBasicType(mip::metadata::Type type, microstrain::Span<const uint8_t> payload, size_t offset=0)
{
    switch(type)
    {
    case mip::metadata::Type::BOOL:   return microstrain::extract<    bool, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::U8:     return microstrain::extract< uint8_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::S8:     return microstrain::extract<  int8_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::U16:    return microstrain::extract<uint16_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::S16:    return microstrain::extract< int32_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::U32:    return microstrain::extract<uint32_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::S32:    return microstrain::extract< int64_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::U64:    return microstrain::extract<uint64_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::S64:    return microstrain::extract< int64_t, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::FLOAT:  return microstrain::extract<   float, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    case mip::metadata::Type::DOUBLE: return microstrain::extract<  double, microstrain::serialization::Endian::big>(payload.data(), payload.size(), offset);
    default: return std::nullopt;
    }
}
#endif  // MICROSTRAIN_HAS_OPTIONAL

///@brief Determines the type used for serialization purposes.
///
/// This essentially translates bitfields and enums to their underlying types.
///
///@returns A basic type, such as u8, s32, float, etc. if possible.
///@returns Type::NONE if type is NONE or if it's a compound type like a struct or union.
///
inline Type serializedType(const TypeInfo& type)
{
    if (type.isBasicType())
        return type.type;

    if (const EnumInfo *ptr = type.enumPointer())
        return ptr->type;

    if (const BitfieldInfo *ptr = type.bitfieldPointer())
        return ptr->type;

    return Type::NONE;
}

///@brief Gets the size of a basic type (including bitfields and enums if class_ is not NULL).
///
constexpr size_t serializedSizeForBasicType(Type type, const void* info=nullptr)
{
    switch(type)
    {
    case Type::CHAR:
    case Type::BOOL:
    case Type::U8:
    case Type::S8:
        return 1;
    case Type::U16:
    case Type::S16:
        return 2;
    case Type::U32:
    case Type::S32:
    case Type::FLOAT:
        return 4;
    case Type::U64:
    case Type::S64:
    case Type::DOUBLE:
        return 8;

    case Type::ENUM:
        if(!info)
            return 0;
        return serializedSizeForBasicType(static_cast<const EnumInfo *>(info)->type);

    case Type::BITS:
        if(!info)
            return 0;
        return serializedSizeForBasicType(static_cast<const BitfieldInfo *>(info)->type);

    default:
        return 0;
    }
}
constexpr size_t serializedSizeForBasicType(const TypeInfo& type) { return serializedSizeForBasicType(type.type, type.infoPtr); }

constexpr const char* nameForBasicType(Type type)
{
    switch(type)
    {
    case Type::NONE:   return "none";
    case Type::BOOL:   return "bool";
    case Type::CHAR:   return "char";
    case Type::U8:     return "u8";
    case Type::S8:     return "s8";
    case Type::U16:    return "u16";
    case Type::S16:    return "s16";
    case Type::U32:    return "u32";
    case Type::S32:    return "s32";
    case Type::U64:    return "u64";
    case Type::S64:    return "s64";
    case Type::FLOAT:  return "float";
    case Type::DOUBLE: return "double";
    case Type::ENUM:   return "enum";
    case Type::BITS:   return "bitfield";
    case Type::STRUCT: return "struct";
    case Type::UNION:  return "union";
    default: return nullptr;
    }
}

constexpr const char* nameOfType(const TypeInfo& type)
{
    switch(type.type)
    {
    default: return nameForBasicType(type.type);
    case Type::ENUM:   return type.infoPtr ? static_cast<const EnumInfo*    >(type.infoPtr)->name : "enum";
    case Type::BITS:   return type.infoPtr ? static_cast<const BitfieldInfo*>(type.infoPtr)->name : "bitfield";
    case Type::STRUCT: return type.infoPtr ? static_cast<const StructInfo*  >(type.infoPtr)->name : "struct";
    case Type::UNION:  return type.infoPtr ? static_cast<const UnionInfo*   >(type.infoPtr)->name : "union";
    }
}

} // namespace mip::metadata
