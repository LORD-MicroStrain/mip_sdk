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
inline std::optional<uint64_t> extractIntegralType(mip::metadata::Type type, microstrain::Span<const uint8_t> payload, size_t offset=0)
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

constexpr void getOffsetForAlignment(const TypeInfo& type, size_t &current_offset);

constexpr size_t sizeForBasicType(const TypeInfo& type);

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the size of a basic type (including bitfields and enums if info
///       is not NULL).
///
constexpr size_t sizeForBasicType(const Type type, const void* info = nullptr)
{
    switch (type)
    {
        case Type::CHAR:
        {
            return sizeof(char);
        }
        case Type::BOOL:
        {
            return sizeof(bool);
        }
        case Type::U8:
        {
            return sizeof(uint8_t);
        }
        case Type::S8:
        {
            return sizeof(int8_t);
        }
        case Type::U16:
        {
            return sizeof(uint16_t);
        }
        case Type::S16:
        {
            return sizeof(int16_t);
        }
        case Type::U32:
        {
            return sizeof(uint32_t);
        }
        case Type::S32:
        {
            return sizeof(int32_t);
        }
        case Type::U64:
        {
            return sizeof(uint64_t);
        }
        case Type::S64:
        {
            return sizeof(int64_t);
        }
        case Type::FLOAT:
        {
            return sizeof(float);
        }
        case Type::DOUBLE:
        {
            return sizeof(double);
        }
        case Type::ENUM:
        {
            if (!info)
            {
                return 0;
            }

            return sizeForBasicType(static_cast<const EnumInfo*>(info)->type);
        }
        case Type::BITS:
        {
            if (!info)
            {
                return 0;
            }

            return sizeForBasicType(static_cast<const BitfieldInfo*>(info)->type);
        }
        case Type::STRUCT:
        {
            size_t total_size = 0;

            if (!info)
            {
                return total_size;
            }

            if (const StructInfo* struct_info = static_cast<const StructInfo*>(info))
            {
                for (const ParameterInfo& param : struct_info->parameters)
                {
                    getOffsetForAlignment(param.type, total_size);
                    total_size += sizeForBasicType(param.type);
                }
            }

            return total_size;
        }
        case Type::UNION:
        {
            size_t highest_size = 0;

            if (!info)
            {
                return highest_size;
            }

            if (const StructInfo* struct_info = static_cast<const StructInfo*>(info))
            {
                for (const ParameterInfo& param : struct_info->parameters)
                {
                    const size_t param_size = sizeForBasicType(param.type);

                    if (param_size > highest_size)
                    {
                        highest_size = param_size;
                    }
                }
            }

            return highest_size;
        }
        default:
        {
            return 0;
        }
    }
}
constexpr size_t sizeForBasicType(const TypeInfo& type) { return sizeForBasicType(type.type, type.infoPtr); }

constexpr size_t alignmentForBasicType(const TypeInfo& type);

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the size of a basic type (including bitfields and enums if info
///       is not NULL).
///
constexpr size_t alignmentForBasicType(const Type type, const void* info = nullptr)
{
    switch (type)
    {
        case Type::CHAR:
        {
            return alignof(char);
        }
        case Type::BOOL:
        {
            return alignof(bool);
        }
        case Type::U8:
        {
            return alignof(uint8_t);
        }
        case Type::S8:
        {
            return alignof(int8_t);
        }
        case Type::U16:
        {
            return alignof(uint16_t);
        }
        case Type::S16:
        {
            return alignof(int16_t);
        }
        case Type::U32:
        {
            return alignof(uint32_t);
        }
        case Type::S32:
        {
            return alignof(int32_t);
        }
        case Type::U64:
        {
            return alignof(uint64_t);
        }
        case Type::S64:
        {
            return alignof(int64_t);
        }
        case Type::FLOAT:
        {
            return alignof(float);
        }
        case Type::DOUBLE:
        {
            return alignof(double);
        }
        case Type::ENUM:
        {
            if (!info)
            {
                return 0;
            }

            return alignmentForBasicType(static_cast<const EnumInfo*>(info)->type);
        }
        case Type::BITS:
        {
            if (!info)
            {
                return 0;
            }

            return alignmentForBasicType(static_cast<const BitfieldInfo*>(info)->type);
        }
        case Type::UNION:
        case Type::STRUCT:
        {
            size_t highest_alignment = 0;

            if (!info)
            {
                return highest_alignment;
            }

            if (const StructInfo* struct_info = static_cast<const StructInfo*>(info))
            {
                for (const ParameterInfo& param : struct_info->parameters)
                {
                    const size_t param_alignment = alignmentForBasicType(param.type);

                    if (param_alignment > highest_alignment)
                    {
                        highest_alignment = param_alignment;
                    }
                }
            }

            return highest_alignment;
        }
        default:
        {
            return 0;
        }
    }
}
constexpr size_t alignmentForBasicType(const TypeInfo& type) { return alignmentForBasicType(type.type, type.infoPtr); }

constexpr void getOffsetForAlignment(const TypeInfo& type, size_t &current_offset)
{
    const size_t align = alignmentForBasicType(type);

    if (align != 0)
    {
        const size_t align_offset = current_offset % align;

        if (align_offset != 0)
        {
            current_offset += align - align_offset;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Get a pointer to a parameter given the offset of the member in the
///       class/struct
///
///@warning This does not work for nested types, I.E. ClassType.member1.member2
///
///@param offset     The offset of the member within the struct/class. Use the
///                  C++ offsetof or a variant to get this value
///
///@param field_info Metadata field info to look up parameter info for
///
///@return A pointer to the parameter info if found, otherwise nullptr
///
const inline ParameterInfo* getParameterInfoFromStructMemberOffset(const size_t offset, const StructInfo& field_info)
{
    size_t check_offset = 0;

    for (const ParameterInfo& param : field_info.parameters)
    {
        getOffsetForAlignment(param.type, check_offset);

        if (offset == check_offset)
        {
            return &param;
        }

        if (offset < check_offset)
        {
            break;
        }

        check_offset += sizeForBasicType(param.type);
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Get a pointer to a parameter given the offset of the member in the
///       class/struct
///
///@param base_offset   The offset of the member within the struct/class. Use
///                     the C++ offsetof or a variant to get this value
///
///@param nested_offset The offset of the member within the struct/class. Use
///                     the C++ offsetof or a variant to get this value
///
///@param field_info    Metadata field info to look up parameter info for
///
///@return A pointer to the parameter info if found, otherwise nullptr
///
const inline ParameterInfo* getNestedParameterInfoFromStructMemberOffset(const size_t base_offset, const size_t nested_offset, const StructInfo& field_info)
{
    if (const ParameterInfo* base_param = getParameterInfoFromStructMemberOffset(base_offset, field_info))
    {
        switch (base_param->type.type)
        {
            case Type::STRUCT:
            {
                const StructInfo* struct_info = static_cast<const StructInfo*>(base_param->type.infoPtr);
                return getParameterInfoFromStructMemberOffset(nested_offset, *struct_info);
            }
            case Type::UNION:
            {
                const UnionInfo* union_info = static_cast<const UnionInfo*>(base_param->type.infoPtr);
                return getParameterInfoFromStructMemberOffset(nested_offset, *union_info);
            }
            default:
            {
                break;
            }
        }
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Get a pointer to a parameter given the member of a MIP type
///
/// @param struct_info Metadata struct info to find the member of
///
/// @param param_name The name of the member to find info for
///
/// @param dummy       Dummy variable for macro compile checks and code
///                    completion
///
///@return A pointer to the parameter info if found, otherwise nullptr
///
const inline ParameterInfo* getParameterInfoForMember(const StructInfo& struct_info, const char* param_name, const size_t dummy = 0)
{
    // Dummy variable for macro checks
    (void)dummy;

    for (const ParameterInfo& param : struct_info.parameters)
    {
        if (strcmp(param.name, param_name) == 0)
        {
            return &param;
        }
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Get a pointer to a parameter given the member of a nested
///       class/struct
///
///@param MemberParameter   Member parameter to get the type of which has
///                         metadata
///
///@param NestedMemberField The member field within the nested parameter type
///
///@see mip::metadata::utils::getParameterInfoForMember
///
///@return A pointer to the parameter info if found, otherwise nullptr
///
#define GET_MIP_METADATA_NESTED_PARAM_INFO(MemberParameter, NestedMemberField) \
mip::metadata::utils::getParameterInfoForMember(mip::metadata::MetadataFor<decltype(MemberParameter)>::value, #NestedMemberField, \
offsetof(decltype(MemberParameter), NestedMemberField)) // Dummy entry to allow offsetof to check the member exists

////////////////////////////////////////////////////////////////////////////////
///@brief Get a pointer to a parameter given the member of a class/struct
///
///@param MipType MIP type that has metadata
///
///@param MemberField The member field within the MIP type
///
///@see mip::metadata::utils::getParameterInfoForMember
///
///@return A pointer to the parameter info if found, otherwise nullptr
///
#define GET_MIP_METADATA_PARAM_INFO(MipType, MemberField) \
mip::metadata::utils::getParameterInfoForMember(mip::metadata::MetadataFor<MipType>::value, #MemberField, \
offsetof(MipType, MemberField)) // Dummy entry to allow offsetof to check the member exists
} // namespace mip::metadata
