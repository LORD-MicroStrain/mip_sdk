#pragma once

#include <microstrain/common/index.hpp>
#include <microstrain/common/span.hpp>
#include <mip/mip_descriptors.hpp>

#include <stdint.h>


namespace mip::metadata
{
  template<class T>
  using Span = microstrain::Span<T>;

//struct EnumInfo;
//struct BitfieldInfo;
//struct UnionInfo;
//struct StructInfo;
//struct FieldInfo;
//struct ParameterInfo;


enum class Type
{
    NONE = 0,  ///< Invalid/unknown.

    CHAR,
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
    BITS,
    STRUCT,
    UNION,
};

struct TypeInfo
{
    //template<class Field, class T>
    //TypeInfo(T Field::*member);

    Type type = Type::NONE;

    const void* infoPtr = nullptr;
    //union
    //{
    //    const StructInfo   *si;
    //    const EnumInfo     *ei;
    //    const BitfieldInfo *bi;
    //    const UnionInfo    *ui;
    //};

    bool isBasicType() const { return type <= Type::DOUBLE; }
};

struct EnumInfo
{
    struct Entry
    {
        uint32_t    value = 0;
        const char* name  = nullptr;
        const char* docs  = nullptr;
    };

    const char*   name    = nullptr;
    const char*   docs    = nullptr;
    Type          type    = Type::NONE;

    Span<const Entry> entries;
};

struct BitfieldInfo : public EnumInfo {};


//struct ParameterInfo;  // Defined below

struct Attributes
{
    constexpr Attributes() = default;
    constexpr Attributes(bool w, bool r, bool s, bool l, bool d, bool e=false, bool x=false) : bits(0x00)
    {
        setCanWrite(w);
        setCanRead(r);
        setCanSave(s);
        setCanLoad(l);
        setCanReset(d);
        setNotSerialized(x);
        setIsEchoed(e);
    }

    constexpr bool any() const { return bits > 0; }

    constexpr bool canWrite() const { return bits & 0b00000001; }
    constexpr bool canRead()  const { return bits & 0b00000010; }
    constexpr bool canSave()  const { return bits & 0b00000100; }
    constexpr bool canLoad()  const { return bits & 0b00001000; }
    constexpr bool canReset() const { return bits & 0b00010000; }
    constexpr bool isNotSerialized()    const { return bits & 0b01000000; }
    constexpr bool isEchoed()           const { return bits & 0b10000000; }
    constexpr bool isSerialized() const { return !isNotSerialized(); }

    constexpr Attributes& setCanWrite      (bool w) { bits = (bits & 0b11111110) | (uint8_t(w)<<0); return *this; }
    constexpr Attributes& setCanRead       (bool r) { bits = (bits & 0b11111101) | (uint8_t(r)<<1); return *this; }
    constexpr Attributes& setCanSave       (bool s) { bits = (bits & 0b11111011) | (uint8_t(s)<<2); return *this; }
    constexpr Attributes& setCanLoad       (bool l) { bits = (bits & 0b11110111) | (uint8_t(l)<<3); return *this; }
    constexpr Attributes& setCanReset      (bool d) { bits = (bits & 0b11101111) | (uint8_t(d)<<4); return *this; }
    constexpr Attributes& setNotSerialized (bool x) { bits = (bits & 0b10111111) | (uint8_t(x)<<6); return *this; }
    constexpr Attributes& setIsEchoed      (bool e) { bits = (bits & 0b01111111) | (uint8_t(e)<<7); return *this; }

    constexpr bool has(mip::FunctionSelector function) const { return (bits >> (static_cast<uint8_t>(function) - static_cast<uint8_t>(mip::FunctionSelector::WRITE))) & 1; }

    uint8_t bits = 0x00;
};
static constexpr inline Attributes ALL_FUNCTIONS = {true, true, true, true, true};
static constexpr inline Attributes NO_FUNCTIONS  = {false, false, false, false, false};


struct ParameterInfo
{
    struct Count
    {
        constexpr Count() = default;
        constexpr Count(uint8_t n) : count(n) {}
        constexpr Count(uint8_t n, microstrain::Id id) : count(n), paramIdx(id) {}

        uint8_t         count    = 1;  ///< Fixed size if paramIdx unassigned.
        microstrain::Id paramIdx = {}; ///< If assigned, specifies parameter that holds the actual runtime count.

        constexpr bool isFixed() const { return count > 0 && !paramIdx.isAssigned(); }
        constexpr bool hasCounter() const { return paramIdx.isAssigned(); }
    };

    struct Condition
    {
        enum class Type : uint8_t
        {
            NONE     = 0,  ///< No condition, member always valid
            ENUM     = 1,  ///< Enum value selector (e.g. for parameters in unions)
            //PRODUCT = 2,  ///< Depends on product variant (TBD)
            //OPTIONAL = 2,  ///< Parameter can be omitted (TBD)
        };

        Type            type     = Type::NONE; ///< Type of condition.
        microstrain::Id paramIdx = {};         ///< Index of enum parameter identifying whether this parameter is enabled.
        uint16_t        value    = 0;          ///< Value of the enum parameter which activates this parameter.

        constexpr bool hasCondition() const { return type != Type::NONE; }
    };

    using Accessor = void* (*)(void*);

    const char* name = nullptr;     ///< Programmatic name (e.g. for printing or language bindings).
    const char* docs = nullptr;     ///< Human-readable documentation.
    TypeInfo    type;               ///< Data type.
    Accessor    accessor = nullptr; ///< Obtains a reference to the member variable.
    Attributes  attributes;         ///< This parameter is required for the specified function selectors.
    Count       count;              ///< Number of instances for arrays.
    Condition   condition;          ///< For conditionally-enabled parameters like those in unions.
};


struct StructInfo
{
    const char* name  = nullptr;
    const char* title = nullptr;
    const char* docs  = nullptr;

    Span<const ParameterInfo> parameters;
};

struct UnionInfo : public StructInfo {};

struct FieldInfo : public StructInfo
{
    CompositeDescriptor descriptor  = {0x00, 0x00};
    Attributes          functions   = {false, false, false, false, false};
    bool                proprietary = false;
    const FieldInfo*    response    = nullptr;
};


///@brief Gets the size of a basic type (including bitfields and enums if class_ is not NULL).
///
constexpr size_t sizeForBasicType(Type type, const void* info=nullptr)
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
        return sizeForBasicType(static_cast<const EnumInfo *>(info)->type);

    case Type::BITS:
        if(!info)
            return 0;
        return sizeForBasicType(static_cast<const BitfieldInfo *>(info)->type);

    default:
        return 0;
    }
}
constexpr size_t sizeForBasicType(const TypeInfo& type) { return sizeForBasicType(type.type, type.infoPtr); }


} // namespace mip::metadata
