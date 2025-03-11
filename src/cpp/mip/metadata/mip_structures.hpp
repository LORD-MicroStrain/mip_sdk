#pragma once

#include <microstrain/index.hpp>
#include <microstrain/span.hpp>
#include <mip/mip_descriptors.hpp>

#include <stdint.h>


namespace mip::metadata
{
  template<class T>
  using Span = microstrain::Span<T>;

struct EnumInfo;
struct BitfieldInfo;
struct UnionInfo;
struct StructInfo;
struct FieldInfo;
struct ParameterInfo;


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

    const EnumInfo*     enumPointer()     const { return (type == Type::ENUM  ) ? static_cast<const EnumInfo*    >(infoPtr) : nullptr; }
    const BitfieldInfo* bitfieldPointer() const { return (type == Type::BITS  ) ? static_cast<const BitfieldInfo*>(infoPtr) : nullptr; }
    const UnionInfo*    unionPointer()    const { return (type == Type::UNION ) ? static_cast<const UnionInfo*   >(infoPtr) : nullptr; }
    const StructInfo*   structPointer()   const { return (type == Type::STRUCT) ? static_cast<const StructInfo*  >(infoPtr) : nullptr; }

    bool isBasicType() const { return type <= Type::DOUBLE; }
    bool isCustom()  const { return type != Type::NONE && type >= Type::ENUM; }
    bool isClass()   const { return type != Type::NONE && type >= Type::STRUCT; }
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

    const char* nameForValue(uint32_t value) const
    {
        for(const Entry& entry : entries)
        {
            if(entry.value == value)
                return entry.name;
        }
        return nullptr;
    }
};

struct BitfieldInfo : public EnumInfo {};


// Basic attribute bitfield for fields and parameters.
struct FunctionBits
{
    constexpr FunctionBits() = default;
    constexpr FunctionBits(bool w, bool r, bool s, bool l, bool d) : bits(0x00)
    {
        setCanWrite(w);
        setCanRead(r);
        setCanSave(s);
        setCanLoad(l);
        setCanReset(d);
    }

    constexpr bool canWrite() const { return bits & 0b00000001; }
    constexpr bool canRead()  const { return bits & 0b00000010; }
    constexpr bool canSave()  const { return bits & 0b00000100; }
    constexpr bool canLoad()  const { return bits & 0b00001000; }
    constexpr bool canReset() const { return bits & 0b00010000; }

    constexpr FunctionBits& setCanWrite      (bool w) { bits = (bits & 0b11111110) | (uint8_t(w)<<0); return *this; }
    constexpr FunctionBits& setCanRead       (bool r) { bits = (bits & 0b11111101) | (uint8_t(r)<<1); return *this; }
    constexpr FunctionBits& setCanSave       (bool s) { bits = (bits & 0b11111011) | (uint8_t(s)<<2); return *this; }
    constexpr FunctionBits& setCanLoad       (bool l) { bits = (bits & 0b11110111) | (uint8_t(l)<<3); return *this; }
    constexpr FunctionBits& setCanReset      (bool d) { bits = (bits & 0b11101111) | (uint8_t(d)<<4); return *this; }

    constexpr bool has(mip::FunctionSelector function) const { return (bits >> (static_cast<uint8_t>(function) - static_cast<uint8_t>(mip::FunctionSelector::WRITE))) & 1; }

    uint8_t bits = 0x00;
};

// Extended attributes for parameters.
struct ParamAttributes : public FunctionBits
{
    constexpr ParamAttributes() = default;
    constexpr ParamAttributes(FunctionBits bits) : FunctionBits(bits) {}
    constexpr ParamAttributes(bool w, bool r, bool s, bool l, bool d, bool e=false, bool x=false) : FunctionBits(w,r,s,l,d)
    {
        setNotSerialized(x);
        setIsEchoed(e);
    }

    constexpr bool isNotSerialized()    const { return bits & 0b01000000; }
    constexpr bool isEchoed()           const { return bits & 0b10000000; }
    constexpr bool isSerialized() const { return !isNotSerialized(); }

    constexpr ParamAttributes& setNotSerialized (bool x) { bits = (bits & 0b10111111) | (uint8_t(x)<<6); return *this; }
    constexpr ParamAttributes& setIsEchoed      (bool e) { bits = (bits & 0b01111111) | (uint8_t(e)<<7); return *this; }
};

// Extended attributes for fields.
struct FieldAttributes : public FunctionBits
{
    constexpr FieldAttributes() = default;
    constexpr FieldAttributes(FunctionBits bits) : FunctionBits(bits) {}
    constexpr FieldAttributes(bool w, bool r, bool s, bool l, bool d, bool p=false) : FunctionBits(w,r,s,l,d) { setProprietary(p); }

    constexpr bool hasFunctionSelector() const { return (bits & 0b00011111) > 0; }

    constexpr bool isProprietary() const { return bits & 0b10000000; }

    constexpr FieldAttributes& setProprietary(bool p) { bits = (bits & 0b01111111) | (uint8_t(p)<<7); return *this; }
};

static constexpr inline FunctionBits ALL_FUNCTIONS = {true, true, true, true, true};
static constexpr inline FunctionBits NO_FUNCTIONS  = {false, false, false, false, false};


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

    const char*     name = nullptr;     ///< Programmatic name (e.g. for printing or language bindings).
    const char*     docs = nullptr;     ///< Human-readable documentation.
    TypeInfo        type;               ///< Data type.
    Accessor        accessor = nullptr; ///< Obtains a reference to the member variable.
    ParamAttributes attributes;         ///< This parameter is required for the specified function selectors.
    Count           count;              ///< Number of instances for arrays.
    Condition       condition;          ///< For conditionally-enabled parameters like those in unions.
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
    FieldAttributes     functions   = {false, false, false, false, false,  false};
    const FieldInfo*    response    = nullptr;
};

struct DescriptorSetInfo
{
    uint8_t                      descriptor = mip::INVALID_DESCRIPTOR_SET;
    const char*                  name       = nullptr;
    Span<const FieldInfo* const> fields     = {};

    //const FieldInfo* findField(uint8_t field_desc) const
    //{
    //    // Binary search assumes fields are sorted by descriptor.
    //    auto it = std::lower_bound(fields.begin(), fields.end(), field_desc, [](const FieldInfo* a, uint8_t b){ return a->descriptor.fieldDescriptor < b; });
    //    return (it != fields.end()) && ((*it)->descriptor.fieldDescriptor==field_desc) ? *it : nullptr;
    //}
};



} // namespace mip::metadata
