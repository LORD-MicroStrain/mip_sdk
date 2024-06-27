#pragma once

#include <mip/mip_descriptors.hpp>

#include <span>
#include <stdint.h>


namespace mip::metadata
{

struct EnumInfo;
struct BitfieldInfo;
struct UnionInfo;
struct StructInfo;
struct FieldInfo;
struct ParameterInfo;


enum class Type
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
        const char* name  = nullptr;
        uint32_t    value = 0;
    };

    const char*   name    = nullptr;
    const char*   docs    = nullptr;
    Type          type    = Type::NONE;
    std::span<const Entry> entries;
};

struct BitfieldInfo : public EnumInfo {};


struct ParameterInfo;  // Defined below

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
static constexpr inline FuncBits ALL_FUNCTIONS = {true,true,true,true,true};
static constexpr inline FuncBits NO_FUNCTIONS  = {false, false, false, false, false};


struct StructInfo
{
    const char* name  = nullptr;
    const char* title = nullptr;
    const char* docs  = nullptr;
    std::span<const ParameterInfo> parameters;
};

struct UnionInfo : public StructInfo {};

struct FieldInfo : public StructInfo
{
    CompositeDescriptor descriptor = {0x00, 0x00};
    FuncBits   functions   = {false, false, false, false, false};
    bool       proprietary = false;
    FieldInfo* response    = nullptr;
};



struct ParameterInfo
{
    using Accessor = void* (*)(void*);

    const char* name = nullptr;     ///< Programmatic name (e.g. for printing or language bindings).
    const char* docs = nullptr;     ///< Human-readable documentation.
    TypeInfo    type;               ///< Data type.
    Accessor    accessor = nullptr; ///< Obtains a reference to the member variable.
    uint8_t     byte_offset   = 0;  ///< Offset "on the wire" or in a serialized buffer, in bytes.
    FuncBits    functions;          ///< This parameter is required for the specified function selectors.
    uint8_t     count = 1;          ///< Number of elements. 0 if size is specified at runtime.
    int8_t      counter_idx = 0;    ///< When count == 0, this specifies the parameter holding the runtime count, relative to this parameter's index.
    int8_t      union_index = 0;    ///< When type is UNION, this specifies the parameter that controls which union member is active (the discriminator).
    uint16_t    union_value = 0;    ///< When in a union, this specifies the value of the discriminator parameter that selects this member.
};


} // namespace mip::metadata
