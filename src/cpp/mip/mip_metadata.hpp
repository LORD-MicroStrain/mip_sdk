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

    struct TypeInfo
    {
        template<class Field, class T>
        TypeInfo(T Field::*member);

        Type type;
        uint8_t struct_offset = 0;
        StructInfo*
    };

    const char* name = nullptr;     ///< Programmatic name (e.g. for printing or language bindings).
    const char* docs = nullptr;     ///< Human-readable documentation.
    Type        type = Type::NONE;  ///< Data type.
    uint8_t     struct_offset = 0;  ///< Offset into the corresponding struct (e.g. using offsetof).
    uint8_t     byte_offset   = 0;  ///< Offset "on the wire" or in a serialized buffer, in bytes.
    FuncBits    functions;          ///< This parameter is required for the specified function selectors.
    uint8_t     count = 1;          ///< Number of elements. 0 if size is specified at runtime.
    int8_t      counter_idx = 0;    ///< When count == 0, this specifies the parameter holding the runtime count, relative to this parameter's index.
    int8_t      union_index = 0;    ///< When type is UNION, this specifies the parameter that controls which union member is active (the discriminator).
    uint16_t    union_value = 0;    ///< When in a union, this specifies the value of the discriminator parameter that selects this member.
};


template<class T, class=void> static constexpr inline ParameterInfo::Type paramTypeFromCppType = ParameterInfo::Type::NONE;

template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<bool,     void> = ParameterInfo::Type::BOOL;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<uint8_t,  void> = ParameterInfo::Type::U8;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType< int8_t,  void> = ParameterInfo::Type::S8;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<uint16_t, void> = ParameterInfo::Type::U16;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType< int16_t, void> = ParameterInfo::Type::S16;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<uint32_t, void> = ParameterInfo::Type::U32;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType< int32_t, void> = ParameterInfo::Type::S32;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<uint64_t, void> = ParameterInfo::Type::U64;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType< int64_t, void> = ParameterInfo::Type::S64;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<float,    void> = ParameterInfo::Type::FLOAT;
template<> static constexpr inline ParameterInfo::Type paramTypeFromCppType<double,   void> = ParameterInfo::Type::DOUBLE;
template<class T> static constexpr inline ParameterInfo::Type paramTypeFromCppType<T, typename std::enable_if<std::is_enum<T>::value, T>::type> = ParameterInfo::Type::ENUM;
template<class T> static constexpr inline ParameterInfo::Type paramTypeFromCppType<Bitfield<T>, T> = ParameterInfo::Type::BITFIELD;
template<class T> static constexpr inline ParameterInfo::Type paramTypeFromCppType<T, typename std::enable_if<isField<T>::value, T>::type> = ParameterInfo::Type::STRUCT;
template<class T> static constexpr inline ParameterInfo::Type paramTypeFromCppType<T, typename std::enable_if<std::is_union<T>::value, T>::type> = ParameterInfo::Type::UNION;

//template<class T, class=void>
//struct ParamTypeFromCppType;
//
//template<>
//struct ParamTypeFromCppType<bool,void> : public std::integral_constant ParameterInfo::Type type =

//constexpr ParameterInfo::Type typeEnumFromCppType(bool   ) { return ParameterInfo::Type::BOOL; }
//constexpr ParameterInfo::Type typeEnumFromCppType(uint8_t) { return ParameterInfo::Type::U8; }
//constexpr ParameterInfo::Type typeEnumFromCppType( int8_t) { return ParameterInfo::Type::S8; }
//constexpr ParameterInfo::Type typeEnumFromCppType(uint16_t) { return ParameterInfo::Type::U16; }
//constexpr ParameterInfo::Type typeEnumFromCppType( int16_t) { return ParameterInfo::Type::S16; }
//constexpr ParameterInfo::Type typeEnumFromCppType(uint32_t) { return ParameterInfo::Type::U32; }
//constexpr ParameterInfo::Type typeEnumFromCppType( int32_t) { return ParameterInfo::Type::S32; }
//constexpr ParameterInfo::Type typeEnumFromCppType(uint64_t) { return ParameterInfo::Type::U64; }
//constexpr ParameterInfo::Type typeEnumFromCppType( int64_t) { return ParameterInfo::Type::S64; }
//constexpr ParameterInfo::Type typeEnumFromCppType(float   ) { return ParameterInfo::Type::FLOAT; }
//constexpr ParameterInfo::Type typeEnumFromCppType(double  ) { return ParameterInfo::Type::DOUBLE; }
//
//template<class T>
//constexpr typename std::enable_if<std::is_enum<T>::value, ParameterInfo::Type>::type
///*ParameterInfo::Type*/ typeEnumFromCppType() { return ParameterInfo::Type::ENUM; }
//
//template<class T>
//constexpr ParameterInfo::Type typeEnumFromCppType(Bitfield<T>) { return ParameterInfo::Type::BITFIELD; }
//
//template<class T>
//constexpr ParameterInfo::Type typeEnumFromCppType(typename std::enable_if<std::is_class<T>::value, T>::type) { return ParameterInfo::Type::STRUCT; }
//
//template<class T>
//constexpr ParameterInfo::Type typeEnumFromCppType(typename std::enable_if<std::is_union<T>::value, T>::type) { return ParameterInfo::Type::UNION; }


static constexpr inline ParameterInfo FUNCTION_PARAMETER = {
    /* .type          = */ ParameterInfo::Type::ENUM,
    /* .name          = */ "function",
    /* .docs          = */ "Standard MIP function selector",
    /* .byte_offset   = */ 0,
    /* .struct_offset = */ 0, // Function selector MUST be the first struct member.
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
    std::initializer_list<Entry> entries = {};
};

struct StructInfo
{
    const char* name  = nullptr;
    const char* title = nullptr;
    const char* docs  = nullptr;
    std::initializer_list<ParameterInfo> parameters;
};


// Type trait class to be specialized for each field type.
template<class FieldType>
struct FieldInfo;



template<class FieldType, size_t ParamIndex, class ParamType>
ParamType get(FieldType& field)
{
    return
}

template<size_t ParameterIndex, class FieldType>
auto get(FieldType& field)
{
    const ParameterInfo& paramInfo = FieldInfo<FieldType>::PARAMETERS[ParameterIndex];


}



struct Example : public FieldStruct
{
    static constexpr inline std::initializer_list<ParameterInfo> PARAMETERS = {
        {
            /*.type          = */ ParameterInfo::Type::BOOL,
            /*.name          = */ "Enable",
            /*.docs          = */ "Enables the thing",
            /*.byte_offset   = */ 0,
            /*.struct_offset = */ 0,
            /*.functions     = */ {true, false, false, false, false},
        },
        {
            /*.type          = */ ParameterInfo::Type::U8,
            /*.name          = */ "Count",
            /*.docs          = */ "Number of things",
            /*.byte_offset   = */ 1,
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
