#pragma once

#include <microstrain/common/platform.hpp>

#include "mip_descriptors.hpp"

namespace mip
{


struct FuncBits
{
    constexpr FuncBits() = default;
    constexpr FuncBits(bool w, bool r, bool s, bool l, bool d) : bits(0x00) { write(w); read(r); save(s); load(l); reset(d); }

    constexpr bool any() const { return bits > 0; }

    constexpr bool write() const { return bits & 0b00000001; }
    constexpr bool read()  const { return bits & 0b00000010; }
    constexpr bool save()  const { return bits & 0b00000100; }
    constexpr bool load()  const { return bits & 0b00001000; }
    constexpr bool reset() const { return bits & 0b00010000; }

    constexpr FuncBits& write(bool w) { bits = (bits & 0b11111110) | (uint8_t(w)<<0); return *this; }
    constexpr FuncBits& read (bool r) { bits = (bits & 0b11111101) | (uint8_t(r)<<0); return *this; }
    constexpr FuncBits& save (bool s) { bits = (bits & 0b11111011) | (uint8_t(s)<<0); return *this; }
    constexpr FuncBits& load (bool l) { bits = (bits & 0b11110111) | (uint8_t(l)<<0); return *this; }
    constexpr FuncBits& reset(bool d) { bits = (bits & 0b11101111) | (uint8_t(d)<<0); return *this; }

    constexpr bool has(mip::FunctionSelector function) const { return (bits >> (static_cast<uint8_t>(function) - static_cast<uint8_t>(mip::FunctionSelector::WRITE))) & 1; }

    uint8_t bits = 0x00;
};


struct ParameterInfo
{
    enum class Type
    {
        NONE = 0,
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

    Type        type = Type::NONE;
    const char* name = nullptr;
    const char* docs = nullptr;
    uint8_t     offset = 0;
    FuncBits    functions;
    uint8_t     count = 1;
    uint8_t     counter_idx = 0xFF;
    uint8_t     union_index = 0xFF;
    uint16_t    union_value = 0;
};

struct FieldStruct
{
    //static_assert(std::is_base_of<FieldStruct<T>, T>::value, "T must inherit from FieldStruct<T>");

private:
    // No direct instantiation or destruction,
    // this class must be inherited.
    FieldStruct() = default;
    ~FieldStruct() = default;
};

template<class T>
using isField = std::is_base_of<FieldStruct, T>;


struct Example : public FieldStruct
{
    static constexpr inline ParameterInfo PARAMETERS[] = {
        {
            /*.type        = */ ParameterInfo::Type::BOOL,
            /*.name        = */ "Enable",
            /*.docs        = */ "Enables the thing",
            /*.offset      = */ 0,
            /*.functions   = */ {true, false, false, false, false},
        },
        {
            /*.type        = */ ParameterInfo::Type::U8,
            /*.name        = */ "Count",
            /*.docs        = */ "Number of things",
            /*.offset      = */ 1,
            /*.functions   = */ {true, true, false, false, false},
        }
    };

    bool    enable = false;
    uint8_t count  = 0;

    auto asTuple() { return std::make_tuple(std::ref(enable), std::ref(count)); }
    auto asTuple() const { return std::make_tuple(enable, count); }
};



} // namespace mip

#include <microstrain/common/serialization.hpp>

// These functions must be in the microstrain namespace in order to be seen as overloads.
namespace microstrain
{

template<class T>
std::enable_if<mip::isField<T>::value, size_t>::type
/*size_t*/ insert(microstrain::Serializer& serializer, const T& field)
{
    //static_assert(std::is_base_of<mip::FieldStruct<T>, T>::value, "T doesn't derive from FieldStruct");

    return insert(serializer, static_cast<const T&>(field).asTuple());
}


} // namespace microstrain
