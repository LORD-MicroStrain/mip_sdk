#pragma once

#include <type_traits>
#include <limits>
#include <stdint.h>
#include <assert.h>


namespace microstrain
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// bitmaskXXX
//

////////////////////////////////////////////////////////////////////////////////
///@brief Creates a bitmask with 1s in the first N bits, i.e. bits [0..N).
///
///@tparam T
///        An unsigned integral type.
///
///@param nbits
///       The number of 1 bits starting at location 0. Must not exceed the
///       number of bits in T (e.g. nbits <= 32 for T=uint32_t).
///
///@returns the bitmask
///
template<typename T>
constexpr T bitmaskFirstN(unsigned int nbits)
{
    static_assert(std::is_unsigned<T>::value, "Register type must be unsigned.");
    assert(nbits <= std::numeric_limits<T>::digits);

    nbits &= std::numeric_limits<T>::digits - 1;
    return T( T(1u) << nbits ) - 1;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Creates a bitmask with N '1' bits starting at bit I.
///
/// If bitI == bitJ, then the result will have exactly one bit set.
///
///@tparam T
///        An unsigned integral type.
///
///@param bitI
///       The 0-based index of the first 1 bit. Must be LESS than the number of
///       bits in T (and not equal).
///@param bitJ
///       The 0-based index of the the last 1 bit. Must be greater than or equal
///       to bitI and cannot exceed the number of bits in T.
///
///@returns the bitmask
///
template<typename T>
constexpr T bitmaskN(unsigned int bitI, unsigned int nbits)
{
    assert(bitI < std::numeric_limits<T>::digits);
    return bitmaskFirstN<T>(nbits) << bitI;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Creates a bitmask with bits [bitI..bitJ) set to 1 (excludes bit J).
///
///@tparam T
///        An unsigned integral type.
///
///@param bitI
///       The 0-based index of the first 1 bit. Must not exceed the number of
///       bits in T.
///@param bitJ
///       The 0-based index of the first 0 bit after the last 1 bit. Must be
///       greater than or equal to bitI and cannot exceed the number of bits in T.
///
///@returns the bitmask
///
template<typename T>
constexpr T bitmaskItoJ(unsigned int bitI, unsigned int bitJ)
{
    assert(bitI <= bitJ);
    return bitmaskFirstN<T>(bitJ) & ~bitmaskFirstN<T>(bitI);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Creates a bitmask with bits [bitI..bitJ] set to 1 (includes bit J).
///
/// If bitI == bitJ, then the result will have exactly one bit set.
///
///@tparam T
///        An unsigned integral type.
///
///@param bitI
///       The 0-based index of the first 1 bit. Must not exceed the number of
///       bits in T.
///@param bitJ
///       The 0-based index of the the last 1 bit. Must be greater than or equal
///       to bitI and cannot exceed the number of bits in T.
///
///@returns the bitmask
///
template<typename T>
constexpr T bitmaskIthruJ(unsigned int bitI, unsigned int bitJ)
{
    return bitmaskFirstN<T>(bitJ+1) & ~bitmaskFirstN<T>(bitI);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// testBit
//


////////////////////////////////////////////////////////////////////////////////
///@brief Checks if a specific bit in a value is set.
///
///@tparam RegType An integral type. Let the compiler deduce this in most cases.
///
///@param reg Value containing the bit to check.
///@param bit The 0-based index of the bit to check in val.
///           Must be less than the number of bits in RegType.
///
///@returns True if the bit is set, false otherwise.
///
template<typename RegType>
constexpr bool testBit(RegType reg, unsigned int bit)
{
    assert(bit < std::numeric_limits<RegType>::digits);  // bit must be less than the number of bits in T.

    return (reg >> bit) & 1u;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a specific bit in a variable to a specified value.
///
///@tparam RegType An integral type. Let the compiler deduce this.
///
///@param reg   Value to be modified.
///@param bit   The 0-based index of the bit to modify.
///             Must be less than the number of bits in RegType.
///@param value Sets the bit to this value. Default true.
///
template<typename RegType>
constexpr void setBit(RegType& reg, unsigned int bit, bool value=true)
{
    assert(bit < std::numeric_limits<RegType>::digits);  // bit must be less than the number of bits in T.

    using T = typename std::remove_volatile<RegType>::type;
    T tmp = reg;
    tmp &= ~T(T(1) << bit);
    tmp |= T(value) << bit;
    reg = tmp;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// getRegisterBitXXX
//


////////////////////////////////////////////////////////////////////////////////
///@brief Extracts a bitfield from a variable value.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg   Register value.
///@param bitI  The 0-based index of the first bit to extract. Must be less
///             than the number of bits in RegType.
///@param nbits How many consecutive bits to extract.
///
///@returns The value stored in bits [bitI..bitI+N) as the same type as reg.
///
template<typename RegType>
constexpr RegType getBitsN(RegType reg, unsigned int bitI, unsigned int nbits)
{
    assert(bitI < std::numeric_limits<RegType>::digits);  // BitI must be less than the number of bits in T.

    return (reg >> bitI) & bitmaskFirstN<RegType>(nbits);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Extracts a bitfield from a variable value.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg  Register value.
///@param bitI  The 0-based index of the first bit to extract. Must be less
///             than the number of bits in RegType.
///@param bitJ The 0-based index of the bit after last bit to extract. Must be
///            greater than or equal to bitI and cannot exceed the number of
///            bits in RegType.
///
///@returns The value stored in bits [bitI..bitJ) as the same type as reg.
///
template<typename RegType>
constexpr RegType getBitsItoJ(RegType reg, unsigned int bitI, unsigned int bitJ)
{
    assert(bitI < std::numeric_limits<RegType>::digits);  // BitI must be less than the number of bits in T.

    return (reg >> bitI) & bitmaskItoJ<RegType>(bitI, bitJ);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Extracts a bitfield from a variable value.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg  Register value.
///@param bitI The 0-based index of the first bit to extract. Must be less
///            than the number of bits in RegType.
///@param bitJ The 0-based index of the last bit to extract. Must be greater than
///            or equal to bitI and less than the number of bits in RegType.
///
///@returns The value stored in bits [bitI..bitJ] as the same type as reg.
///
template<typename RegType>
constexpr RegType getBitsIthruJ(RegType reg, unsigned int bitI, unsigned int bitJ)
{
    assert(bitI < std::numeric_limits<RegType>::digits);  // BitI must be less than the number of bits in T.

    return (reg >> bitI) & bitmaskIthruJ<RegType>(bitI, bitJ);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// setRegisterBitXXX
//


////////////////////////////////////////////////////////////////////////////////
///@brief Sets a series of consecutive bits in a variable to 1.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg   Value to be modified.
///@param bitI  The 0-based index of the first affected bit.
///@param nbits Number of consecutive bits to set to '1'.
///
template<typename RegType>
constexpr void setBitsN(RegType& reg, unsigned int bitI, unsigned int nbits)
{
    using T = typename std::remove_volatile<RegType>::type;

    T tmp = reg;
    tmp |= bitmaskN<T>(bitI, nbits);
    reg = tmp;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a series of consecutive bits in a variable to 1.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg  Value to be modified.
///@param bitI The 0-based index of the first affected bit.
///@param bitJ The 0-based index of the bit after the last affected bit.
///
template<typename RegType>
constexpr void setBitsItoJ(RegType& reg, unsigned int bitI, unsigned int bitJ)
{
    using T = typename std::remove_volatile<RegType>::type;

    T tmp = reg;
    tmp |= bitmaskItoJ<T>(bitI, bitJ);
    reg = tmp;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a series of consecutive bits in a variable to 1.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg  Value to be modified.
///@param bitI The 0-based index of the first affected bit.
///@param bitJ The 0-based index of the last affected bit.
///
template<typename RegType>
constexpr void setBitsIthruJ(RegType& reg, unsigned int bitI, unsigned int bitJ)
{
    using T = typename std::remove_volatile<RegType>::type;

    T tmp = reg;
    tmp |= bitmaskIthruJ<T>(bitI, bitJ);
    reg = tmp;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// clearRegisterBitXXX
//


////////////////////////////////////////////////////////////////////////////////
///@brief Sets a series of consecutive bits in a variable to 0.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg   Value to be modified.
///@param bitI  The 0-based index of the first affected bit.
///@param nbits Number of consecutive bits to set to '0'.
///
template<typename RegType>
constexpr void clearBitsN(RegType& reg, unsigned int bitI, unsigned int nbits)
{
    using T = typename std::remove_volatile<RegType>::type;

    T tmp = reg;
    tmp &= ~bitmaskN<T>(bitI, nbits);
    reg = tmp;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a series of consecutive bits in a variable to 0.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg  Value to be modified.
///@param bitI The 0-based index of the first affected bit.
///@param bitJ The 0-based index of the bit after the last affected bit.
///
template<typename RegType>
constexpr void clearBitsItoJ(RegType& reg, unsigned int bitI, unsigned int bitJ)
{
    using T = typename std::remove_volatile<RegType>::type;

    T tmp = reg;
    tmp &= ~bitmaskItoJ<T>(bitI, bitJ);
    reg = tmp;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a series of consecutive bits in a variable to 0.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///
///@param reg  Value to be modified.
///@param bitI The 0-based index of the first affected bit.
///@param bitJ The 0-based index of the last affected bit.
///
template<typename RegType>
constexpr void clearBitsIthruJ(RegType& reg, unsigned int bitI, unsigned int bitJ)
{
    using T = typename std::remove_volatile<RegType>::type;

    T tmp = reg;
    tmp &= ~bitmaskIthruJ<T>(bitI, bitJ);
    reg = tmp;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// modifyBitsXXX
//


////////////////////////////////////////////////////////////////////////////////
///@brief Writes a value to a specific set of bits in a variable as if it were
///       a bitfield.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///@tparam ValType An integral type used for the input value. Let the compiler deduce this.
///
///@param reg   Value to be modified.
///@param bitI  The 0-based index of the first bit of value in reg.
///@param nbits Number of consecutive bits used to represent the value in reg.
///@param value Value to insert into reg at bit I.
///
template<typename RegType, typename ValType>
constexpr void setBitsN(RegType& reg, unsigned int bitI, unsigned int nbits, ValType value)
{
    static_assert(std::is_unsigned<RegType>::value, "Register type must be unsigned.");

    using T = typename std::remove_volatile<RegType>::type;

    T valueT = value;

    assert(bitI < std::numeric_limits<T>::digits);         // bit I is within T
    assert(nbits <= std::numeric_limits<T>::digits-bitI);  // Sane number of bits
    assert((valueT & ~bitmaskFirstN<T>(nbits)) == 0);      // Value has no bits outside the window

    T tmp = reg;  // Do all operations on a local copy, in case T is volatile.

    const T mask = bitmaskN<T>(bitI, nbits);

    tmp &= ~mask;
    tmp |= (valueT << bitI) & mask;

    reg = tmp;  // Write back to register.
}

////////////////////////////////////////////////////////////////////////////////
///@brief Writes a value to a specific set of bits in a variable as if it were
///       a bitfield.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///@tparam ValType An integral type used for the input value. Let the compiler deduce this.
///
///@param reg  Value to be modified.
///@param bitI The 0-based index of the first bit of value in reg.
///@param bitJ The 0-based index of the bit after the last affected bit.
///@param value Value to insert into reg at bit I.
///
template<typename RegType, typename ValType>
constexpr void setBitsItoJ(RegType& reg, unsigned int bitI, unsigned int bitJ, ValType value)
{
    assert(bitJ >= bitI);  // J comes after I (non-negative number of bits)

    return setBitsN(reg, bitI, bitJ-bitI, value);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Writes a value to a specific set of bits in a variable as if it were
///       a bitfield.
///
///@tparam RegType An unsigned integral type. Let the compiler deduce this.
///@tparam ValType An integral type used for the input value. Let the compiler deduce this.
///
///@param reg  Value to be modified.
///@param bitI The 0-based index of the first bit of value in reg.
///@param bitJ The 0-based index of the last bit of value in reg.
///@param value Value to insert into reg at bit I.
///
template<typename RegType, typename ValType>
constexpr void modifyBitsIthruJ(RegType& reg, unsigned int bitI, unsigned int bitJ, ValType value)
{
    return setBitsItoJ(reg, bitI, bitJ+1, value);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// BitfieldMemberXXX
//


namespace detail
{
    template<class TestType, bool IsEnum = std::is_enum<TestType>::value>
    struct BaseIntegerType
    {
        static_assert(std::is_integral<TestType>::value, "Type must be an integer.");

        using type = TestType;
    };

    template<class TestType>
    struct BaseIntegerType<TestType, true>
    {
        static_assert(std::is_enum<TestType>::value, "Expected an enum.");

        using type = typename std::underlying_type<TestType>::type;
    };
}


////////////////////////////////////////////////////////////////////////////////
///@brief A class which represents part of a bitfield.
///
/// This is used to store the range of bits, to avoid repeating them in get/set
/// function implementations and other situations.
///
/// Usage would typically be like this:
///@code{.cpp}
///  using Part1Bits = BitfieldMemberItoJ<uint16_t, 0, 12>;  // First 12 bits, holding a u16
///  using Part2Bits = BitfieldMemberItoJ<uint8_t, 12, 16>;  // Last 4 bits as a u8
///
///  uint16_t getPart1() const { return Part1Bits::get(m_value); }
///  uint16_t getPart2() const { return Part2Bits::get(m_value); }
///  void setPart1(uint16_t p1) { Part1Bits::set(m_value, p1); }
///  void setPart2(uint16_t p2) { Part2Bits::set(m_value, p2); }
///
///  uint16_t m_value;
///@endcode
///
///@tparam T     Type of the bitfield member. Must be integral or a non-scoped enum.
///@tparam BitI The 0-based index of the first bit used to represent this value.
///@tparam Nbits Number of bits in the field used to represent this value.
///
template<class T, unsigned int BitI, unsigned int Nbits>
struct BitfieldMemberN
{
    using Type = T;
    using BaseType = typename detail::BaseIntegerType<T>::type;
    static constexpr inline unsigned int bitI = BitI;
    static constexpr inline unsigned int nbits = Nbits;

    static_assert(std::numeric_limits<BaseType>::digits >= nbits, "T doesn't have enough bits");

    template<typename RegType>
    static constexpr T get(RegType reg) { return static_cast<Type>(getBitsN(reg, bitI, nbits)); }

    template<typename RegType>
    static constexpr void set(RegType& reg, Type value) { setBitsN(reg, bitI, nbits, static_cast<BaseType>(value)); }

    template<typename RegType>
    static constexpr void clear(RegType& reg) { clearBitsN(reg, bitI, nbits); }
};


////////////////////////////////////////////////////////////////////////////////
///@brief A class which represents part of a bitfield (see BitfieldMemberN)
///
///@tparam T    Type of the bitfield member.
///@tparam BitI The 0-based index of the first bit used to represent this value.
///@tparam BitJ The 0-based index of the bit after the last bit.
///
template<class T, unsigned int BitI, unsigned int BitJ>
struct BitfieldMemberItoJ : public BitfieldMemberN<T, BitI, BitJ-BitI> {};


////////////////////////////////////////////////////////////////////////////////
///@brief A class which represents part of a bitfield (see BitfieldMemberN)
///
///@tparam T    Type of the bitfield member.
///@tparam BitI The 0-based index of the first bit used to represent this value.
///@tparam BitJ The 0-based index of the last bit used to represent this value.
///
template<class T, unsigned int BitI, unsigned int BitJ>
struct BitfieldMemberIthruJ : public BitfieldMemberItoJ<T, BitI, BitJ+1> {};



////////////////////////////////////////////////////////////////////////////////
///@brief Helper type to get an integer of at least N bytes in size.
///
template<unsigned int nbytes>
struct IntegerWithBytes {};

template<> struct IntegerWithBytes<8>
{
    using Unsigned = uint64_t;
    using Signed   = int64_t;
};
template<> struct IntegerWithBytes<7> : IntegerWithBytes<8> {};
template<> struct IntegerWithBytes<6> : IntegerWithBytes<8> {};
template<> struct IntegerWithBytes<5> : IntegerWithBytes<8> {};
template<> struct IntegerWithBytes<4>
{
    using Unsigned = uint32_t;
    using Signed   = int32_t;
};
template<> struct IntegerWithBytes<3> : IntegerWithBytes<4> {};
template<> struct IntegerWithBytes<2>
{
    using Unsigned = uint16_t;
    using Signed   = int16_t;
};
template<> struct IntegerWithBytes<1>
{
    using Unsigned = uint8_t;
    using Signed   = int8_t;
};


////////////////////////////////////////////////////////////////////////////////
///@brief Helper type to get an integer of at least N bits in size.
///
template<unsigned int nbits>
struct IntegerWithBits : IntegerWithBytes< (nbits+7)/8 > {};


}  // namespace microstrain::common
