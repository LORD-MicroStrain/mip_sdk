#pragma once

#include <limits>
#include <type_traits>
#include <numeric>

namespace microstrain
{

#if __cpp_lib_saturation_arithmetic >= 202311L

using std::add_sat;
using std::sub_sat;
using std::mul_sat;
using std::div_sat;
using std::saturate_cast;

#else // __cpp_lib_saturation_arithmetic

//
// Incomplete implementations of the std saturated arithmetic library from C++26.
// Currently, these only work with UNSIGNED integers.
//

/// see https://en.cppreference.com/w/cpp/numeric/add_sat
///
template<class T>
constexpr T add_sat(T x, T y) noexcept
{
    static_assert(std::is_integral<T>::value, "Saturating arithmetic is only allowed on integral types");
    static_assert(std::is_unsigned<T>::value, "Saturating arithmetic is not tested for signed types");

    constexpr T maximum = std::numeric_limits<T>::max();
    constexpr T minimum = std::numeric_limits<T>::min();

    if constexpr(std::is_unsigned<T>::value)
    {
        return ((x + y) >= x) ? (x + y) : maximum;  // If x+y overflows, return max
    }
    else  // Signed
    {
        if(x >= 0)
        {
            if(y >= 0)
                return (maximum - x <= y) ? (x+y) : maximum;
            else
                return x + y;  // Adding opposite signs can never overflow
        }
        else  // x < 0
        {
            if(y >= 0)
                return x + y;  // Adding opposite signs can never overflow
            else // If the difference between x and min is less than -y, then x+y would underflow.
                return (minimum - x <= y) ? (x+y) : minimum;  // Same as (x - minium >= -y)
        }
    }
}

/// see https://en.cppreference.com/w/cpp/numeric/sub_sat
///
template<class T>
constexpr T sub_sat(T x, T y) noexcept
{
    static_assert(std::is_integral<T>::value, "Saturating arithmetic is only allowed on integral types");
    static_assert(std::is_unsigned<T>::value, "Saturating arithmetic is not tested for signed types");

    constexpr T maximum = std::numeric_limits<T>::max();
    constexpr T minimum = std::numeric_limits<T>::min();

    if constexpr(std::is_unsigned<T>::value)
    {
        return (x > y) ? (x-y) : 0u;
    }
    else  // Signed
    {
        if(x >= 0)
        {
            if(y >= 0)
                return x - y;  // Subtracting same signs can never overflow
            else  // Negative y, so adding to x
                return (-y >= maximum - x) ? (x - y) : maximum;
        }
        else  // x < 0
        {
            if(y >= 0)
                return (y <= minimum - x) ? (x - y) : minimum;  // Same as (x - minium >= -y)
            else // If the difference between x and min is less than -y, then x+y would underflow.
                return x - y;  // Subtracting same signs can never overflow
        }
    }
}


///@brief Convert an integral value to a different integral type while preventing overflow.
///
/// See https://en.cppreference.com/w/cpp/numeric/saturate_cast
///
///@tparam T Result type.
///@tparam U Source type. Let the compiler deduce this.
///
///@param x Value to cast.
///
///@returns The value of x, clamped to the valid range of T.
///
template<class T, class U>
constexpr T saturate_cast(U x) noexcept
{
    static_assert(std::is_integral<T>::value, "T must be an integral type");
    static_assert(std::is_integral<U>::value, "U must be an integral type");

    // This method takes advantage of the fact that comparing two same-signed values
    // is done by first promoting the smaller type to the larger type.
    // E.g. uint32_t(5) < uint64_t(10) is done as if both were uint64_t.
    // If the result type T is smaller, bounds checks are needed.
    // Trouble arises when the passed type U and result type T have differing signedness.
    // In that case, comparisons or operations involving the two can cause the signed type
    // to silently be converted to unsigned. Therefore, this function must check
    // 3 cases: same-signedness, unsigned T but signed U, and signed T but unsigned U.
    // See https://stackoverflow.com/q/47351654 and https://stackoverflow.com/a/46073296.

    // If both values have the same signedness
    if constexpr(std::is_signed<T>::value == std::is_signed<U>::value)
    {
        // If result type T is the same or larger, then it can directly hold the value.
        if constexpr(std::numeric_limits<T>::digits >= std::numeric_limits<U>::digits)
            return x;
        else  // T is a smaller type and bounds checks are needed.
        {
            constexpr T minimum = std::numeric_limits<T>::min();
            constexpr T maximum = std::numeric_limits<T>::max();

            if(x > static_cast<U>(maximum))
                return maximum;

            // Only check minimum if signed, otherwise compiler may warn about unsigned comparison with 0 always false.
            if constexpr(std::is_signed<U>::value)
            {
                if(x < static_cast<U>(minimum))
                    return minimum;
            }

            return x;
        }
    }
    // Unsigned result
    else if constexpr(std::is_unsigned<T>::value)
    {
        static_assert(std::is_signed<U>::value); // Sanity check
        using unsigned_U = typename std::make_unsigned<U>::type;

        constexpr T maximum = std::numeric_limits<T>::max();

        // Negative numbers are out of range for unsigned values.
        if(x <= 0)
            return 0;
        // Positive numbers need a bounds check (if result T is a smaller type).
        // X is positive, so safe to cast to unsigned and compare to maximum.
        else if(static_cast<unsigned_U>(x) > maximum)
            return maximum;
        else
            return x;
    }
    else  // Signed result
    {
        static_assert(std::is_signed<T>::value);   // Sanity check
        static_assert(std::is_unsigned<U>::value); // Sanity check

        using unsigned_T = typename std::make_unsigned<T>::type;

        // Maximum value of (signed) T, represented as an unsigned value.
        constexpr unsigned_T maximum = std::numeric_limits<T>::max();

        // Check upper limit since signed numbers have lower maximum.
        // If T is larger, then x > maximum is false.
        if(x > maximum)
            return maximum;
        else
            return x;
    }
}

#endif  // __cpp_lib_saturation_arithmetic


///@brief Assign an integral value to a destination variable, clamping its value to the range of the destination type.
///
///@tparam T Destination type. Let the compiler deduce this.
///@tparam U Source type. Let the compiler deduce this.
///
///@param[out] destination Variable in which to store the value.
///@param[in]  value       Value to store in destination.
///
template<class T, class U>
constexpr void assign_sat(T& destination, U value)
{
    destination = saturate_cast<T>(value);
}

///@brief Add a value to an accumulator and prevent it from overflowing.
///
///@tparam T Type of the accumulator. Let the compiler deduce this.
///@tparam U Type of the value being added. Let the compiler deduce this.
///
///@param[out] counter Accumulator variable to modify.
///@param[in]  value   Value to add to counter.
///
///@returns A reference the the accumulator.
///
template<class T, class U>
constexpr T& accum_sat(T& counter, U amount)
{
    return counter = add_sat<T>(counter, amount);
}

///@brief Subtract a value from an accumulator and prevent it from overflowing.
///
///@tparam T Type of the accumulator. Let the compiler deduce this.
///@tparam U Type of the value being subtracted. Let the compiler deduce this.
///
///@param[out] counter Accumulator variable to modify.
///@param[in]  value   Value to subtract from counter.
///
///@returns A reference the the accumulator.
///
template<class T, class U>
constexpr T& reduce_sat(T& counter, U amount)
{
    return counter = sub_sat<T>(counter, amount);
}


////////////////////////////////////////////////////////////////////////////////
///@brief A class representing an integer that saturates instead of overflowing.
///
///@tparam T Integer data type, e.g. int, uint8_t, int32_t, uint64_t.
///
template<class T>
class SaturatingInt
{
    static_assert(std::is_integral<T>::value(), "SaturatingInteger type must be integral");

    SaturatingInt() = default;
    SaturatingInt(T value) : m_value(value) {}

    operator T() const { return m_value; }

    SaturatingInt& operator++() { if(m_value < std::numeric_limits<T>::max()) ++m_value; return *this; }
    SaturatingInt operator++(int) { SaturatingInt tmp=*this; ++(*this); return tmp; }

    SaturatingInt& operator--() { if(m_value > std::numeric_limits<T>::min()) --m_value; return *this; }
    SaturatingInt operator--(int) { SaturatingInt tmp=*this; --(*this); return tmp; }

    SaturatingInt& operator+(T value) { accum_sat(m_value, value); return *this; }
    SaturatingInt& operator-(T value) { reduce_sat(m_value, value); return *this; }

private:
    T m_value = 0;
};



} // namespace microstrain
