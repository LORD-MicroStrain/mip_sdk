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

template<class T>
constexpr T add_sat(T x, T y) noexcept
{
    static_assert(std::is_integral<T>::value, "Saturating arithmetic is only allowed on integral types");
    static_assert(std::is_unsigned<T>::value, "Signed saturating arithmetic not implemented");

    T tmp = x + y;
    return (tmp >= x) ? tmp : std::numeric_limits<T>::max();
}

template<class T>
constexpr T sub_sat(T x, T y) noexcept
{
    static_assert(std::is_integral<T>::value, "Saturating arithmetic is only allowed on integral types");
    static_assert(std::is_unsigned<T>::value, "Signed saturating arithmetic not implemented");

    return (x > y) ? (x - y) : T(0);
}

template<class T, class U>
constexpr T saturate_cast(U x) noexcept
{
    static_assert(std::is_unsigned<T>::value, "Signed saturating arithmetic not implemented");
    static_assert(std::is_unsigned<U>::value, "Signed saturating arithmetic not implemented");

    if( x < std::numeric_limits<T>::min() )
        return std::numeric_limits<T>::min();
    else if( x > std::numeric_limits<T>::max() )
        return std::numeric_limits<T>::max();
    else
        return x;
}

#endif  // __cpp_lib_saturation_arithmetic


template<class T, class U>
constexpr void assign_sat(T& destination, U value)
{
    destination = saturate_cast<T>(value);
}

template<class T, class U>
constexpr T& add_sat(T& x, U y)
{
    T yt = saturate_cast<T>(y);
    return x = add_sat(x, yt);
}

template<class T, class U>
constexpr T& sub_sat(T& x, U y)
{
    T yt = saturate_cast<T>(y);
    return x = sub_sat(x, yt);
}



template<class T>
class SaturatingInt
{
    static_assert(std::is_integral<T>::value(), "SaturatingInteger type must be integral");

    SaturatingInt() = default;
    SaturatingInt(T value) : m_value(value) {}

    operator T() const { return m_value; }

    SaturatingInt& operator++() { if(m_value < std::numeric_limits<T>::max()) ++m_value; return *this; }
    SaturatingInt operator++(int) { auto tmp=*this; ++(*this); return tmp; }

    SaturatingInt& operator--() { if(m_value > std::numeric_limits<T>::min()) --m_value; return *this; }
    SaturatingInt operator--(int) { auto tmp=*this; --(*this); return tmp; }

    SaturatingInt& operator+(T other) { m_value = add_sat(m_value, other); return *this; }
    SaturatingInt& operator-(T other) { m_value = sub_sat(m_value, other); return *this; }

private:
    T m_value = 0;
};



} // namespace microstrain
