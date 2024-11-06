#pragma once

#include <mip/mip_serialization.hpp>

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{
///
///@defgroup MipCommands_cpp  Mip Commands
///@brief Contains all MIP command definitions.
///
///@defgroup MipData_cpp  Mip Data
///@brief Contains all MIP data definitions.
///
///@}
////////////////////////////////////////////////////////////////////////////////


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

struct DescriptorRate
{
    uint8_t descriptor;
    uint16_t decimation;

    size_t insert(Serializer& buffer) const { return buffer.insert(descriptor, decimation); }
    size_t extract(Serializer& buffer) { return buffer.extract(descriptor, decimation); }
};


//////////////////////////////////////////////////////////////////////////////////
///@brief Vector is a wrapper around an array of some type T, usually float or double.
///
/// Implicit conversion to/from C-style pointers is provided to allow simple
/// integration with code using plain arrays.
///
template<typename T, size_t N>
struct Vector
{
    /// Default constructor, no initialization.
    Vector() {}

    /// Set all elements to this value (typically 0).
    ///@param value
    template<typename U>
    Vector(U value) { fill(value); }

    /// Construct from a C array of known size.
    ///@param ptr
    template<typename U>
    Vector(const U (&ptr)[N]) { copyFrom(ptr, N); }

    /// Construct from a C array of different size (smaller or larger vector).
    ///@param ptr
    template<typename U, size_t M>
    explicit Vector(const U (&ptr)[M]) { static_assert(M>=N, "Input array is too small"); copyFrom(ptr, M); }

    /// Construct from a pointer and size.
    ///@param ptr Pointer to data to copy. Can be NULL if n==0.
    ///@param n   Number of elements to copy. Clamped to N.
    template<typename U>
    explicit Vector(const U* ptr, size_t n) { copyFrom(ptr, n); }

    /// Construct from individual elements or a braced init list.
    ///@param u The first value (typically X).
    ///@param v The value value (typically Y).
    ///@param rest Additional optional values (typically none, Z, or Z and W).
    template<typename U, typename V, typename... Rest>
    Vector(U u, V v, Rest... rest) : m_data{u, v, rest...} {}

    /// Copy constructor.
    Vector(const Vector&) = default;

    /// Assignment operator.
    Vector& operator=(const Vector&) = default;

    /// Assignment operator from different type (e.g. float to double).
    template<typename U>
    Vector& operator=(const Vector<U,N>& other) { copyFrom(other, N); return *this; }


    typedef T Array[N];

#if _MSC_VER < 1930
    // MSVC 2017 has a bug which causes operator[] to be ambiguous.
    // See https://stackoverflow.com/questions/48250560/msvc-error-c2593-when-overloading-const-and-non-const-conversion-operator-return
    operator T*() { return m_data; }
    operator const T*() const { return m_data; }
#else
    /// Implicitly convert to a C-style array (rather than a pointer) so size information is preserved.
    operator Array&() { return m_data; }
    operator const Array&() const { return m_data; }
#endif

    /// Explicitly convert to a C-style array.
    Array& data() { return m_data; }
    const Array& data() const { return m_data; }

    /// Fill all elements with a given value.
    template<typename U>
    void fill(U value) { for(size_t i=0; i<N; i++) m_data[i]=value; }

    /// Copy data from a pointer and size to this vector.
    ///@param ptr Pointer to data. Can be NULL if n==0.
    ///@param n   Number of elements in ptr. Clamped to N.
    template<typename U>
    void copyFrom(const U* ptr, size_t n) { if(n>N) n=N; for(size_t i=0; i<n; i++) m_data[i] = (T)ptr[i]; }

    /// Get the size of the array
    size_t size() const { return N; }

private:
    T m_data[N];
};

using Vector3f = Vector<float,3>;
using Vector4f = Vector<float,4>;
using Matrix3f = Vector<float,9>;
using Vector3d = Vector<double,3>;
using Vector4d = Vector<double,4>;
using Matrix3d = Vector<double,9>;

using Quatf = Vector4f;

template<typename T, size_t N>
size_t insert(Serializer& serializer, const Vector<T,N>& v) { return serializer.insert(v.data()); }

template<typename T, size_t N>
size_t extract(Serializer& serializer, Vector<T,N>& v) { return serializer.extract(v.data()); }

///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
