#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "../utils/serialization.h"

#ifdef __cplusplus

#include <tuple>
#include <type_traits>

namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus


typedef struct mip_descriptor_rate
{
    uint8_t  descriptor;
    uint16_t decimation;
} mip_descriptor_rate;

void insert_mip_descriptor_rate(mip_serializer* serializer, const mip_descriptor_rate* self);
void extract_mip_descriptor_rate(mip_serializer* serializer, mip_descriptor_rate* self);

#define DECLARE_MIP_VECTOR_TYPE(n,type,name) \
typedef type name[n]; \
\
void insert_##name(mip_serializer* serializer, const name self); \
void extract_##name(mip_serializer* serializer, name self);

DECLARE_MIP_VECTOR_TYPE(3, float,  mip_vector3f)
DECLARE_MIP_VECTOR_TYPE(4, float,  mip_vector4f)
DECLARE_MIP_VECTOR_TYPE(9, float,  mip_matrix3f)
DECLARE_MIP_VECTOR_TYPE(3, double, mip_vector3d)
DECLARE_MIP_VECTOR_TYPE(4, double, mip_vector4d)
DECLARE_MIP_VECTOR_TYPE(9, double, mip_matrix3d)
DECLARE_MIP_VECTOR_TYPE(4, float,  mip_quatf)

#undef DECLARE_MIP_VECTOR_TYPE

//typedef struct mip_vector3f
//{
//    float m_data[3];
//} mip_vector3f;
//
//void insert_mip_vector3(mip_serializer* serializer, const mip_vector3* self);
//void extract_mip_vector3(mip_serializer* serializer, mip_vector3* self);
//
//typedef struct mip_vector4f
//{
//    float m_data[4];
//} mip_vector4f;
//
//void insert_mip_vector3(mip_serializer* serializer, const mip_vector3* self);
//void extract_mip_vector3(mip_serializer* serializer, mip_vector3* self);
//
//typedef struct mip_matrix3
//{
//    float m[9];
//} mip_matrix3;
//
//void insert_mip_matrix3(mip_serializer* serializer, const mip_matrix3* self);
//void extract_mip_matrix3(mip_serializer* serializer, mip_matrix3* self);


#ifdef __cplusplus

} // extern "C"
} // namespace "C"


using DescriptorRate = C::mip_descriptor_rate;

inline void insert(Serializer& serializer, const DescriptorRate& self) { return C::insert_mip_descriptor_rate(&serializer, &self); }
inline void extract(Serializer& serializer, DescriptorRate& self) { return C::extract_mip_descriptor_rate(&serializer, &self); }


template<typename T, size_t N>
using Vector = T[N];

using Vector3f = Vector<float,3>;
using Vector4f = Vector<float,4>;
using Matrix3f = Vector<float,9>;
using Vector3d = Vector<double,3>;
using Vector4d = Vector<double,4>;
using Matrix3d = Vector<double,9>;

using Quatf = Vector4f;

template<typename T, size_t N>
void insert(Serializer& serializer, const Vector<T,N>& v) { for(size_t i=0; i<N; i++) insert(serializer, v[i]); }

template<typename T, size_t N>
void extract(Serializer& serializer, Vector<T,N>& v) { for(size_t i=0; i<N; i++) extract(serializer, v[i]); }

//////////////////////////////////////////////////////////////////////////////////
/////@brief A mathematical vector object used by mip commands and data.
/////
//template<typename T, size_t N>
//class Vector
//{
//public:
//    Vector() { *this = T(0); }
//    Vector(const Vector&) = default;
//    Vector(const T (&ptr)[N]) { copyFrom(ptr); }
//
//    Vector& operator=(const Vector&) = default;
//    Vector& operator=(const T* ptr) { copyFrom(ptr); return this; }
//    Vector& operator=(T value) { for(size_t i=0; i<N; i++) m_data[i]=0; return *this; }
//
//    T& operator[](unsigned int i) { return m_data[i]; }
//    T operator[](unsigned int i) const { return m_data[i]; }
//
//    T* ptr() { return m_data; }
//    const T* ptr() const { return m_data; }
//
//    operator T*() { return m_data; }
//    operator const T*() const { return m_data; }
//
//    template<typename U=T>
//    void copyFrom(const U* ptr) { for(size_t i=0; i<N; i++) m_data[i] = ptr[i]; }
//
//    template<typename U=T>
//    void copyTo(U* ptr) const { for(size_t i=0; i<N; i++) ptr[i] = m_data[i]; }
//
//    static constexpr size_t size() { return N; }
//
//protected:
//    T m_data[N];
//};
//
/////@brief A 2D vector which provides .x() and .y() accessors.
/////
//template<typename T>
//class Vector2 : public Vector<T,2>
//{
//    using Vector<T,2>::Vector;
//
//    template<typename U>
//    Vector2(U x_, U y_) { x()=x_; y()=y_;}
//
//    T& x() { return this->m_data[0]; }
//    T& y() { return this->m_data[1]; }
//
//    T x() const { return this->m_data[0]; }
//    T y() const { return this->m_data[1]; }
//};
//
/////@brief A 3D vector which provides .x(), .y() and .z() accessors.
/////
//template<typename T>
//class Vector3 : public Vector<T,3>
//{
//    //using Vector<T,3>::Vector;
//    template<typename U>
//    Vector3(const U* ptr) { copyFrom(ptr); }
//
//    template<typename U>
//    Vector3(U x_, U y_, U z_) { x()=x_; y()=y_; z()=z_; }
//
//    T& x() { return this->m_data[0]; }
//    T& y() { return this->m_data[1]; }
//    T& z() { return this->m_data[2]; }
//
//    T x() const { return this->m_data[0]; }
//    T y() const { return this->m_data[1]; }
//    T z() const { return this->m_data[2]; }
//};
//
/////@brief A 4D vector which provides .x(), .y(), .z(), and .w() accessors.
/////
//template<typename T>
//class Vector4 : public Vector<T,4>
//{
//    using Vector<T,4>::Vector;
//
//    template<typename U>
//    Vector4(U x_, U y_, U z_, U w_) { x()=x_; y()=y_; z()=z_; w()=w_; }
//
//    T& x() { return this->m_data[0]; }
//    T& y() { return this->m_data[1]; }
//    T& z() { return this->m_data[2]; }
//    T& w() { return this->m_data[3]; }
//
//    T x() const { return this->m_data[0]; }
//    T y() const { return this->m_data[1]; }
//    T z() const { return this->m_data[2]; }
//    T w() const { return this->m_data[3]; }
//};
//
/////@brief A quaternion which provides .x(), .y(), .z(), and .w() accessors.
/////
//template<typename T>
//class Quaternion : public Vector4<T>
//{
//    using Vector4<T>::Vector;
//};
//
//typedef Vector2<float> Vector2f;
//typedef Vector3<float> Vector3f;
//typedef Vector4<float> Vector4f;
//
//typedef Vector2<double> Vector2d;
//typedef Vector3<double> Vector3d;
//typedef Vector4<double> Vector4d;
//
//typedef Quaternion<float> Quatf;
//typedef Vector<float, 9> Matrix3f;
//
//template<typename T, size_t N>
//void insert(Serializer& serializer, const Vector<T,N>& self)
//{
//    for(size_t i=0; i<N; i++)
//        insert(serializer, self[i]);
//}
//
//template<typename T, size_t N>
//void extract(Serializer& serializer, Vector<T,N>& self)
//{
//    for(size_t i=0; i<N; i++)
//        extract(serializer, self[i]);
//}
//
//template<typename T>
//void insert(Serializer& serializer, const Quaternion<T>& self)
//{
//    insert(serializer, self.w());  // w comes first in mip
//    insert(serializer, self.x());
//    insert(serializer, self.y());
//    insert(serializer, self.z());
//}
//
//template<typename T>
//void extract(Serializer& serializer, Quaternion<T>& self)
//{
//    extract(serializer, self.w());  // w comes first in mip
//    extract(serializer, self.x());
//    extract(serializer, self.y());
//    extract(serializer, self.z());
//}


} // namespace mip

#endif // __cplusplus
