#pragma once

#include "../platform.hpp"

#include <cstring>
#include <stdint.h>
#include <stddef.h>
#include <type_traits>

#if __cpp_lib_endian >= 201907L
#include <bit>
#endif

namespace microstrain
{
namespace serialization
{

#if __cpp_lib_endian >= 201907L
using Endian = std::endian;
#else
enum class Endian
{
    little,
    big,
    //native = little,  // Todo
};
#endif

//
// Write to buffer
//

namespace big_endian
{

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T) == 1, size_t>::type
/*size_t*/ write(uint8_t *buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t *>(&value)[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T) == 2, size_t>::type
/*size_t*/ write(uint8_t *buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t *>(&value)[1];
    buffer[1] = reinterpret_cast<const uint8_t *>(&value)[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T) == 4, size_t>::type
/*size_t*/ write(uint8_t *buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t *>(&value)[3];
    buffer[1] = reinterpret_cast<const uint8_t *>(&value)[2];
    buffer[2] = reinterpret_cast<const uint8_t *>(&value)[1];
    buffer[3] = reinterpret_cast<const uint8_t *>(&value)[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T) == 8, size_t>::type
/*size_t*/ write(uint8_t *buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t *>(&value)[7];
    buffer[1] = reinterpret_cast<const uint8_t *>(&value)[6];
    buffer[2] = reinterpret_cast<const uint8_t *>(&value)[5];
    buffer[3] = reinterpret_cast<const uint8_t *>(&value)[4];
    buffer[4] = reinterpret_cast<const uint8_t *>(&value)[3];
    buffer[5] = reinterpret_cast<const uint8_t *>(&value)[2];
    buffer[6] = reinterpret_cast<const uint8_t *>(&value)[1];
    buffer[7] = reinterpret_cast<const uint8_t *>(&value)[0];
    return sizeof(T);
}

} // namespace big_endian
namespace little_endian
{

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ write(uint8_t *buffer, T value)
{
    std::memcpy(buffer, &value, sizeof(value));
    return sizeof(value);
}

} // namespace little_endian


template<Endian E, class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ write(uint8_t* buffer, T value) { return E==Endian::little ? little_endian::write(buffer,value) : big_endian::write(buffer,value); }


//
// Read from buffer
//

namespace big_endian
{

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==1, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==2, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[1];
    reinterpret_cast<uint8_t*>(&value)[1] = buffer[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==4, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[3];
    reinterpret_cast<uint8_t*>(&value)[1] = buffer[2];
    reinterpret_cast<uint8_t*>(&value)[2] = buffer[1];
    reinterpret_cast<uint8_t*>(&value)[3] = buffer[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==8, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[7];
    reinterpret_cast<uint8_t*>(&value)[1] = buffer[6];
    reinterpret_cast<uint8_t*>(&value)[2] = buffer[5];
    reinterpret_cast<uint8_t*>(&value)[3] = buffer[4];
    reinterpret_cast<uint8_t*>(&value)[4] = buffer[3];
    reinterpret_cast<uint8_t*>(&value)[5] = buffer[2];
    reinterpret_cast<uint8_t*>(&value)[6] = buffer[1];
    reinterpret_cast<uint8_t*>(&value)[7] = buffer[0];
    return sizeof(T);
}

} // namespace big_endian
namespace little_endian
{

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    std::memcpy(&value, buffer, sizeof(T));
    return sizeof(T);
}

} // namespace little_endian


template<Endian E, class T>
size_t read(const uint8_t* buffer, T& value)
{
    return (E==Endian::little) ? little_endian::read(buffer, value) : big_endian::read(buffer, value);
}

// Read T and return by value
template<Endian E, class T>
T read(const uint8_t* buffer)
{
    T value;
    read<E,T>(buffer, value);
    return value;
}

} // namespace serialization
} // namespace microstrain