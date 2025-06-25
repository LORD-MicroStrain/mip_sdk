#pragma once

#include <microstrain/serialization.hpp>

#include <type_traits>
#include <stddef.h>


namespace mip
{

using Serializer = microstrain::BigEndianSerializer;

namespace serialization
{
    using namespace microstrain::serialization;
    using namespace microstrain::serialization::big_endian;

} // namespace serialization

using microstrain::insert;
using microstrain::extract;

// Explicitly overload insert/extract versions that take raw buffers
// since the endianness can't be deduced from the arguments.

template<class T>
inline bool extract(T& object, const uint8_t* buffer, size_t bufferLength, size_t offset=0, bool exactSize=false)
{
    using namespace microstrain::serialization;
    return extract<Endian::big>(object, buffer, bufferLength, offset, exactSize);
}

template<class T>
inline bool insert(const T& object, uint8_t* buffer, size_t bufferLength, size_t offset=0)
{
    using namespace microstrain::serialization;
    return insert<Endian::big>(object, buffer, bufferLength, offset);
}


} // namespace mip

