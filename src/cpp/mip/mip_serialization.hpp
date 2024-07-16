#pragma once

#include <microstrain/common/serialization.hpp>

#include <type_traits>
#include <stddef.h>


namespace mip
{

using Serializer = microstrain::BigEndianSerializer;

namespace serialization
{
    using namespace microstrain::serialization;
    using namespace microstrain::serialization::big_endian;
}

//template<microstrain::serialization::Endian E, class... Ts>
//size_t insert(const Ts&... values)
//{
//    static_assert(E == microstrain::serialization::Endian::big);
//    return insert<Ts...>(values...);
//}


//
//template<class... Ts>
//size_t insert(Serializer& serializer, const Ts&... values)
//{
//    return serializer.insert(std::forward<Ts>(values)...);
//}
//
//template<class... Ts>
//size_t extract(Serializer& serializer, Ts&... values)
//{
//    return microstrain::extract(serializer, std::forward<Ts>(values)...);
//}
//
//template<class T>
//size_t extract_count(Serializer& buffer, T* count, size_t max_count)
//{
//    return microstrain::extract_count(buffer, count, max_count);
//}
//template<class T>
//size_t extract_count(Serializer& buffer, T& count, size_t max_count)
//{
//    return microstrain::extract_count(buffer, count, max_count);
//}


} // namespace mip


// These functions must be in the microstrain namespace in order to be seen as overloads.
namespace microstrain
{
} // namespace microstrain
