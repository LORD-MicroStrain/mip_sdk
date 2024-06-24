#pragma once

#include <microstrain/common/serialization.hpp>

#include <type_traits>
#include <stddef.h>


namespace mip
{
} // namespace mip


// These functions must be in the microstrain namespace in order to be seen as overloads.
namespace microstrain
{
//
// Mip field types (commands and data)
//

template<class T>
typename std::enable_if<mip::isField<T>::value, size_t>::type
/*size_t*/ insert(microstrain::Serializer& serializer, const T& field)
{
    //static_assert(std::is_base_of<mip::FieldStruct<T>, T>::value, "T doesn't derive from FieldStruct");

    return insert(serializer, field.asTuple());
}

template<class T>
typename std::enable_if<mip::isField<T>::value, size_t>::type
/*size_t*/ extract(microstrain::Serializer& serializer, T& field)
{
    //static_assert(std::is_base_of<mip::FieldStruct<T>, T>::value, "T doesn't derive from FieldStruct");

    return extract(serializer, field.asTuple());
}

//
// Bitfields
//

template<class T>
typename std::enable_if<std::is_base_of<mip::Bitfield<T>, T>::value, size_t>::type
/*size_t*/ insert(Serializer& serializer, T bits)
{
    return insert(serializer, bits.value);
}

template<class T>
typename std::enable_if<std::is_base_of<mip::Bitfield<T>, T>::value, size_t>::type
/*size_t*/ extract(Serializer& serializer, T& bits)
{
    return extract(serializer, bits.value);
}


} // namespace microstrain
