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

} // namespace mip


// These functions must be in the microstrain namespace in order to be seen as overloads.
namespace microstrain
{
} // namespace microstrain
