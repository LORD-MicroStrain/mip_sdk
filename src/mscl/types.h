#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl{
namespace C {
extern "C" {
#endif



// Used like a signed version of size_t
typedef int_least16_t remaining_count;


///@brief Type used for packet timestamps and timeouts.
///
/// Timestamps must be monotonic except for overflow at the maximum value back to 0.
/// The units can be anything, but typically are milliseconds. Choose a long enough
/// unit so that consecutive calls to the parser will not exceed half of the maximum
/// value for this type. For milliseconds, the time to overflow is approximately 50
/// days, so the parser should be invoked at least every 25 days. Failure to observe
/// this requirement may result in false timeouts or delays in getting parsed packets.
///
typedef uint32_t       timestamp_type;
typedef timestamp_type timeout_type;

#ifdef __cplusplus

} // extern "C"
} // namespace C

using RemainingCount = C::remaining_count;
using Timestamp      = C::timestamp_type;
using Timeout        = C::timeout_type;

} // namespace mscl

#endif
