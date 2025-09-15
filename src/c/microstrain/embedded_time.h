#pragma once

#include "microstrain/platform.h"

#include <assert.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
namespace microstrain {
namespace C {
extern "C" {
#endif // __cplusplus

///@brief Type used for packet timestamps and timeouts.
///
/// Timestamps must be monotonic except for overflow at the maximum value back to 0.
/// The units can be anything, but typically are milliseconds. Choose a long enough
/// unit so that consecutive calls to the parser will not exceed half of the maximum
/// value for this type. For milliseconds, the time to overflow is approximately 50
/// days, so the parser should be invoked at least every 25 days. Failure to observe
/// this requirement may result in false timeouts or delays in getting parsed packets.
///
#ifdef MICROSTRAIN_TIMESTAMP_TYPE
typedef MICROSTRAIN_TIMESTAMP_TYPE microstrain_embedded_timestamp;
static_assert(sizeof(microstrain_embedded_timestamp) >= 8 || (microstrain_embedded_timestamp)-1 > 0,
    "MICROSTRAIN_TIMESTAMP_TYPE must be unsigned unless 64 bits.");
#elif defined(MICROSTRAIN_PLATFORM_DESKTOP)
// By default, on desktop we use 64-bit timestamps.
typedef uint64_t microstrain_embedded_timestamp;
#else
// If the platform isn't known, assume u32 timestamps.
typedef uint32_t microstrain_embedded_timestamp;
#endif // MICROSTRAIN_TIMESTAMP_TYPE

// Timeouts are just an alias for timestamps
typedef microstrain_embedded_timestamp microstrain_embedded_timeout;

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the current system timestamp in milliseconds
///
/// @details Provides basic timestamping using system time:
///          - Returns milliseconds since Unix epoch
///          - Uses timespec_get() with UTC time base
///          - Returns 0 if time cannot be obtained
///
/// @return Current system time in milliseconds since epoch
///
static inline microstrain_embedded_timestamp microstrain_get_current_timestamp()
{
    struct timespec ts;

    // Get system UTC time since epoch
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC)
    {
        return 0;
    }

    // Get the time in milliseconds
    return (microstrain_embedded_timestamp)ts.tv_sec * 1000 + (microstrain_embedded_timestamp)ts.tv_nsec / 1000000;
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace microstrain
#endif // __cplusplus
