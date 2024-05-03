#pragma once

#if defined _WIN32
#define MICROSTRAIN_PLATFORM_WINDOWS
#elif defined __APPLE__
#define MICROSTRAIN_PLATFORM_APPLE
#elif defined __linux__
#define MICROSTRAIN_PLATFORM_LINUX
#else
#define MICROSTRAIN_PLATFORM_OTHER
#endif

#include <stdint.h>



//#ifdef __cplusplus
//namespace microstrain {
//#endif
//
/////@brief Type used for packet timestamps and timeouts.
/////
///// Timestamps must be monotonic except for overflow at the maximum value back to 0.
///// The units can be anything, but typically are milliseconds. Choose a long enough
///// unit so that consecutive calls to the parser will not exceed half of the maximum
///// value for this type. For milliseconds, the time to overflow is approximately 50
///// days, so the parser should be invoked at least every 25 days. Failure to observe
///// this requirement may result in false timeouts or delays in getting parsed packets.
/////
//#ifdef MICROSTRAIN_TIMESTAMP_TYPE
//typedef MICROSTRAIN_TIMESTAMP_TYPE microstrain_timestamp;
//    static_assert( sizeof(microstrain_timestamp) >= 8 || microstrain_timestamp(-1) > 0, "MICROSTRAIN_TIMESTAMP_TYPE must be unsigned unless 64 bits.");
//#else
//typedef uint64_t microstrain_timestamp;
//#endif
//
//typedef microstrain_timestamp microstrain_timeout;
//
//
//#ifdef __cplusplus
//
//} // extern "C"
//} // namespace C
//
//using Timestamp      = C::mip_timestamp;
//using Timeout        = C::mip_timeout;
//
//} // namespace microstrain
