#pragma once

#include <chrono>
#include <ratio>

namespace mip
{
    using Nanoseconds  = std::chrono::duration<std::uint64_t, std::nano>;
    using Microseconds = std::chrono::duration<std::uint64_t, std::micro>;
    using Milliseconds = std::chrono::duration<std::uint64_t, std::milli>;
    using Seconds      = std::chrono::duration<std::uint64_t>;
    using Minutes      = std::chrono::duration<std::uint32_t, std::ratio<60>>;
    using Hours        = std::chrono::duration<std::uint32_t, std::ratio<3600>>;
    using Days         = std::chrono::duration<std::uint32_t, std::ratio<86400>>;
    using Weeks        = std::chrono::duration<std::uint32_t, std::ratio<604800>>;
    using Years        = std::chrono::duration<std::uint32_t, std::ratio<31556952>>;

    // TODO: Switch out for custom time durations.
//     using std::chrono::duration_cast;
//     using std::chrono::time_point;
    // using Nanoseconds = std::chrono::nanoseconds;
//     using Microseconds = std::chrono::microseconds;
//     using Milliseconds = std::chrono::milliseconds;
//     using Seconds = std::chrono::seconds;
//     using Minutes = std::chrono::minutes;
//     using Hours = std::chrono::hours;

// #if _HAS_CXX20
//     using Days = std::chrono::days;
//     using Weeks = std::chrono::weeks;
//     using Years = std::chrono::years;
// #else
//     #include <ratio>
//     using Days = std::chrono::duration<int, std::ratio<86400>>;
//     using Weeks = std::chrono::duration<int, std::ratio<604800>>;
//     using Years = std::chrono::duration<int, std::ratio<31556952>>;
// #endif // _HAS_CXX20
}