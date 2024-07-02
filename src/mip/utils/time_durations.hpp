#pragma once

namespace mip
{
    // TODO: Switch out for custom time durations.
    using std::chrono::duration_cast;
    using std::chrono::time_point;
    using Nanoseconds = std::chrono::nanoseconds;
    using Microseconds = std::chrono::microseconds;
    using Milliseconds = std::chrono::milliseconds;
    using Seconds = std::chrono::seconds;
    using Minutes = std::chrono::minutes;
    using Hours = std::chrono::hours;

#if _HAS_CXX20
    using Days = std::chrono::days;
    using Weeks = std::chrono::weeks;
    using Years = std::chrono::years;
#else
    #include <ratio>
    using Days = std::chrono::duration<int, std::ratio<86400>>;
    using Weeks = std::chrono::duration<int, std::ratio<604800>>;
    using Years = std::chrono::duration<int, std::ratio<31556952>>;
#endif // _HAS_CXX20
}