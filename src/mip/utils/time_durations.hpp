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
}