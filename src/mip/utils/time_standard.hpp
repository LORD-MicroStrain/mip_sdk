#pragma once

#include <chrono>

namespace mip
{
    // TODO: Add more comprehensive documentation.
    /// Extend this to create a new time standard.
    struct TimeStandard
    {
        virtual std::chrono::nanoseconds now() const = 0;
        virtual std::chrono::nanoseconds convertToBase(std::chrono::nanoseconds time) const = 0;
        virtual std::chrono::nanoseconds convertFromBase(std::chrono::nanoseconds time) const = 0;
    };
    
    /** Common standards ****************************************************************/

    struct UnixTime : TimeStandard
    {
        static const UnixTime instance;
    };

    struct GpsTime : TimeStandard
    {

    };
} // namespace mip